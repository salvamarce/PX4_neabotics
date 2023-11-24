/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file FlightTaskLamaPosition.cpp
 */

#include "FlightTaskLamaPosition.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskLamaPosition::updateInitialize()
{
	bool ret = FlightTaskManualPosition::updateInitialize();
	// require valid position / velocity in xy
	return ret;
}

bool FlightTaskLamaPosition::activate(const trajectory_setpoint_s &last_setpoint)
{
	// all requirements from altitude-mode still have to hold
	_first_position_setpoint = last_setpoint;
	_prev_position_setpoint = Vector3f(_first_position_setpoint.position);
	_idle_position_setpoint = _prev_position_setpoint;
	bool ret = FlightTaskManualPosition::activate(last_setpoint);
	return ret;
}

bool FlightTaskLamaPosition::update(){
	bool ret = FlightTaskManualPosition::update();
	if(!ret)
		return false;

	if (_vehicle_attitude_setpoint_sub.updated()) {
		vehicle_attitude_setpoint_s vehicle_attitude_setpoint;
		if (_vehicle_attitude_setpoint_sub.copy(&vehicle_attitude_setpoint)
			    && (vehicle_attitude_setpoint.timestamp > _vehicle_attitude_setpoint.timestamp))

			_vehicle_attitude_setpoint = vehicle_attitude_setpoint;
	}

	if (_approaching_activation_sub.updated()) {
		approaching_activation_s approaching_activation;
		if (_approaching_activation_sub.copy(&approaching_activation)
			    && (approaching_activation.timestamp > _approaching_activation.timestamp))
			
			_approaching_activation = approaching_activation;
	}


	_readSensors();

	_handleStateTransitions();

	// auto landing gear
	// ...

	// log
	// ...

	// debug data on mavlink
	// ...

	switch(_currentState){
		case LamaState::IDLE:
			_idleMode();
			break;
		case LamaState::APPROACH:
			_approachMode();
			break;
		case LamaState::INTERACTION:
			_interactionMode();
			break;
		case LamaState::LEAVING:
			_leavingMode();
			break;
	}

	if(_state_sub.updated()){
		lama_state_s temp_state;
		if (_state_sub.copy(&temp_state)
			    && (temp_state.timestamp > _lama_state.timestamp))
			_lama_state = temp_state;	// save flags and fields not controlled by this module
	}
	_lama_state.state = (uint8_t)_currentState;
	_state_pub.publish(_lama_state);

	// save last setpoint
	_prev_position_setpoint = _position_setpoint;

	return true;
}


void FlightTaskLamaPosition::_readSensors(){
	if(_concrete_tool_data_sub.updated()){
		concrete_tool_data_s data;
		if(_concrete_tool_data_sub.copy(&data) && (data.timestamp > _tool_data.timestamp)){
			_tool_data = data;

			// Check if tof measure is reliable
			_tofMeasureOk = false;
			for(int i=0; i<4; ++i){
				if(_tool_data.distance[i] >= _param_tof_max_dist.get() || _tool_data.distance[i] <= 0.05f){
					//PX4_WARN("tof not ok");
					return ;
				}
			}
			_tofMeasureOk = true;

			// Compute distance error d
			float dist = 0;
			for(int i=0; i<4; ++i)
				dist += _tool_data.distance[i];
			_avgDist = dist / 4.0f;	// average distance read from tof sensors

			//PX4_INFO("dist = %f", (double)_avgDist);
		}
	}
}


void FlightTaskLamaPosition::_idleMode(){
	//PX4_INFO("[IDLE]");
	Vector2f eps(0.25f, 0.0f);
	Sticks::rotateIntoHeadingFrameXY(eps, _yaw, NAN);
	_position_setpoint = _idle_position_setpoint;
	_position_setpoint.xy() += eps;
	_velocity_setpoint.setZero();
	
}

void FlightTaskLamaPosition::_approachMode(){
	if(!_tofMeasureOk){
		PX4_ERR("Non dovresti essere qui");
		return;
	}

	//PX4_INFO("\t[APPR]");

	// Remove offset between tof sensors and contact surface
	float dist = _avgDist - _param_tof_d_offset.get();

	// Rotate d wrt body pitch
	float d_x = dist * cos(_vehicle_attitude_setpoint.pitch_body);
	float d_z = dist * sin(_vehicle_attitude_setpoint.pitch_body);

	// Gains
	float k = _param_approach_max_vel.get() / _param_tof_max_dist.get();
	//float k_z = _param_approach_max_vel_z.get() / _param_tof_max_dist.get();

	// Feedforward velocity setpoint
	Vector2f vel_sp_xy (k * d_x, 0.0f);
	Sticks::rotateIntoHeadingFrameXY(vel_sp_xy, _yaw, NAN);
	_velocity_setpoint.xy() = vel_sp_xy;
	_velocity_setpoint(2) = k * d_z;

	// Position setpoints
	Vector2f pos_sp_xy;
	pos_sp_xy(0) = k * d_x * _deltatime;
	pos_sp_xy(1) = 0.0f;		// y constant
	Sticks::rotateIntoHeadingFrameXY(pos_sp_xy, _yaw, NAN);

	//_position_setpoint.xy() += pos_sp_xy;	// first_pose inutile?
	_position_setpoint(0) = NAN;
	_position_setpoint(1) = NAN;
	_position_setpoint(2) = _prev_position_setpoint(2) - _velocity_setpoint(2) * _deltatime;

	if(_avgDist < 0.05f){
		wasNearWall = true;
		//PX4_WARN("wasNearWall = true!");
	}else if(_avgDist > 0.5f){
		wasNearWall = false;
		//PX4_WARN("wasNearWall = false!");
	}
	
	//PX4_INFO("v_sp:%3.3f dx_sp:%3.3f\t=>\tvx_sp:%3.3f\tvy_sp:%3.3f; x_sp:%3.3f\ty_sp:%3.3f",
	//	(double)(k_x * d_x), (double)(k_x*d_x*_deltatime), (double)vel_sp_xy(0), (double)vel_sp_xy(1), (double)_position_setpoint(0), (double)_position_setpoint(1));
	
}

void FlightTaskLamaPosition::_interactionMode(){
	
}

void FlightTaskLamaPosition::_leavingMode(){
	
}


void FlightTaskLamaPosition::_handleStateTransitions(){

	switch(_currentState){

		case LamaState::IDLE:
			// switch to approach if yaw and pitch error is less than threshold
			//if(_approaching_activation.active_approach)
			if(!approachOnlyOnce && !wasNearWall && _tofMeasureOk && _avgDist <= 1.8f){
				PX4_WARN("Switch into approach");
				_currentState = LamaState::APPROACH;
				approachOnlyOnce = true;
			}
			break;


		case LamaState::APPROACH:
			// switch back to idle if active_approach is false
			if(!_tofMeasureOk){
				_currentState = LamaState::IDLE;
				_idle_position_setpoint = _position;
				PX4_WARN("SWITCH IN IDLE:\t%3.3f\t%3.3f\t%3.3f", (double)_idle_position_setpoint(0), (double)_idle_position_setpoint(1), (double)_idle_position_setpoint(2));
			}else if(wasNearWall && _avgDist > 0.1f){
				PX4_WARN("MOVING! BACK TO IDLE");
				_currentState = LamaState::IDLE;
			}
			
			//if(!_approaching_activation.active_approach)
			//	_currentState = LamaState::IDLE;
			// switch to interaction if ...
			//else if(...)
			//	_currentState = LamaState::INTERACTION;
			break;
		

		case LamaState::INTERACTION:
			break;
		

		case LamaState::LEAVING:
			break;

	}
}