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

FlightTaskLamaPosition::FlightTaskLamaPosition(){
	_lama_state.engage_interaction = false;
	_lama_state.engage_approach = false;
	_lama_state.timestamp = 0;
	_tool_data.timestamp = 0;
}

bool FlightTaskLamaPosition::_checkActivationConditions(){
	return true;

	// Require valid data from tofs
	if(_concrete_tool_data_sub.advertised()){
		concrete_tool_data_s data;
		_concrete_tool_data_sub.update(&data);
		if(hrt_absolute_time() - data.timestamp_tof > 100_ms)
			return false;
		return true;
	}
	return false;
}

bool FlightTaskLamaPosition::updateInitialize()
{
	bool ret = FlightTaskManualPosition::updateInitialize();
	return ret && _checkActivationConditions();
}

bool FlightTaskLamaPosition::activate(const trajectory_setpoint_s &last_setpoint)
{
	// all requirements from altitude-mode still have to hold
	_first_position_setpoint = last_setpoint;
	_prev_position_setpoint = Vector3f(_first_position_setpoint.position);
	_idle_position_setpoint = _prev_position_setpoint;

	bool ret = FlightTaskManualPosition::activate(last_setpoint);
	return ret && _checkActivationConditions();
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

	// reset sticks setpoint
	_position_setpoint.setNaN();
	_velocity_setpoint.setNaN();
	_acceleration_setpoint.setNaN();
	_yaw_setpoint = NAN;
	_yawspeed_setpoint = 0;

	_readSensors();

	_handleStateTransitions();

	// Auto landing gear
	// ...


	// Debug data on mavlink
	/*for(int i=0; i<4; ++i)
		_log_tool_data.data[i] = _tool_data.distance[i];
	_log_tool_data.data[4] = _tool_data.force[0];
	_log_tool_data.data[5] = _tool_data.torque[1];
	_log_tool_data.data[6] = _tool_data.torque[2];*/


	// State machine
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

	// Save last setpoint
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
				if(_tool_data.distance[i] >= _param_tof_max_dist.get() || _tool_data.distance[i] <= _param_tof_min_dist.get()){
					//PX4_WARN("tof %d not ok", i);
					return;
				}
			}
			_tofMeasureOk = true;

			// Compute distance error d
			float dist = 0;
			for(int i=0; i<4; ++i)
				dist += _tool_data.distance[i];
			_avgDist = dist / 4.0f;	// average distance read from tof sensors

			// Compute yaw and pitch
			float left_mean = 0.5f*(_tool_data.distance[concrete_tool_data_s::TOP_LEFT] +
						_tool_data.distance[concrete_tool_data_s::BOTTOM_LEFT]);

			float right_mean = 0.5f*(_tool_data.distance[concrete_tool_data_s::TOP_RIGHT] +
						_tool_data.distance[concrete_tool_data_s::BOTTOM_RIGHT]);

			float bottom_mean = 0.5f*(_tool_data.distance[concrete_tool_data_s::BOTTOM_LEFT] +
						_tool_data.distance[concrete_tool_data_s::BOTTOM_RIGHT]);

			float top_mean = 0.5f*(_tool_data.distance[concrete_tool_data_s::TOP_LEFT] +
						_tool_data.distance[concrete_tool_data_s::TOP_RIGHT]);

			_tof_yaw = atan2(left_mean-right_mean,_param_concrete_tool_y_dist.get());
			_tof_pitch = atan2(bottom_mean-top_mean, _param_concrete_tool_z_dist.get());
		}
	}
}


void FlightTaskLamaPosition::_idleMode(){
	_position_setpoint = _idle_position_setpoint;
	_velocity_setpoint.setZero();
}

void FlightTaskLamaPosition::_approachMode(){
	// Used to ignore sticks command in all cases
	_position_setpoint = _prev_position_setpoint;
	_velocity_setpoint.setZero();

	if(!_tofMeasureOk && !wasNearWall){
		PX4_ERR("Lost tof measure during approach");
		return;
	}

	_pushing_setpoint_saved = true;
	// Push towards the wall waiting for interaction phase
	if(!_tofMeasureOk || _avgDist < 0.02f){
		//PX4_INFO("pushing");
		_pushing_setpoint_saved = false;
		Vector2f eps((_param_interaction_push_force.get() - _param_interaction_force_eps.get())/9.8f, 0.0f);
		Sticks::rotateIntoHeadingFrameXY(eps, _yaw, NAN);
		/*_position_setpoint = _pushing_position_setpoint;
		_position_setpoint.xy() += eps;
		_velocity_setpoint.setZero();	*/
		_position_setpoint = _position;
		_acceleration_setpoint.xy() = eps;
	}

	// Approach the wall
	else {

		// Rotate d wrt body pitch
		float d_x = _avgDist * cosf(_vehicle_attitude_setpoint.pitch_body);
		float d_z = _avgDist * sinf(_vehicle_attitude_setpoint.pitch_body);

		// Gains
		float k = _param_approach_max_vel.get() / _param_tof_max_dist.get();

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


		if(_param_approach_send_pos_sp.get()){
			_position_setpoint(0) = _prev_position_setpoint(0) + _velocity_setpoint(0) * _deltatime;
			_position_setpoint(1) = _prev_position_setpoint(1) + _velocity_setpoint(1) * _deltatime;
		}else{
			_position_setpoint(0) = NAN;
			_position_setpoint(1) = NAN;
		}
		_position_setpoint(2) = _prev_position_setpoint(2) - _velocity_setpoint(2) * _deltatime;
	}

	if(_pushing_setpoint_saved)
		_pushing_position_setpoint = _position_setpoint;


	if(_avgDist < 0.1f)
		wasNearWall = true;
	else if(_avgDist > 0.3f && _avgDist < 0.4f)
		wasNearWall = false;
}

void FlightTaskLamaPosition::_interactionMode(){
	_position_setpoint = _interaction_position_setpoint;
	_velocity_setpoint.setZero();
}

void FlightTaskLamaPosition::_leavingMode(){

}


void FlightTaskLamaPosition::_handleStateTransitions(){

	switch(_currentState){

		case LamaState::IDLE:
						// Switch to approach if angle error ok and pitch sticks up
			if(_tofMeasureOk && _lama_state.engage_approach && _sticks.getPitch() > 0.75f){
				PX4_WARN("Switch into approach");
				_currentState = LamaState::APPROACH;
			}
				//PX4_WARN("tofMeasureOk: %d\tengage_approach: %d\tstick_pitch: %f", _tofMeasureOk, _lama_state.engage_approach, (double)_sticks.getPitch());

			break;


		case LamaState::APPROACH:
			// switch back to idle if active_approach is false
			if(!_tofMeasureOk && !wasNearWall){
				_currentState = LamaState::IDLE;
				_idle_position_setpoint = _position;
				PX4_WARN("Switch into idle");
			}
			else if (_lama_state.engage_interaction){
				PX4_WARN("Switch into interaction");
				_interaction_position_setpoint = _position;
				_currentState = LamaState::INTERACTION;
			}
			else if(wasNearWall && _avgDist > 0.25f){	// wasNearWall simulation-only
				PX4_WARN("MOVING! BACK TO IDLE");
				wasNearWall = false;
				_currentState = LamaState::IDLE;
			}

			break;


		case LamaState::INTERACTION:
			// when pitch stick down, disapproach
			if(_sticks.getPitch() < -0.75f){
				PX4_WARN("Switch into disapproach");
				_currentState = LamaState::LEAVING;
			}
			break;


		case LamaState::LEAVING:
			break;

	}
}
