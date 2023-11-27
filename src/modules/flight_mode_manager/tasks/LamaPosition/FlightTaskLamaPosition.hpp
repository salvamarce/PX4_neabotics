/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskLamaPosition.hpp
 *
 * Flight task for LAMA position controlled mode.
 *
 */

#pragma once

#include "FlightTaskManualPosition.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/topics/concrete_tool_data.h>
#include <uORB/topics/approaching_activation.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/lama_state.h>
#include <uORB/topics/debug_array.h>

class FlightTaskLamaPosition : public FlightTaskManualPosition
{
public:
	FlightTaskLamaPosition() = default;
	virtual ~FlightTaskLamaPosition() = default;
	bool activate(const trajectory_setpoint_s &last_setpoint) override;
	bool updateInitialize() override;
	bool update() override;

private:
	void _readSensors();
	void _idleMode();
	void _approachMode();
	void _interactionMode();
	void _leavingMode();
	void _handleStateTransitions();


private:
	uORB::Subscription _concrete_tool_data_sub {ORB_ID(concrete_tool_data)};
	struct concrete_tool_data_s _tool_data;

	uORB::Subscription _vehicle_attitude_setpoint_sub {ORB_ID(vehicle_attitude_setpoint)};
	struct vehicle_attitude_setpoint_s _vehicle_attitude_setpoint;

	uORB::Subscription _approaching_activation_sub {ORB_ID(approaching_activation)};
	struct approaching_activation_s _approaching_activation;

	uORB::Subscription _state_sub {ORB_ID(lama_state)};
	uORB::Publication<lama_state_s> _state_pub {ORB_ID(lama_state)};
	lama_state_s _lama_state;

	trajectory_setpoint_s _first_position_setpoint;
	matrix::Vector3f _prev_position_setpoint;
	matrix::Vector3f _idle_position_setpoint;

	bool _tofMeasureOk {false};
	float _tofAvgDistance {0.0f};

	float _tof_yaw{0};
	float _tof_pitch{0};

	float _avgDist{0};
	bool wasNearWall{false};

	bool _pushing_setpoint_saved{true};
	matrix::Vector3f _pushing_position_setpoint;

	struct debug_array_s _log_tool_data;
	uORB::Publication<debug_array_s> _log_tool_data_pub{ORB_ID(debug_array)};


	enum class LamaState : uint8_t{
		IDLE = 0,
		APPROACH = 1,
		INTERACTION = 2,
		LEAVING = 3
	};
	LamaState _currentState {LamaState::IDLE};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::APPR_MAX_VEL>)			_param_approach_max_vel,
		(ParamFloat<px4::params::APPR_PUSH_FORCE>) 		_param_approach_push_force,
		(ParamFloat<px4::params::TOF_MAX_DIST>) 		_param_tof_max_dist,
		(ParamFloat<px4::params::TOF_D_OFFSET>) 		_param_tof_d_offset,
		(ParamBool<px4::params::APPR_SEND_POS_SP>)		_param_approach_send_pos_sp,
		(ParamFloat<px4::params::CONC_TOOL_Y_DIST>) 	_param_concrete_tool_y_dist,
		(ParamFloat<px4::params::CONC_TOOL_Z_DIST>) 	_param_concrete_tool_z_dist
	)

};
