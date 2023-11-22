/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/concrete_tool_data.h>
#include <uORB/topics/tilting_servo_sp.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>


using namespace time_literals;

class MulticopterInteractionControl : public ModuleBase<MulticopterInteractionControl>, public ModuleParams, public px4::WorkItem
{
public:
	MulticopterInteractionControl();
	~MulticopterInteractionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * Set the integral term in x to 0.
	 * @see _force_int
	 */
	void resetIntegral() { _force_int = 0.0f; }

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	// Publications
	uORB::Publication<vehicle_rates_setpoint_s> _thrust_sp_pub{ORB_ID(interaction_rates_setpoint)};
	uORB::Publication<tilting_servo_sp_s> _servo_sp_pub{ORB_ID(interaction_servo_setpoint)};

	// Subscriptions
	// The frequency of this module will be given by the force feedback
	uORB::SubscriptionCallbackWorkItem _concrete_tool_sub{this, ORB_ID(concrete_tool_data)}; // subscription that schedules MulticopterInteractionControl when updated
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription _attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	perf_counter_t  _loop_perf;             /**< loop duration performance counter */

	vehicle_control_mode_s _vehicle_control_mode;

	hrt_abstime _last_concrete_tool_data_time{0};

	float _force_setpoint;
	float _force_int;
	float _Kp_gain, _Ki_gain;
	float _min_interaction_force;

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::LAMA_FORCE_SP>) _param_lama_force_sp,   /** Force setpoint for interaction */
		(ParamFloat<px4::params::LAMA_FORCE_KP>) _param_lama_force_Kp,   /** Force control proportional gain */
		(ParamFloat<px4::params::LAMA_FORCE_KI>) _param_lama_force_Ki,   /** Force control proportional gain */
		(ParamFloat<px4::params::LAMA_MIN_FORCE>) _param_lama_min_int_force /** Minimum force value to start the control */
	)

};
