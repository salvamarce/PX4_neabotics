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

#include <drivers/drv_hrt.h>

#include "MulticopterInteractionControl.hpp"

MulticopterInteractionControl::MulticopterInteractionControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_updated();
}

MulticopterInteractionControl::~MulticopterInteractionControl()
{
	perf_free(_loop_perf);
}

bool
MulticopterInteractionControl::init()
{
	// execute Run() on every concrete_tool_data publication
	if (!_concrete_tool_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}
void
MulticopterInteractionControl::parameters_updated()
{
	// Store some of the parameters in a more convenient way & precompute often-used values
	_force_setpoint = _param_lama_force_sp.get();
	_Kp_gain = _param_lama_force_Kp.get();
	_Ki_gain = _param_lama_force_Ki.get();
	_min_interaction_force = _param_lama_min_int_force.get();

}

void MulticopterInteractionControl::Run()
{
	if (should_exit()) {
		_concrete_tool_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();
	}

	// Update control mode
	_vehicle_control_mode_sub.update(&_vehicle_control_mode);

	//  Run on concrete_tool_data updates
	if(_concrete_tool_sub.updated()){

		// Run only if in LAMA flight mode
		if (_vehicle_control_mode.flag_control_lama_enabled){

			concrete_tool_data_s concrete_tool_data;

			if(_concrete_tool_sub.copy(&concrete_tool_data)
			   &&(concrete_tool_data.timestamp > _last_concrete_tool_data_time)){
			   //concrete_tool_data.timestamp_force > _last_concrete_tool_data_time){

				if(concrete_tool_data.force[0] > _min_interaction_force){

					float dt = math::constrain(((hrt_absolute_time()  - _last_concrete_tool_data_time) * 1e-6f), 0.0002f, 0.02f);

					float force_error = _force_setpoint - concrete_tool_data.force[0];

					if ( (force_error * dt) != NAN)
						_force_int += _Ki_gain * force_error * dt;

					float jerk_sp = _Kp_gain * force_error + _force_int;

					PX4_INFO("jerk: %3.3f", (double)jerk_sp);

					_last_concrete_tool_data_time = hrt_absolute_time();
				}

			}

		}
		else
			resetIntegral();

	}


	perf_end(_loop_perf);
}

int MulticopterInteractionControl::task_spawn(int argc, char *argv[])
{
	MulticopterInteractionControl *instance = new MulticopterInteractionControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterInteractionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterInteractionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements the multicopter interaction controller. It takes force
setpoint from parameters as inputs and outputs a thrust setpoint along Z and the
servo setpoints for the tilting drone.

The controller has a PI loop for force error

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_int_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Multicopter interaction control app start / stop handling function
 */
extern "C" __EXPORT int mc_int_control_main(int argc, char *argv[])
{
	return MulticopterInteractionControl::main(argc, argv);
}
