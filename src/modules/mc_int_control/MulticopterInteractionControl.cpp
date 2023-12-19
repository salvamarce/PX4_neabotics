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
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_updated();

	lama_state.engage_interaction = false;
	lama_state.engage_approach = false;
	lama_state.timestamp = 0;
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
	_limit_integral_error = _param_limit_integral_error.get();

	// Per il calcolo della forza massima potrei prendere i Kf del motore,
	// ma non sapendo la velocità massima, non posso calcolare la forza massima esplicabile
	// char buffer[17];
	// int i = 0;

	// do{
	// 	snprintf(buffer, sizeof(buffer), "CA_ROTOR%u_CT", i);
	// 	_param_handles[i].thrust_coef = param_find(buffer);
	// 	i++;
	// }while();

}

void MulticopterInteractionControl::Run()
{
	if (should_exit()) {
		_concrete_tool_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	ScheduleDelayed(100_ms);

	perf_begin(_loop_perf);

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();
	}

	// Update useful variables
	if(_control_mode_sub.updated())
		_control_mode_sub.update(&_control_mode);

	if(_attitude_setpoint_sub.updated()){
		if(_attitude_setpoint_sub.copy(&_attitude_setpoint)){
			float theta = _attitude_setpoint.pitch_body;

			_R_b(0,0) = cosf(theta); _R_b(0,1) = 0.0f; _R_b(0,2) = sin(theta);
			_R_b(1,0) = 0.0f; 	 _R_b(1,1) = 1.0f; _R_b(1,2) = 0.0f;
			_R_b(2,0) = -sin(theta); _R_b(2,1) = 0.0f; _R_b(2,2) = cos(theta);
		}
	}

	if(_rates_setpoint_sub.updated())
		_rates_setpoint_sub.update(&_rates_setpoint);

	if(_servo_setpoint_sub.updated())
		_servo_setpoint_sub.update(&_attitude_servo_sp);

	//  Run on concrete_tool_data updates
	if(_concrete_tool_sub.updated()){

		// Run only if in LAMA flight mode
		if(_control_mode.flag_control_lama_enabled){

			concrete_tool_data_s concrete_tool_data;

			//lama_state_s lama_state;
			_lama_state_sub.update(&lama_state);

			lama_state.engage_interaction = false;

			if(_concrete_tool_sub.copy(&concrete_tool_data)
			   && concrete_tool_data.timestamp_load > _last_concrete_tool_data_time){

				_last_concrete_tool_data_time = hrt_absolute_time();

				if(concrete_tool_data.force[0] > _min_interaction_force){

					float dt = math::constrain(((hrt_absolute_time()  - _last_concrete_tool_data_time) * 1e-6f), 0.0002f, 0.02f);

					float force_error = _force_setpoint - concrete_tool_data.force[0];

					if ( (force_error * dt) != NAN){
						_force_int += _Ki_gain * force_error * dt;
						_force_int = math::constrain(_force_int, -_limit_integral_error, _limit_integral_error);
					}

					float delta_f = (_Kp_gain * force_error + _force_int) * dt;

					float fx = _old_f_sp(0) + delta_f;
					float servo_angle =  asinf(fx/_old_f_sp(2));

					float fz = _old_f_sp(2) + delta_f * cosf(servo_angle);

					// PX4_INFO("fz, max, servo: \t %3.3f \t %3.3f \t %3.3f", (double)(2.0f*2.5f*CONSTANTS_ONE_G), (double)(fz/(2.0f*2.5f*CONSTANTS_ONE_G)), (double)math::degrees(servo_angle));

					vehicle_rates_setpoint_s rates_sp;

					rates_sp = _rates_setpoint;
					rates_sp.timestamp = hrt_absolute_time();
					rates_sp.thrust_body[2] = -fz/(THRUST_Z_MAX);

					tilting_servo_sp_s servo_sp;
					servo_sp.timestamp = hrt_absolute_time();
					servo_sp.angle[0] = servo_angle;
					servo_sp.angle[1] = 0.0f;
					servo_sp.angle[2] = 0.0f;
					servo_sp.angle[3] = 0.0f;

					_thrust_setpoint_pub.publish(rates_sp);
					_servo_setpoint_pub.publish(servo_sp);

					if(lama_state.state == lama_state_s::APPROACH){
						// Force slightly smaller than interaction_force and greater than the approach setpoint is expected
						if(abs(concrete_tool_data.force[0]) > _param_min_int_force.get() - 1.5f * _param_interaction_force_eps.get()){
							// PX4_INFO("dentro");

							if(1e-6f*(hrt_absolute_time()-_last_interaction_time) > _param_interaction_enable_delay.get()){
								lama_state.engage_interaction = true;

								// PX4_WARN("Interaction ok");
							}
						}
						else{
							// PX4_INFO("fuori");
							_last_interaction_time = hrt_absolute_time();
						}
					}

				}
				else{
					_last_interaction_time = hrt_absolute_time();
					resetIntegral();
				}

			}
			lama_state.timestamp = hrt_absolute_time();
			_lama_state_pub.publish(lama_state);

		}
		else{
			resetIntegral();
			_last_interaction_time = hrt_absolute_time();

			// fx = Fz * cos(90-alpha) = Fz * sin(alpha)
			_old_f_sp(0) = -_rates_setpoint.thrust_body[2] * THRUST_Z_MAX * sinf(_attitude_servo_sp.angle[0]);
			_old_f_sp(1) = 0.0f;
			_old_f_sp(2) = -_rates_setpoint.thrust_body[2] * THRUST_Z_MAX; // Fz is negative because is in NED
			// PX4_INFO("att fx, fz, servo: \t %3.3f \t %3.3f \t %3.3f", (double)_old_f_sp(0), (double)_old_f_sp(2), (double)math::degrees(_attitude_servo_sp.angle[0]));

		}

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
