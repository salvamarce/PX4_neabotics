/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file mc_int_control_params.c
 * Parameters for multicopter interaction controller.
 *
 * @author Salvatore Marcellini <salvatore.marcellini@gmail.com>
 */

/**
 * Interaction force setpoint
 *
 * Force setpoint for the interaction control.
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Lama
 */
PARAM_DEFINE_FLOAT(LAMA_FORCE_SP, 15.0f);


/**
 * Difference between interaction force setpoint and interaction activation setpoint
 *
 * @min 0.0
 * @max 100.0
 * @decimal 2
 * @increment 0.01
 * @group Lama
 */
PARAM_DEFINE_FLOAT(LAMA_FORCE_EPS, 5.0f);

/**
 * Kp force control gain
 *
 * Force control proportional gain.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group Lama
 */
PARAM_DEFINE_FLOAT(LAMA_FORCE_KP, 1.0f);

/**
 * Ki force control gain
 *
 * Force control integral gain.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.001
 * @group Lama
 */
PARAM_DEFINE_FLOAT(LAMA_FORCE_KI, 0.001f);

/**
 * Minimum interaction force measure
 *
 * Minimum force value of the measured force to start the interaction controller
 *
 * @min 0.0
 * @max 20.0
 * @decimal 2
 * @increment 0.01
 * @group Lama
 */
PARAM_DEFINE_FLOAT(LAMA_MIN_FORCE, 2.0f);

/**
 * Integral force error limit
 *
 * Upper and lower limit for integral force error of the interaction controller
 *
 * @min 0.0
 * @max 25.0
 * @decimal 2
 * @increment 0.01
 * @group Lama
 */
PARAM_DEFINE_FLOAT(LAMA_LIMIT_INT, 15.0f);


/**
 * Minimum desired force to start the interaction
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group Lama
 */
PARAM_DEFINE_FLOAT(LAMA_MIN_INT_F, 0.5f);

/**
 * Delay to enable interaction control after push
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group Lama
 */
PARAM_DEFINE_FLOAT(INT_EN_DELAY, 0.001f);




