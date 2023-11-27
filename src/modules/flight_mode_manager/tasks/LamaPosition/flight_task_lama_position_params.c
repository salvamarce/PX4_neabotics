/**
 * Send position setpoint during approach.
 *
 * If enabled, during the approach phase both velocity and position setpoints
 * are sent. Otherwise, position is not controlled and only velocity setpoints
 * are used.
 *
 * @boolean
 * @group Lama
 */
PARAM_DEFINE_INT32(APPR_SEND_POS_SP, 0);

/**
 * Maximum approach velocity
 *
 * @unit m/s
 * @decimal 3
 * @group Lama
 */
PARAM_DEFINE_FLOAT(APPR_MAX_VEL, 0.6f);

/**
 * Force value for acceleration setpoint during the transition between approach and interaction
 *
 * @unit N
 * @min 1.0
 * @max 50.0
 * @decimal 2
 * @group Lama
 */
PARAM_DEFINE_FLOAT(APPR_PUSH_FORCE, 10.0f);

/**
 * Maximum distance that can be read by tof sensors.
 *
 * It is advisable to set this parameter a little lower than the real
 * maximum limit of the sensor. Any measurement that is higher than this
 * value is considered not valid.
 *
 * @unit m
 * @decimal 2
 * @group Lama
 */
PARAM_DEFINE_FLOAT(TOF_MAX_DIST, 1.8f);

/**
 * Offset between the tof sensors and the contact surface of the drone.
 *
 * This is the distance that the tof sensors measure when the drone is
 * in contact.
 *
 * @unit m
 * @decimal 3
 * @group Lama
 */
PARAM_DEFINE_FLOAT(TOF_D_OFFSET, 0.07f);
