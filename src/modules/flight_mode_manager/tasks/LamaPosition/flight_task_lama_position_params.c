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
 * Minimum distance that can be read by tof sensors.
 *
 * Any measurement that is less than this value is considered not valid.
 *
 * @unit m
 * @decimal 3
 * @group Lama
 */
PARAM_DEFINE_FLOAT(TOF_MIN_DIST, 0.01f);

/**
 * Maximum distance that can be read by tof sensors.
 *
 * It is advisable to set this parameter a little lower than the real
 * maximum limit of the sensor. Any measurement that is higher than this
 * value is considered not valid.
 *
 * @unit m
 * @decimal 3
 * @group Lama
 */
PARAM_DEFINE_FLOAT(TOF_MAX_DIST, 1.8f);


