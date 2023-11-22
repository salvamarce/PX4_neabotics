
/**
 * Max approach velocity on x axis
 *
 * @unit m/s
 * @decimal 3
 * @group Lama
 */
PARAM_DEFINE_FLOAT(APPR_MAX_VEL_X, 0.3f);

/**
 * Max approach velocity on z axis
 *
 * @unit m/s
 * @decimal 3
 * @group Lama
 */
PARAM_DEFINE_FLOAT(APPR_MAX_VEL_Z, 0.2f);

/**
 * Epsilon value for approach phase.
 * 
 * The position setpoint for the approach phase is considered
 * to be behind the wall, specifically APPR_EPS meters behind the surface.
 * Tuning this parameter helps to reach a satisfactory contact at the end
 * of approach phase.
 *
 * @unit m
 * @decimal 3
 * @group Lama
 */
PARAM_DEFINE_FLOAT(APPR_EPS, 0.3f);

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