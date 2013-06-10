#ifndef CB_CONFIG_H
#define CB_CONFIG_H

/* Timesteps in seconds */
#define CONTROL_LOOP_DT 0.001
#define DEATH_RAY_DT    0.00125
#define HOPPER_DT       0.002
#define LINEAR_RAIL_DT  0.002

/* Controller parameters */
#define DEATH_RAY_KP (-0.018)
#define DEATH_RAY_KI (-0.006)
#define DEATH_RAY_KD (-0.000)

#define DEATH_RAY_I_CAP 15   // I term cap. The right value here can minimize positive overshoot.
#define DEATH_RAY_STARTUP_COUNTER_MAX 200
#define DEATH_RAY_STARTUP_DC 0.60

#define HOPPER_DC_MIN 0.0
#define HOPPER_DC_MAX 0.5
#define HOPPER_PULSE_DC 0.5
#define HOPPER_PULSE_FREQUENCY 2

#define LINEAR_RAIL_POS_KP (5)
#define LINEAR_RAIL_POS_KI (0)
#define LINEAR_RAIL_POS_KD (0)
#define LINEAR_RAIL_VEL_KP (8)
#define LINEAR_RAIL_VEL_KI (0)   // 1 is a good value here, but it increases power consumption without a target deadzone, so 0 for now.
#define LINEAR_RAIL_VEL_KD (0)

#define LINEAR_RAIL_MIN_SEPARATION 0.112
#define LINEAR_RAIL_POS_I_CAP 2
#define LINEAR_RAIL_VEL_I_CAP 2
#define LINEAR_RAIL_VEL_CAP 1.0   // In m/s.
#define LINEAR_RAIL_DC_MIN 0.0   // Minimum duty cycle for linear rail motor controllers.
#define LINEAR_RAIL_DC_MAX 1.0   // Maximum duty cycle for linear rail motor controllers.

/* Communication */
#define COMM_HEADER 255
#define COMM_PACKET_LENGTH 4   // Length of command packet excluding header.
#define COMM_RECEIVE_BUFFER_LENGTH 50   // Size of circular buffer used to store received bytes.
#define COMM_INPUT_MAX 250   // Highest valid command byte value (lowest is 0).

/* Status values */
#define DISABLED           0
#define STANDBY            1
#define BEAT_DANIEL_MILLER 2

/* Indices */
#define I_ADC_0 0
#define I_ADC_1 1
#define I_ADC_2 2
#define I_ADC_3 3
#define I_ICU_DEATH_RAY_0   2
#define I_ICU_DEATH_RAY_1   3
#define I_ICU_LINEAR_RAIL_0 4
#define I_ICU_LINEAR_RAIL_1 5
#define I_DIGITAL_LINEAR_RAIL_0 0
#define I_DIGITAL_LINEAR_RAIL_1 1
#define I_DIGITAL_2 2
#define I_DIGITAL_3 3
#define I_DIGITAL_4 4
#define I_DIGITAL_5 5
#define I_DIGITAL_6 6
#define I_DIGITAL_7 7
#define I_PWM_DEATH_RAY_0 0
#define I_PWM_DEATH_RAY_1 6   // This would be 1, but there's some sort of coupling going on between 0 and 1.
#define I_PWM_LINEAR_RAIL_0 2
#define I_PWM_LINEAR_RAIL_1 7
#define I_PWM_HOPPER_0 4
#define I_PWM_HOPPER_1 5

/* Constants */
#define M_PI 3.14159
#define WHEEL_DIA 0.05842   // In m (2.3 in)
#define BB_VEL_0 3.948   // In m/s (10 ft/s, plus a little extra because reality. 5.248 is competition speed!)
#define BB_VEL_1 4.548   // In m/s (10 ft/s, plus a little extra because reality. 5.248 is competition speed!)

#define ROT_SIZE_0 ((BB_VEL_0 * CONTROL_LOOP_DT) / (WHEEL_DIA * M_PI))
#define ROT_SPEED_0 (BB_VEL_0/(WHEEL_DIA*M_PI))   // In rev/s
#define ROT_PERIOD_ST_0 (MS2ST(1000)/(ROT_SPEED_0*256))   // 256 quadrature pulses per revolution.

#define ROT_SIZE_1 ((BB_VEL_1 * CONTROL_LOOP_DT) / (WHEEL_DIA * M_PI))
#define ROT_SPEED_1 (BB_VEL_1/(WHEEL_DIA*M_PI))   // In rev/s
#define ROT_PERIOD_ST_1 (MS2ST(1000)/(ROT_SPEED_1*256))   // 256 quadrature pulses per revolution.

#define REVS_PER_LENGTH 8.3   // Number of revolutions encoder pulley spins as it moves from one end of the slide to the other.

#define ESC_MIN_DC 0.45   // Minimum duty cycle for Mikrokopter ESCs.
#define ESC_MAX_DC 0.83    // Maximum duty cycle for Mikrokopter ESCs.

#endif /* CB_CONFIG_H */

