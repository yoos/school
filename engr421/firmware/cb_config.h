#ifndef CB_CONFIG_H
#define CB_CONFIG_H

/* Timesteps in seconds */
#define CONTROL_LOOP_DT 0.001
#define DEATH_RAY_DT    0.00125
#define HOPPER_DT       0.002
#define LINEAR_RAIL_DT  0.002

/* Controller parameters */
#define DEATH_RAY_KP (-0.005)
#define DEATH_RAY_KI (-0.005)
#define DEATH_RAY_KD (-0.000)

#define DEATH_RAY_I_CAP 15   // I term cap. The right value here can minimize positive overshoot.
#define DEATH_RAY_STARTUP_COUNTER_MAX 200
#define DEATH_RAY_STARTUP_DC 0.60

#define HOPPER_DC_MIN 0.0
#define HOPPER_DC_MAX 1.0

#define LINEAR_RAIL_POS_KP (1)
#define LINEAR_RAIL_POS_KI (0)
#define LINEAR_RAIL_POS_KD (0)
#define LINEAR_RAIL_VEL_KP (4)
#define LINEAR_RAIL_VEL_KI (0)
#define LINEAR_RAIL_VEL_KD (0)

#define LINEAR_RAIL_POS_I_CAP 1
#define LINEAR_RAIL_VEL_I_CAP 1
#define LINEAR_RAIL_VEL_CAP 0.5   // In m/s.
#define LINEAR_RAIL_DC_MIN 0.0   // Minimum duty cycle for linear rail motor controllers.
#define LINEAR_RAIL_DC_MAX 1.0   // Maximum duty cycle for linear rail motor controllers.

/* Communication */
#define COMM_HEADER 255
#define COMM_PACKET_LENGTH 4   // Length of command packet excluding header.
#define COMM_RECEIVE_BUFFER_LENGTH 50   // Size of circular buffer used to store received bytes.
#define COMM_INPUT_MAX 250   // Highest valid command byte value (lowest is 0).

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
#define I_PWM_DEATH_RAY_1 1
#define I_PWM_LINEAR_RAIL_0 2
#define I_PWM_LINEAR_RAIL_1 3
#define I_PWM_HOPPER_0 4
#define I_PWM_HOPPER_1 5

/* Constants */
#define M_PI 3.14159
#define WHEEL_DIA 0.05842   // In m (2.3 in)
#define BB_VEL 3.048   // In m/s (10 ft/s)

#define ROT_SIZE ((BB_VEL * CONTROL_LOOP_DT) / (WHEEL_DIA * M_PI))
#define ROT_SPEED (BB_VEL/(WHEEL_DIA*M_PI))   // In rev/s
#define ROT_PERIOD_ST (MS2ST(1000)/(ROT_SPEED*256))   // 256 quadrature pulses per revolution.

#define REVS_PER_LENGTH 8.3   // Number of revolutions encoder pulley spins as it moves from one end of the slide to the other.

#define ESC_MIN_DC 0.472   // Minimum duty cycle for Mikrokopter ESCs.
#define ESC_MAX_DC 0.83    // Maximum duty cycle for Mikrokopter ESCs.

#endif /* CB_CONFIG_H */

