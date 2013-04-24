#ifndef CB_CONFIG_H
#define CB_CONFIG_H

#define CONTROL_LOOP_DT 0.001   // In seconds
#define DEATH_RAY_DT    0.00125   // In seconds

#define M_PI 3.14159
#define WHEEL_DIA 0.05842   // In m (2.3 in)
#define BB_VEL 3.048   // In m/s (10 ft/s)

#define ROT_SIZE ((BB_VEL * CONTROL_LOOP_DT) / (WHEEL_DIA * M_PI))
#define ROT_SPEED (BB_VEL/(WHEEL_DIA*M_PI))   // In rev/s
#define ROT_PERIOD_ST (MS2ST(1000)/(ROT_SPEED*256))   // 256 quadrature pulses per revolution.

#define DEATH_RAY_KP (-0.003)
#define DEATH_RAY_KI (-0.003)
#define DEATH_RAY_KD (-0.000)

#define DEATH_RAY_I_CAP 15   // I term cap. The right value here can minimize positive overshoot.
#define DEATH_RAY_STARTUP_COUNTER_MAX 200
#define DEATH_RAY_STARTUP_DC 0.058

#define ESC_MIN_DC 0.053   // Minimum duty cycle for ESCs.
#define ESC_MAX_DC 0.335


#endif /* CB_CONFIG_H */

