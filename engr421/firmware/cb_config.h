#ifndef CB_CONFIG_H
#define CB_CONFIG_H

#define CONTROL_LOOP_DT 0.001   // In seconds

#define M_PI 3.14159
#define WHEEL_DIA 0.05842   // In m (2.3 in)
#define BB_VEL 3.048   // In m/s (10 ft/s)

#define ROT_SIZE ((BB_VEL * CONTROL_LOOP_DT) / (WHEEL_DIA * M_PI))
#define ROT_SPEED (BB_VEL/(WHEEL_DIA*M_PI))   // In rev/s
#define ROT_PERIOD_ST (MS2ST(1000)/(ROT_SPEED*256))   // 256 quadrature pulses per revolution.


#endif /* CB_CONFIG_H */

