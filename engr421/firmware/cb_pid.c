#include <cb_pid.h>

float calculate_pid (float cur, float des, pid_data_t* pid_data)
{
    pid_data->P = des - cur;
    pid_data->I += pid_data->P * pid_data->dt;

    // Derivative term from difference between last two measured values divided
    // by time interval.
    pid_data->D = (cur - pid_data->last_val) / pid_data->dt;
    pid_data->last_val = cur;

    return pid_data->Kp * pid_data->P +
           pid_data->Ki * pid_data->I +
           pid_data->Kd * pid_data->D;
}

