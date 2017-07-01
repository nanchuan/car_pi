#ifndef _IMU_H_
#define _IMU_H_

typedef struct
{
    float rol;
    float pit;
    float yaw;
}T_float_angle;

void Prepare_Data(T_float_xyz *acc_in,T_float_xyz *acc_out);

void IMUupdate(T_float_xyz *gyr, T_float_xyz *acc, float interval, T_float_angle *angle);

#endif
