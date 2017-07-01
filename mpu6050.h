#ifndef __MPU6050_H_
#define __MPU6050_H_

int MPU6050_Init(void);
void MPU6050_Dataanl(T_float_xyz *data_tempacc,T_float_xyz *data_tempgyr);
void MPU6050_CalOff_Gyr(void);
void MPU6050_Read(void);

#endif
