
#include "typadp.h"
#include "imu.h"
#include "math.h"

#define RtA         57.2957795f        //���ȵ��Ƕ�        
#define AtR            0.0174533f        //�ȵ��Ƕ�        
#define Acc_G         0.0011963f        //���ٶȱ��G        
#define Gyro_G         114.58763f        //���ٶȱ�ɶ�   �˲�����Ӧ����2000��ÿ��        
#define FILTER_NUM     16

#define Kp 1.6f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
//#define Ki 0.001f                      // integral gain governs rate of convergence of gyroscope biases
//#define halfT 0.001f                   // half the sample period �������ڵ�һ��
#define TH 0.002

float AngleOffset_Rol=0, AngleOffset_Pit=0;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;  // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0; // scaled integral error

void Prepare_Data(T_float_xyz *acc_in,T_float_xyz *acc_out)
{
    static int filter_cnt=0;
    static float ACC_X_BUF[FILTER_NUM], ACC_Y_BUF[FILTER_NUM], ACC_Z_BUF[FILTER_NUM];
    float temp1=0, temp2=0, temp3=0;
    int i;

    ACC_X_BUF[filter_cnt] = acc_in->X;
    ACC_Y_BUF[filter_cnt] = acc_in->Y;
    ACC_Z_BUF[filter_cnt] = acc_in->Z;
    
    for(i=0;i<FILTER_NUM;i++)
    {
        temp1 += ACC_X_BUF[i];
        temp2 += ACC_Y_BUF[i];
        temp3 += ACC_Z_BUF[i];
    }
    
    acc_out->X = temp1 / FILTER_NUM;
    acc_out->Y = temp2 / FILTER_NUM;
    acc_out->Z = temp3 / FILTER_NUM;
    
    filter_cnt++;    
    if(filter_cnt==FILTER_NUM)    filter_cnt=0;
}

void IMUupdate(T_float_xyz *gyr, T_float_xyz *acc, float interval, T_float_angle *angle)
{
    float ax = acc->X, ay = acc->Y, az = acc->Z;
    float gx = gyr->X, gy = gyr->Y, gz = gyr->Z;
    float norm;
    
    float vx, vy, vz;
    float ex, ey, ez;

    // �Ȱ���Щ�õõ���ֵ���
    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
    //  float q0q3 = q0*q3;
    float q1q1 = q1*q1;
    //  float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;

    if(ax*ay*az==0)
        return;

    /*
    if (ABS(gx) < TH)
    {
        gx = 0;
    }
    if (ABS(gy) < TH)
    {
        gy = 0;
    }
    if (ABS(gz) < TH)
    {
        gz = 0;
    }*/    

    interval /= 2.0;
    
    // estimated direction of gravity and flux (v and w)      �����������������/��Ǩ
    // ���ǰ���Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء�
    // �������Ҿ����ŷ���ǵĶ��壬��������ϵ������������ת����������ϵ��������������Ԫ�ء�
    // ���������vx\y\z����ʵ���ǵ�ǰ��ŷ���ǣ�����Ԫ�����Ļ����������ϵ�ϣ����������������λ������      
    vx = 2*(q1q3 - q0q2);                                                
    vy = 2*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay*vz - az*vy);         //�������������õ���־������                                       
    ey = (az*vx - ax*vz);
    ez = (ax*vy - ay*vx);

    exInt = exInt + ex * interval;    //�������л���
    eyInt = eyInt + ey * interval;
    ezInt = ezInt + ez * interval;

    // adjusted gyroscope measurements
    gx = gx + Kp*ex + exInt;    //�����PI�󲹳��������ǣ�����������Ư��
    gy = gy + Kp*ey + eyInt;    
    gz = gz + Kp*ez + ezInt;    //�����gz����û�й۲��߽��н��������Ư�ƣ����ֳ����ľ��ǻ����������Լ�

    // integrate quaternion rate and normalise                          
    //��Ԫ�ص�΢�ַ��̣�һ�ױϿ���ⷨ��������Ԫ��
    q0 = q0 + (-q1*gx - q2*gy - q3*gz) * interval;
    q1 = q1 + (q0*gx + q2*gz - q3*gy) * interval;
    q2 = q2 + (q0*gy - q1*gz + q3*gx) * interval;
    q3 = q3 + (q0*gz + q1*gy - q2*gx) * interval;

    // normalise quaternion
    norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;

    if (ABS(gyr->Z) >= TH)
    {
        angle->yaw += gyr->Z*Gyro_G*interval;
    }    
    //angle->yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)*57.3;
    angle->rol = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3 - AngleOffset_Pit; // pitch
    angle->pit = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1) * 57.3 - AngleOffset_Rol; // roll
}

