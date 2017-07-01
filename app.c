
#include <string.h>
#include <termio.h>
#include <pthread.h>
#include <sys/time.h>
#include <stdlib.h>
#include <softPwm.h>
#include <pigpio.h>

#include "typadp.h"
#include "mpu6050.h"
#include "imu.h"

#define CHAR_GET_LEN 128

// Pin modes
#define    INPUT              0
#define    OUTPUT             1
#define    PWM_OUTPUT         2
#define    GPIO_CLOCK         3
#define    SOFT_PWM_OUTPUT    4
#define    SOFT_TONE_OUTPUT   5
#define    PWM_TONE_OUTPUT    6

#define    LOW         0
#define    HIGH        1

#define    INT_EDGE_SETUP       0
#define    INT_EDGE_FALLING     1
#define    INT_EDGE_RISING      2
#define    INT_EDGE_BOTH        3

typedef struct{
    char *pcCamName;
    int (*piFun)(char* str_in);
    char *pcCamFun;
} CAM_S;

int cam_help(char* str_in);

void cmd_gets(char acTmp[], unsigned char cTmpLen)
{
    int i = 0;
    char cTmp = 0;
    
    cTmpLen--;
    
    while (1)
    {
        cTmp = getchar();
        
        if (('\n' == cTmp) || ('\r' == cTmp))
        {
            putchar(cTmp);
            break;    
        }                
            
        if (('\b' == cTmp) || (0x7f == cTmp))
        {            
            if (i > 0)
            {
                i--;
                putchar('\b');
                putchar(' ');
                putchar('\b');
            }                
            acTmp[i] = 0;    
            
            continue;
        }

        if (0x1b == cTmp)
        {
            cTmp = getchar();
            if (0x5b == cTmp)
            {
                cTmp = getchar();
                if ((0x32 <= cTmp) && (0x36 >= cTmp))
                {
                    cTmp = getchar();
                }
                else if (0x41 == cTmp)
                {
                    if (0 == i)
                    {
                        while(acTmp[i]) 
                        {
                            putchar(acTmp[i]);
                            i++;
                        }
                    }
                }
            }
            continue;
        }
        
        if ((i < cTmpLen) && (cTmp >= 32))
        {
            acTmp[i] = cTmp;
            i++;
            putchar(cTmp);
        }
    }
    
    acTmp[i] = 0;
    
    return;
}

void cmd_get_cmd(char* str_in, char* str_out ,int* p)
{
    int i = 0, j = 0;
    
    for (i = 0; ((i < CHAR_GET_LEN) && (0 != str_in[i])); i++)
    {
        if (' ' != str_in[i])
            break;
    }
    for (; ((i < CHAR_GET_LEN) && (0 != str_in[i])); i++)
    {
        if (' ' == str_in[i])
        {
            break;
        }
        str_out[j++] = str_in[i];
    }
    str_out[j] = 0;
    if (p) *p = i;

    return;
}

int cmd_getn(char* str_in, int* p)
{
    #define NUM_CHAR_LEN 12
    int iNum = 0;
    int iOk = 0, iSigFlg = 0;
    char acTmp[NUM_CHAR_LEN] = {0};
    int i = 0;

    if (NULL == str_in)
    {
        printf(" ->");
        cmd_gets(acTmp, NUM_CHAR_LEN);
    }
    else
    {
       for (i = 0; i < NUM_CHAR_LEN; i++)
        {
            acTmp[i] = str_in[i];
            if (0 == acTmp[i])
                break;
        }
    }

    for (i = 0; i < NUM_CHAR_LEN; i++)
    {
        if (' ' == acTmp[i])
        {
            if (0 == iOk)
            {
                continue;
            }
        }

        if ('-' == acTmp[i])
        {
            if (0 == iOk)
            {
                iSigFlg = 1;
                iOk++;
                continue;
            }
        }

        if ('+' == acTmp[i])
        {
            if (0 == iOk)
            {
                iSigFlg = 0;
                iOk++;
                continue;
            }
        }

        if (('x' == acTmp[i]) || ('X' == acTmp[i]))
        {
            if (1 == iOk)
            {
                if ('0' == acTmp[i-1])
                {
                    iSigFlg = 2;
                    iOk++;
                    continue;
                }
            }
        }

        if (2 == iSigFlg)
        {
            if (('0' <= acTmp[i]) && ('9' >= acTmp[i]))
            {                
                iNum *= 16;
                iNum += acTmp[i] - '0';
                iOk++;
            }
            else if (('a' <= acTmp[i]) && ('f' >= acTmp[i]))
            {                
                iNum *= 16;
                iNum += acTmp[i] - 'a' + 10;
                iOk++;
            }
            else if (('A' <= acTmp[i]) && ('F' >= acTmp[i]))
            {                
                iNum *= 16;
                iNum += acTmp[i] - 'A' + 10;
                iOk++;
            }
            else
            {
                break;
            }
        }
        else
        {
            if (('0' <= acTmp[i]) && ('9' >= acTmp[i]))
            {                
                iNum *= 10;
                iNum += acTmp[i] - '0';
                iOk++;
            }
            else
            {
                break;
            }
        }
    }

    if (1 == iSigFlg)
    {
        iNum = 0 - iNum;
    }

    if (p) *p = i;

    return iNum;
}

float cmd_getf(char* str_in, int* p)
{
    #define NUM_CHAR_LEN 12
    float fNum = 0, fTmp = 0;
    int iOk = 0, iSigFlg = 0, iFN = 0;
    char acTmp[NUM_CHAR_LEN] = {0};
    int i = 0;

    if (NULL == str_in)
    {
        printf(" ->");
        cmd_gets(acTmp, NUM_CHAR_LEN);
    }
    else
    {
       for (i = 0; i < NUM_CHAR_LEN; i++)
        {
            acTmp[i] = str_in[i];
            if (0 == acTmp[i])
                break;
        }
    }

    for (i = 0; i < NUM_CHAR_LEN; i++)
    {
        if (' ' == acTmp[i])
        {
            if (0 == iOk)
            {
                continue;
            }
        }

        if ('-' == acTmp[i])
        {
            if (0 == iOk)
            {
                iSigFlg = 1;
                iOk++;
                continue;
            }
        }

        if ('+' == acTmp[i])
        {
            if (0 == iOk)
            {
                iSigFlg = 0;
                iOk++;
                continue;
            }
        }

        if ('.' == acTmp[i])
        {
            if (0 == iFN)
            {                
                iOk++;
                iFN++;
                continue;
            }
        }

        if (('0' <= acTmp[i]) && ('9' >= acTmp[i]))
        {    
            if (0 == iFN)
            {
                fNum *= 10;
                fNum += acTmp[i] - '0';
                iOk++;
            }
            else
            {
                iFN *= 10;
                fTmp = acTmp[i] - '0';
                fTmp /= iFN;
                fNum += fTmp;
                iOk++;
            }
        }
        else
        {
            break;
        }
    }

    if (1 == iSigFlg)
    {
        fNum = 0 - fNum;
    }
    
    if (p) *p = i;

    return fNum;
}


int getnum_test(char* str_in)
{
    printf("   %d", cmd_getn(0, 0));
    
    return 0;
}

int getfloat_test(char* str_in)
{
    printf("   %f", cmd_getf(0, 0));
    
    return 0;
}

int getstr_test(char* str_in)
{
    char acTmp[CHAR_GET_LEN] = {0};
    
    printf(" ->");
    cmd_gets(acTmp, CHAR_GET_LEN);
    printf("   %s", acTmp);
    
    return 0;
}

struct termios stored_settings;
void termios_init(void)
{
    struct termios new_settings;
    
    tcgetattr(0,&stored_settings);
    memcpy(&new_settings, &stored_settings, sizeof(new_settings));
    new_settings.c_lflag &= ~(ECHO + ICANON);
    new_settings.c_cc[VTIME] = 0;
    new_settings.c_cc[VMIN] = 1;
    //new_settings.c_iflag = ICRNL | IXON;
     //new_settings.c_oflag = OPOST | ONLCR | NL0 | CR0 | TAB0 | BS0 | VT0 | FF0;
     //new_settings.c_cflag = CS8 | CREAD;
     //new_settings.c_lflag = ISIG | ICANON | IEXTEN | ECHO | ECHOE | ECHOK | ECHOKE;
     //new_settings.c_cc = INIT_C_CC;
    tcsetattr(0,TCSANOW,&new_settings);
}

void termios_deinit(void)
{
    tcsetattr(0,TCSANOW,&stored_settings);
}

int getch_test(char* str_in)
{
    char cTmp = 0;    
        
    printf(" ->");
    while(1)
    {            
        cTmp = getchar();
        printf(" 0x%x", cTmp);

        if ('q' == cTmp)
        {
            break;
        }
    }

    return 0;
}

int mpu_test(char* str_in)
{
    int i = 0;
    T_float_xyz data_tempacc;
    T_float_xyz data_tempgyr;

    if (NULL == str_in)
    {
        return -1;
    }

    i = cmd_getn(str_in, 0);
    while(i-- > 0)
    {
        MPU6050_Dataanl(&data_tempacc, &data_tempgyr);

        printf(" %-10f %-10f %-10f %-10f %-10f %-10f\r\n",
            data_tempacc.X, data_tempacc.Y, data_tempacc.Z,
            data_tempgyr.X, data_tempgyr.Y, data_tempgyr.Z);
        delay(50);
    }

    return 0;
}

#define PWM_RANGE 100
#define PWM_DATH  0
#define PWM_NEWRANGE  (PWM_RANGE - PWM_DATH)
int encoder_count_l = 0, encoder_count_r = 0;
float g_speed_l = 0.0, g_speed_r = 0.0;
void encoder_interrupt_l(void) 
{
    if (digitalRead(4))
        encoder_count_l++;
    else
        encoder_count_l--;
}
void encoder_interrupt_r(void) 
{
    if (digitalRead(21))
        encoder_count_r--;
    else
        encoder_count_r++;        
}

void encoder_alert_l(int gpio, int level, uint32_t tick)
{
    if (0 == level)
    {
        if (gpioRead(23))
            encoder_count_l++;
        else
            encoder_count_l--;
    }
}
void encoder_alert_r(int gpio, int level, uint32_t tick)
{
    if (0 == level)
    {
        if (gpioRead(5))
            encoder_count_r--;
        else
            encoder_count_r++;  
    }
}

void car_speed_init(void)
{
    (void)softPwmCreate (0, 0, PWM_RANGE);
    (void)softPwmCreate (1, 0, PWM_RANGE);
    pinMode(2, OUTPUT);
    digitalWrite(2, LOW);
    pinMode(3, OUTPUT);
    digitalWrite(3, LOW);

    pinMode(4, INPUT);
    pinMode(5, INPUT);
    pinMode(21, INPUT);
    pinMode(22, INPUT);

    //wiringPiISR (5, INT_EDGE_FALLING, &encoder_interrupt_l);
    //wiringPiISR (22, INT_EDGE_FALLING, &encoder_interrupt_r);

    gpioSetMode(23, PI_INPUT);
    gpioSetMode(24, PI_INPUT);
    gpioSetMode(5, PI_INPUT);
    gpioSetMode(6, PI_INPUT);

    gpioSetAlertFunc(24, encoder_alert_l);
    gpioSetAlertFunc(6, encoder_alert_r);
}

void car_speed_set(int speed_l, int speed_r)
{
    if (ABS(speed_l) <= PWM_NEWRANGE)
    {
        if (speed_l < 0)
        {
            digitalWrite(2, HIGH);
            softPwmWrite (0, speed_l + PWM_NEWRANGE);
        }
        else if (speed_l > 0)
        {
            digitalWrite(2, LOW);
            softPwmWrite (0, speed_l + PWM_DATH);
        }
        else
        {
            digitalWrite(2, LOW);
            softPwmWrite (0, 0);
        }
    }

    if (ABS(speed_r) <= PWM_NEWRANGE)
    {
        if (speed_r < 0)
        {
            digitalWrite(3, HIGH);
            softPwmWrite (1, speed_r + PWM_NEWRANGE);
        }
        else if (speed_r > 0)
        {
            digitalWrite(3, LOW);
            softPwmWrite (1, speed_r + PWM_DATH);
        }
        else
        {
            digitalWrite(3, LOW);
            softPwmWrite (1, 0);
        }
    }
}

int speed_show(char* str_in)
{
    int i = 0;
    char* str = str_in;
    int str_p = 0;
    int speed_l = 0, speed_r = 0;

    speed_l = cmd_getn(str, &str_p);
    str += str_p;
    speed_r = cmd_getn(str, &str_p);
    str += str_p;
    i = cmd_getn(str, 0);

    car_speed_set(speed_l, speed_r);        
    while(i-- > 0)
    {
        printf(" %-10f %-10f \r\n", g_speed_l, g_speed_r);
        delay(250);
    }
    car_speed_set(0, 0);
    
    return 0;

}

int pwm_test(char* str_in)
{    
    int speed_l = 0, speed_r = 0;
    int str_p = 0;

    speed_l = cmd_getn(str_in, &str_p);
    speed_r = cmd_getn(str_in + str_p, 0);

    car_speed_set(speed_l, speed_r);

    return 0;
}

int car_test(char* str_in)
{
    int speed_l = 0, speed_r = 0;
    int cTmp = 0;
    /*struct termios old_settings;
    struct termios new_settings;
    
    tcgetattr(0,&old_settings);
    memcpy(&new_settings, &old_settings, sizeof(new_settings));
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHOCTL;
    new_settings.c_cc[VTIME] = 0;
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);*/

    //car_speed_init();
    
    printf(" ->");
    while(1)
    {
        cTmp = getchar();
        if (0x1b == cTmp)
        {
            cTmp <<= 8;
            cTmp |= getchar();
            if (0x1b5b == cTmp)
            {
                cTmp <<= 8;
                cTmp |= getchar();
            }
        }

        switch(cTmp)
        {
        case '2':
        case 0x1b5b41:
            speed_l += 10;
            speed_r += 10;
            printf("u");
            break;
        case '8':
        case 0x1b5b42:
            speed_l -= 10;
            speed_r -= 10;
            printf("b");
            break;
        case '4':
        case 0x1b5b44:
            speed_l -= 10;
            speed_r += 10;
            car_speed_set(speed_l, speed_r);
            printf("l");
            delay(200);
            speed_l += 10;
            speed_r -= 10;
            break;
        case '6':
        case 0x1b5b43:
            speed_l += 10;
            speed_r -= 10;
            car_speed_set(speed_l, speed_r);
            printf("r");
            delay(200);
            speed_l -= 10;
            speed_r += 10;
            break;    
        case '5':
        case ' ':
            speed_l = 0;
            speed_r = 0;
            printf("p");
            break;    
        case 'q':
            speed_l = 0;
            speed_r = 0;
            car_speed_set(speed_l, speed_r);    
            //softPwmStop(0);
            //softPwmStop(1);
            /*tcsetattr(0,TCSANOW,&old_settings);*/
            return 0;
        }
        car_speed_set(speed_l, speed_r);
    }
}

#define SPEED_PID_ACC_MAX 100.0
int speed_pid_en = 0;
float speed_pid_in = 0.0;
float speed_pid_acc = 0.0;
float speed_pid_err_pre = 0.0;
float speed_pid_p = 0.0;
float speed_pid_i = 0.0;
float speed_pid_d = 0.0;
T_float_angle imu_angle;
pthread_t imuthread;
static PI_THREAD (ImuThread)
{    
    T_float_xyz data_acc;
    T_float_xyz data_gyr;
    struct timeval tOld, tNow;
    float interval;
    //int speed_pin_stat[6] = {0};
    int iTim_Count = 0;
    float fPid_Interval = 0;

    float speed_pid_err = 0.0;
    float speed_pid_out = 0.0;

    gettimeofday(&tNow, NULL);
    tOld.tv_sec = tNow.tv_sec;
    tOld.tv_usec = tNow.tv_usec;

    piHiPri(60);
    for (;;)
    {
        gettimeofday(&tNow, NULL);
        if (tNow.tv_sec > tOld.tv_sec)
        {
            interval =  1000000 - tOld.tv_usec + tNow.tv_usec;
        }
        else
        {
            interval = tNow.tv_usec - tOld.tv_usec;
        }

        //if (interval > 2000)
        {
            interval /= 1000000.0f;
            tOld.tv_sec = tNow.tv_sec;
            tOld.tv_usec = tNow.tv_usec;

            MPU6050_Read();
            MPU6050_Dataanl(&data_acc,&data_gyr);
            Prepare_Data(&data_acc,&data_acc);
            IMUupdate(&data_gyr,&data_acc,interval,&imu_angle);
            
            iTim_Count++;
            fPid_Interval += interval;
            if (0 == (iTim_Count % 30))
            {
                g_speed_l = 0.0045 * encoder_count_l / fPid_Interval;
                encoder_count_l = 0;
                g_speed_r = 0.0045 * encoder_count_r / fPid_Interval;
                encoder_count_r = 0;                
                
                if (speed_pid_en)
                {
                    speed_pid_err = speed_pid_in - ((g_speed_l + g_speed_r) / 2);
                    if (ABS(speed_pid_acc + speed_pid_err) < SPEED_PID_ACC_MAX)
                        {speed_pid_acc += speed_pid_err;}
                    speed_pid_out = speed_pid_p * speed_pid_err 
                                  + speed_pid_i * speed_pid_acc * fPid_Interval
                                  + speed_pid_d * (speed_pid_err - speed_pid_err_pre) / fPid_Interval;                                    
                    speed_pid_err_pre = speed_pid_err;

                    car_speed_set((int)speed_pid_out, (int)speed_pid_out);
                }
                
                fPid_Interval = 0;
            }
        }

        #if 0
        speed_pin_stat[1] = digitalRead(5);
        speed_pin_stat[4] = digitalRead(4);
        speed_pin_stat[3] = digitalRead(22);
        speed_pin_stat[5] = digitalRead(21);
        if (!speed_pin_stat[1])
        {
            if (speed_pin_stat[0])
            {
                if (speed_pin_stat[4]) {encoder_count_l++;}
                else {encoder_count_l--;}
            }
        }
        speed_pin_stat[0] = speed_pin_stat[1];        
        if (!speed_pin_stat[3])
        {
            if (speed_pin_stat[2])
            {
                if (speed_pin_stat[5]) {encoder_count_r--;}
                else {encoder_count_r++;}
            }
        }
        speed_pin_stat[2] = speed_pin_stat[3];        
        #endif

        delayMicroseconds(2000);
    }

  return NULL;
}

int imu_test(char* str_in)
{
    int i = 0;

    i = cmd_getn(str_in, 0);

    while(i-- > 0)
    {
        printf(" %-10f %-10f %-10f\r\n", imu_angle.rol, imu_angle.pit, imu_angle.yaw);
        delay(50);
    }
    
    return 0;
}

int imu_gyro_calib(char* str_in)
{
    MPU6050_CalOff_Gyr();
    return 0;
}

int pid_test(char* str_in)
{
    float in,p,i,d;
    char* str = str_in;
    int str_p = 0;    
    int e = 0;

    e = cmd_getn(str, &str_p);
    if (e) 
    {
        str += str_p;
        in = cmd_getf(str, &str_p);
        str += str_p;
        p = cmd_getf(str, &str_p);
        str += str_p;
        i = cmd_getf(str, &str_p);
        str += str_p;
        d = cmd_getf(str, 0);
        speed_pid_en = 1;
        speed_pid_in = in;
        speed_pid_p = p;
        speed_pid_i = i;
        speed_pid_d = d;
        printf("%f %f %f %f\n", in, p, i, d);

        while(e-- > 0)
        {
            printf(" %-10f %-10f \r\n", g_speed_l, g_speed_r);
            delay(250);
        }
    }
    else
    {
        speed_pid_en = 0;
        speed_pid_p = speed_pid_i = speed_pid_d = 0;
        speed_pid_err_pre = 0;
        speed_pid_acc = 0;
        car_speed_set(0, 0);
    }

    return 0;
}

int while_quit(char* str_in)
{
    printf("\r\n");
    
    pthread_cancel(imuthread);
    pthread_join(imuthread, NULL);
    
    softPwmStop(0);
    softPwmStop(1);
    
    termios_deinit();    

    exit(0);  
    
    return 0;
}

CAM_S g_astCamGroup[] = 
{
    {
        "?", cam_help, "cam help"
    },
    {
        "quit", while_quit, "quit"
    },
    {
        "getch-test", getch_test, "get char test"
    },
    {
        "getnum-test", getnum_test, "get num test"
    },
    {
        "getstr-test", getstr_test, "get str test"
    },
    {
        "getf-test", getfloat_test, "get float test"
    },
    {
        "mpu-test", mpu_test, "mpu test [num]"
    },
    {
        "pwm-test", pwm_test, "pwm test [speed-l] [speed-r]"
    },
    {
        "car-test", car_test, "car test"
    },
    {
        "imu-test", imu_test, "imu test [num]"
    },
    {
        "imu-gyro-c", imu_gyro_calib, "imu gyro calibration"
    },
    {
        "pid-test", pid_test, "pid test [en] [p] [i] [d]"
    },
    {
        "speed-show", speed_show, "speed show [speed-l] [speed-r] [num]"
    },
};

int cam_help(char* str_in)
{
    int i = (sizeof(g_astCamGroup) / sizeof(CAM_S));

    printf(" NAME         FUN");
    while(i)
    {
        i--;
        printf("\n\r %-12s %s", g_astCamGroup[i].pcCamName, g_astCamGroup[i].pcCamFun);
    }

    return 0;
}

int main(void)
{
    char str_get[CHAR_GET_LEN] = {0};
    char cmd_str[CHAR_GET_LEN] = {0};
    char* p_str_param = NULL;
    int str_offset = 0;
    int i = 0;
    int iCamFunReturn = 0;
    int iCamRunFlag = 0;
    int res;    

    termios_init();
    wiringPiSetup();
    gpioInitialise();
    car_speed_init();
    (void)MPU6050_Init();
    
    res = pthread_create(&imuthread, NULL, ImuThread, NULL);
    if (0 != res)
    {
        printf(" ERR: pthread_create(&imuThread, NULL, ImuThread, NULL) return %d", res);
        return res;
    }
    
    while (1)
    {
        printf("->");
        cmd_gets(str_get, CHAR_GET_LEN);
        cmd_get_cmd(str_get, cmd_str, &str_offset);
        p_str_param = &str_get[str_offset];

        iCamRunFlag = 0;
        i = (sizeof(g_astCamGroup) / sizeof(CAM_S));
        while (i)
        {
            i--;
            
            if (0 == strcmp(g_astCamGroup[i].pcCamName, cmd_str))
            {
                iCamRunFlag = 1;
                iCamFunReturn = g_astCamGroup[i].piFun(p_str_param);
                if (0 != iCamFunReturn)
                {
                    printf("\r\nCAM RUN ERR : %d", iCamFunReturn);
                }
                else
                {
                    printf("\r\nCAM RUN OK.");
                }
                break;
            }
        }        

        if (strlen(cmd_str) > 0)
        {  
            if (0 == iCamRunFlag)
            {
                printf("NO CAM : %s", cmd_str);
            }
            
            printf("\r\n");
        }        
    }
}



