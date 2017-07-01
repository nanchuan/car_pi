#ifndef __TYPADP_H_
#define __TYPADP_H_

#include <stdio.h>
#include <unistd.h>

#include <wiringPi.h> 

#ifndef uint8_t
#define uint8_t unsigned char
#endif

#ifndef u16
#define u16 unsigned short
#endif

#ifndef int16_t
#define int16_t short
#endif

#ifndef u8
#define u8 unsigned char
#endif

#ifndef int32_t
#define int32_t int
#endif

#ifndef ABS
#define ABS(a) ((a) < 0 ? -(a) : (a))
#endif

typedef struct
{
    short X;
    short Y;
    short Z;
}T_int16_xyz;

typedef struct
{
    float X;
    float Y;
    float Z;
}T_float_xyz;

#endif

