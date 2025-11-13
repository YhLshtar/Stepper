#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>

#define     USE_IWDG                0
#define     IWDG_WINDOWS_TIME      (5000 + 500) //ms

#define     DEBUG_LOG               0
#if DEBUG_LOG
#define     __LOG__         printf
#else
#define     __LOG__         //
#endif

void  StrNum_split(char* arr, char* Chars, char* Number);
float bytefloat(unsigned char a);
float byte1float(unsigned char a, unsigned char b);
float byte2float(unsigned char a, unsigned char b, unsigned char c, unsigned char d);
long  byte1int(unsigned char a, unsigned char b);
long  byte2int(unsigned char a, unsigned char b, unsigned char c, unsigned char d);

#ifdef __cplusplus
}
#endif

#endif // COMMON_H
