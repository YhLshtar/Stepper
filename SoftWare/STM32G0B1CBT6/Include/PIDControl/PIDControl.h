#ifndef __PIDCONTROL_H
#define __PIDCONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

struct PIDParameter
{
    int Kp;
    int Ki;
    int Kd;
    int ErrorTotal;
    int ErrorLast;
    int RealValue;
    int TargetValue;
    int ValueMax;
    int ValueMin;
    int ErrorMax;
    int ErrorMin;

    struct
    {
        float dKpMax;
        float dKpMin;
        float qKp;
        float dKiMax;
        float dKiMin;
        float qKi;
        float dKdMax;
        float dKdMin;
        float qKd;
    }FuzzyParameter;
};

struct PIDParameter_Incr
{
    int Kp;
    int Ki;
    int Kd;
    int Error;
    int ErrorLast_1;
    int ErrorLast_2;
    int ValueMax;
    int ValueMin;
    int ErrorMax;
    int ErrorMin;

    struct
    {
        float dKpMax;
        float dKpMin;
        float qKp;
        float dKiMax;
        float dKiMin;
        float qKi;
        float dKdMax;
        float dKdMin;
        float qKd;
    }FuzzyParameter;
};

int PIDCompute(struct PIDParameter* pid_para, bool isAuto);
int PIDCompute_Incr(struct PIDParameter_Incr* pid_para, bool isAuto);

#ifdef __cplusplus
}
#endif

#endif /* __PIDCONTROL_H */