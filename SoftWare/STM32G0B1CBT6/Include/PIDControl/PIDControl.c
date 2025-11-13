#include "PIDControl.h"

#define     NB          -3
#define     NM          -2
#define     NS          -1
#define     ZO           0
#define     PS           1
#define     PM           2
#define     PB           3
#define     Num          7

signed char KpMatrix[7][7]={{PB,PB,PM,PM,PS,ZO,ZO},//Kp规则表
                            {PB,PB,PM,PS,PS,ZO,NS},
                            {PM,PM,PM,PS,ZO,NS,NS},
                            {PM,PM,PS,ZO,NS,NM,NM},
                            {PS,PS,ZO,NS,NS,NM,NM},
                            {PS,ZO,NS,NM,NM,NM,NB},
                            {ZO,ZO,NM,NM,NM,NB,NB}};
signed char KiMatrix[7][7]={{NB,NB,NM,NM,NS,ZO,ZO},//Ki规则表
                            {NB,NB,NM,NS,NS,ZO,ZO},
                            {NB,NM,NS,NS,ZO,PS,PS},
                            {NM,NM,NS,ZO,PS,PM,PM},
                            {NM,NS,ZO,PS,PS,PM,PB},
                            {ZO,ZO,PS,PS,PM,PB,PB},
                            {ZO,ZO,PS,PM,PM,PB,PB}};
signed char KdMatrix[7][7]={{PS,NS,NB,NB,NB,NM,PS},//Kd规则表
                            {PS,NS,NB,NM,NM,NS,ZO},
                            {ZO,NS,NM,NM,NS,NS,ZO},
                            {ZO,NS,NS,NS,NS,NS,ZO},
                            {ZO,ZO,ZO,ZO,ZO,ZO,ZO},
                            {PB,NS,PS,PS,PS,PS,PB},
                            {PB,PM,PM,PM,PS,PS,PB}};
struct DegreeInformation
{
    float q_value;           //量化值
    float degree_value1;
    float degree_value2;
    int   degree_index1;
    int   degree_index2;
};
struct DegreeInformation E_value  = {0};
struct DegreeInformation dE_value = {0};
/*隶属度函数，自己定义。但是函数和定义的NB--PB的值息息相关*/
float Fun_e(float value, float* index)
{
    if((float)NB <= value && value <= (float)NM)
    {
        *index = 0;
        return -value - 2;
    }
    else if((float)NM < value && value <= (float)NS)
    {
        *index = 1;
        return  value + 2;
    }
    else if((float)NS < value && value <= (float)ZO)
    {
        *index = 2;
        return -value;
    }
    else if((float)ZO < value && value <= (float)PS)
    {
        *index = 3;
        return  value;
    }
    else if((float)PS < value && value <= (float)PM)
    {
        *index = 4;
        return -value + 2;
    }
    else if((float)PM < value && value <= (float)PB)
    {
        *index = 5;
        return  value - 2;
    }

    if( value < (float)NB)
    {
        *index = 0;
    }
    else if((float)PB < value)
    {
        *index = 6;
    }

    return 1;
}

float Fun_de(float value, float* index)
{
    if((float)NB <= value && value <= (float)NM)
    {
        *index = 1;
        return  value + 3;
    }
    else if((float)NM < value && value <= (float)NS)
    {
        *index = 2;
        return -value - 1;
    }
    else if((float)NS < value && value <= (float)ZO)
    {
        *index = 3;
        return  value + 1;
    }
    else if((float)ZO < value && value <= (float)PS)
    {
        *index = 4;
        return -value + 1;
    }
    else if((float)PS < value && value <= (float)PM)
    {
        *index = 5;
        return  value - 1;
    }
    else if((float)PM < value && value <= (float)PB)
    {
        *index = 6;
        return -value + 3;
    }

    if( value < (float)NB)
    {
        *index = 0;
    }
    else if((float)PB < value)
    {
        *index = 6;
    }

    return 0;
}

float E_Map(struct PIDParameter* pid_para)
{
    float e      = pid_para->TargetValue - pid_para->RealValue;/*e*/
    float de     = e - pid_para->ErrorLast;                    /*de/dt*/

    float value  = (Num - 1) * e / (pid_para->ErrorMax - pid_para->ErrorMin);

    return (value - (Num - 1));
}

float dE_Map(struct PIDParameter* pid_para)
{
    float e     = pid_para->TargetValue - pid_para->RealValue;/*e*/
    float de    = e - pid_para->ErrorLast;                    /*de/dt*/

    float value = (Num - 1) * de / (pid_para->ErrorMax - pid_para->ErrorMin) * 2;

    return (value - (Num - 1));
}
/*反区间映射函数*/
float ErrorReMap(float ValueMax, float ValueMin, float Value)
{
    return (Value + (Num - 1)) * (ValueMax - ValueMin) / (Num - 1);
}
/*加权平均求解Kp输出量的量化值*/
float DeltaKp_average(struct DegreeInformation E_value, struct DegreeInformation dE_value)
{
    return (E_value.degree_value1 * dE_value.degree_value1) *
            KpMatrix[E_value.degree_index1][dE_value.degree_index1] +//1
           (E_value.degree_value1 * dE_value.degree_value2) *
            KpMatrix[E_value.degree_index1][dE_value.degree_index2] +//2
           (E_value.degree_value2 * dE_value.degree_value1) *
            KpMatrix[E_value.degree_index2][dE_value.degree_index1] +//3
           (E_value.degree_value2 * dE_value.degree_value2) *
            KpMatrix[E_value.degree_index2][dE_value.degree_index2]; //4
}
/*加权平均求解Ki输出量的量化值*/
float DeltaKi_average(struct DegreeInformation E_value, struct DegreeInformation dE_value)
{
    return (E_value.degree_value1 * dE_value.degree_value1) *
            KiMatrix[E_value.degree_index1][dE_value.degree_index1] +//1
           (E_value.degree_value1 * dE_value.degree_value2) *
            KiMatrix[E_value.degree_index1][dE_value.degree_index2] +//2
           (E_value.degree_value2 * dE_value.degree_value1) *
            KiMatrix[E_value.degree_index2][dE_value.degree_index1] +//3
           (E_value.degree_value2 * dE_value.degree_value2) *
            KiMatrix[E_value.degree_index2][dE_value.degree_index2]; //4
}
/*加权平均求解Kd输出量的量化值*/
float DeltaKd_average(struct DegreeInformation E_value, struct DegreeInformation dE_value)
{
    return (E_value.degree_value1 * dE_value.degree_value1) *
            KdMatrix[E_value.degree_index1][dE_value.degree_index1] +//1
           (E_value.degree_value1 * dE_value.degree_value2) *
            KdMatrix[E_value.degree_index1][dE_value.degree_index2] +//2
           (E_value.degree_value2 * dE_value.degree_value1) *
            KdMatrix[E_value.degree_index2][dE_value.degree_index1] +//3
           (E_value.degree_value2 * dE_value.degree_value2) *
            KdMatrix[E_value.degree_index2][dE_value.degree_index2]; //4
}

void PIDAdapteControl(struct PIDParameter* pid_para)
{
    E_value.q_value  = E_Map(pid_para);
    dE_value.q_value = dE_Map(pid_para);

    E_value.degree_value1  = Fun_e(E_value.q_value, (float *)&E_value.degree_index1);
    E_value.degree_value2  = Fun_de(E_value.q_value, (float *)&E_value.degree_index2);
    dE_value.degree_value1 = Fun_e(dE_value.q_value, (float *)&dE_value.degree_index1);
    dE_value.degree_value2 = Fun_de(dE_value.q_value, (float *)&dE_value.degree_index2);

    float delta_Kp = DeltaKp_average(E_value, dE_value);
    float delta_Ki = DeltaKi_average(E_value, dE_value);
    float delta_Kd = DeltaKd_average(E_value, dE_value);

    float m_Kp = ErrorReMap(pid_para->FuzzyParameter.dKpMax,
                            pid_para->FuzzyParameter.dKpMin, delta_Kp);
    float m_Ki = ErrorReMap(pid_para->FuzzyParameter.dKiMax,
                            pid_para->FuzzyParameter.dKiMin, delta_Ki);
    float m_Kd = ErrorReMap(pid_para->FuzzyParameter.dKdMax,
                            pid_para->FuzzyParameter.dKdMin, delta_Kd);

    // pid_para->Kd = m_Kd;
    // pid_para->Kp = m_Kp;
    // pid_para->Ki = m_Ki;
}

int PIDCompute(struct PIDParameter* pid_para, bool isAuto)
{
    int error = pid_para->TargetValue - pid_para->RealValue;

    // if(isAuto)
    // {
    //    PIDAdapteControl(pid_para);
    // }

    pid_para->ErrorTotal += error;
    int value = pid_para->Kp * error +
                pid_para->Ki * pid_para->ErrorTotal +
                pid_para->Kd * (error - pid_para->ErrorLast);
    pid_para->ErrorLast = error;

    return value;
}

int PIDCompute_Incr(struct PIDParameter_Incr* pid_para, bool isAuto)
{
//Kp*(e(k)-e(k-1)) + Ki*e(k) + Kd*(e(k)-2e(k-1)+e(k-2))
    // if(isAuto)
    // {
    //    PIDAdapteControl(pid_para);
    // }

    int value = pid_para->Kp * (pid_para->Error - pid_para->ErrorLast_1) +
                pid_para->Ki *  pid_para->Error +
                pid_para->Kd * (pid_para->Error - 2 * pid_para->ErrorLast_1 + pid_para->ErrorLast_2);

    pid_para->ErrorLast_2 = pid_para->ErrorLast_1;
    pid_para->ErrorLast_1 = pid_para->Error;

    return value;
}
