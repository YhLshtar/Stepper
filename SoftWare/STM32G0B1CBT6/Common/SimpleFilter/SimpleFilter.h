#ifndef __SIMPLEFILTER_H
#define __SIMPLEFILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>

#define		AvgFilterType				float
#define		MedFilterType				float
#define		MedAvgFilterType		float
#define		SliFilterType				AvgFilterType
#define		LimitFilterType			float
#define		GlitchFilterType		float
#define		LpfFilterType				float
#define		SecLpfFilterType		float
#define		HpfFilterType				float
#define   fKalmanFilterType		float
#define   KalmanFilterType		float
	
typedef	struct LimitInfo
{
	LimitFilterType	limit_value;
	LimitFilterType	old_data;//初始化时赋值只要不为0就行
}Limit_Parameter;

typedef	struct GlitchInfo
{
	unsigned char target_count;
	unsigned char count;
	GlitchFilterType old_data;//初始化时赋值只要不为0就行
}Glitch_Parameter;

typedef struct SlideInfo
{
	struct SlideQueue
	{
		unsigned char queue_head;
		unsigned char queue_tail;
		unsigned char queue_len;
	}queue_info;
	unsigned char  slide_len;
	SliFilterType* queue;
}Slide_Parameter;

typedef struct LpfInfo
{
	unsigned char ratio;//0 - 255
	LpfFilterType old_data;	
}Lpf_Parameter;

typedef struct SecLpfInfo
{
	struct Parameter
	{
		float sample_freq;//采样频率
		float cutoff_freq;//截至频率
	}parameter;
	
	struct Coefficient
	{
		float old_sample_freq;
		float old_cutoff_freq;
		float ohm;
		float c;
		float b0;
		float b1;
		float b2;
		float a1;
		float a2;
	}coefficient;
	
	SecLpfFilterType old_originalData1;
	SecLpfFilterType old_originalData2;
	SecLpfFilterType old_outData1;
	SecLpfFilterType old_outData2;
}Seclpf_Parameter;

typedef struct HpfInfo
{
	unsigned char ratio;//0 - 255
	HpfFilterType old_input_data;	
	HpfFilterType old_output_data;	
}Hpf_Parameter;

typedef struct fKalmanInfo
{
	float P; //估算协方差
	float K; //卡尔曼增益
	float Q; //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
	float R; //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
}fKalman_Parameter;
/*
typedef struct KalmanInfo
{
	float* P; //估算协方差
    float* K; //卡尔曼增益
    float* Q; //过程噪声协方差,Q增大，动态响应变快，收敛稳定性变坏
    float* R; //测量噪声协方差,R增大，动态响应变慢，收敛稳定性变好
}Kalman_Parameter;
*/
typedef unsigned char(*SlideFun)(SliFilterType*, unsigned short, unsigned char, SliFilterType*);

//均值滤波
unsigned char average_filter(AvgFilterType* original_data, unsigned short len, unsigned char lose,
														 AvgFilterType* out_data);
//中值滤波
unsigned char medina_filter(MedFilterType* original_data, unsigned short len, unsigned char lose,
														MedFilterType* out_data);
//中值平均滤波
unsigned char med_avg_filter(MedAvgFilterType* original_data, unsigned short len, unsigned char lose,
														 MedAvgFilterType* out_data);
//滑动滤波
unsigned char slide_filter(SliFilterType* original_data, Slide_Parameter* slide_info, SlideFun slide_fun,
													 unsigned char lose, SliFilterType* out_data);
//限幅滤波
unsigned char limiting_filter(LimitFilterType* original_data, unsigned short len, Limit_Parameter* limit_info, 
															LimitFilterType* out_data);
//消抖滤波
unsigned char glitch_filter(GlitchFilterType* original_data, unsigned short len, Glitch_Parameter* glitch_info,
														GlitchFilterType* out_data);
//一阶低通滤波
unsigned char lpf_filter(LpfFilterType* original_data, Lpf_Parameter* lpf_info, LpfFilterType* out_data);
//二阶低通滤波
unsigned char second_lpf_filter(SecLpfFilterType* original_data, Seclpf_Parameter* sec_lpf_info, 
								SecLpfFilterType* out_data);
//一阶高通滤波
unsigned char hpf_filter(HpfFilterType* original_data, Hpf_Parameter* lpf_info, HpfFilterType* out_data);
//一阶卡尔曼滤波
unsigned char fkalman_filter(fKalmanFilterType* original_data, fKalman_Parameter* fkalman_info, 
							 fKalmanFilterType* out_data);
//多阶卡尔曼滤波
/*
unsigned char kalman_filter(KalmanFilterType* original_data, Kalman_Parameter* kalman_info, 
							KalmanFilterType* out_data);
*/

#ifdef __cplusplus
}
#endif

#endif // SIMPLEFILTER_H
