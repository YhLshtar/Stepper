#include "SimpleFilter.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*----------------------------滤波器传入任意类型的暂未实现------------------------------*/
/*-------------可以使用void*类型，该类型地址不能单纯++，需要+sizeof(类型)----------------*/
/*---------------------------用void*也没找到高效的实现方法------------------------------*/

//加权平均----------TODO
unsigned char weighted_average(float* original_data, unsigned short len, unsigned char lose,
															 float* out_data)
{
	return 1;
}
//均值滤波
//优点：适用于对一般具有随机干扰的信号进行滤波；这种信号的特点是有一个平均值，信号在某一数值范围附近上下波动。
//缺点：对于测量速度较慢或要求数据计算速度较快的实时控制不适用；比较浪费RAM。
unsigned char average_filter(AvgFilterType* original_data, unsigned short len, unsigned char lose,
														 AvgFilterType* out_data)
{
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
	
	AvgFilterType  sum   = 0;
	unsigned short count = len;

	while(count)
	{
		sum += *original_data++;
		count--;
	}

	*out_data = (AvgFilterType)(sum/len);
	
	return 1;
}
//中值滤波
//优点：能有效克服因偶然因素引起的波动干扰；对温度、液位的变化缓慢的被测参数有良好的滤波效果。
//缺点：对流量、速度等快速变化的参数不宜。
unsigned char medina_filter(MedFilterType* original_data, unsigned short len, unsigned char lose,
														MedFilterType* out_data)
{
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
	
	unsigned short i,j;
	MedFilterType  temp = 0;
	
	unsigned short size = len * sizeof(original_data[0]);
	MedFilterType* buffer = malloc(size);
	memcpy(buffer, original_data, size);
	
	// 采样值从小到大排列（冒泡法）
	for(j = 0; j < len - 1; j++) 
	{
		
		for(i = 0; i < len - 1 - j; i++) 
		{
			if(buffer[i] > buffer[i + 1]) 
			{
				temp 		  = buffer[i];
				buffer[i] 	  = buffer[i + 1];
				buffer[i + 1] = temp;
			}
		}
	}
	
	if(len % 2 == 0)
	{
		*out_data = (buffer[len / 2] + buffer[(len / 2) - 1]) / 2;
	}
	else
	{
		*out_data = buffer[len / 2];
	}
	
	free(buffer);
	
	return 1;
}
//中值平均滤波
//优点：“中位值滤波法”+“算术平均滤波法”两种滤波法的优点。对于偶然出现的脉冲性干扰，可消除由其所引起的采样值偏差。对周期干扰有良好的抑制作用。平滑度高，适于高频振荡的系统。
//缺点：计算速度较慢，和算术平均滤波法一样；比较浪费RAM。
unsigned char med_avg_filter(MedAvgFilterType* original_data, unsigned short len, unsigned char lose,
														 MedAvgFilterType* out_data)
{
	if((original_data == NULL) || (out_data == NULL) || len > lose * 2)  
		return 0;
	
	unsigned short i,j;
	MedAvgFilterType  temp = 0;
	MedAvgFilterType  sum  = 0;
	unsigned short    size = len * sizeof(original_data[0]);
	MedAvgFilterType* buffer = malloc(size);
	memcpy(buffer, original_data, size);
	
	// 采样值从小到大排列（冒泡法）
	for(j = 0; j < len - 1; j++) 
	{
		
		for(i = 0; i < len - 1 - j; i++) 
		{
			if(buffer[i] > buffer[i + 1]) 
			{
				temp 		  = buffer[i];
				buffer[i] 	  = buffer[i + 1];
				buffer[i + 1] = temp;
			}
		}
	}
	
	for(i = lose; i < len - lose; i++) 
	{
		sum += buffer[i];
	}
	*out_data =  sum / (len - 2 * lose);

	return 1;
}
//滑动滤波
unsigned char slide_filter(SliFilterType* original_data, Slide_Parameter* slide_info, SlideFun slide_fun,
													 unsigned char lose, SliFilterType* out_data)
{
	unsigned char status = 1;
	
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
	
	slide_info->queue[slide_info->queue_info.queue_head] = *original_data;
	if(slide_info->queue_info.queue_head - slide_info->queue_info.queue_tail != slide_info->slide_len - 1 &&
		 slide_info->slide_len - 1 != slide_info->queue_info.queue_len)
	{
		slide_info->queue_info.queue_len++;
		*out_data = *original_data;
	}
	else
	{
		status = slide_fun(slide_info->queue, slide_info->slide_len, lose, out_data);
	}
	slide_info->queue_info.queue_head++;
	if(slide_info->queue_info.queue_head == slide_info->slide_len)
	{
		slide_info->queue_info.queue_head = 0;
	}
	
	return status;
}
//限幅滤波
//优点：能有效克服因偶然因素引起的脉冲干扰。
//缺点：无法抑制那种周期性的干扰；平滑度差
unsigned char limiting_filter(LimitFilterType* original_data, unsigned short len, Limit_Parameter* limit_info, 
															LimitFilterType* out_data)
{
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
  
	LimitFilterType	data = *original_data;
	if(((data - limit_info->old_data) > limit_info->limit_value) || 
	   ((limit_info->old_data - data) > limit_info->limit_value))
	{
		//*out_data = limit_info->old_data + limit_info->limit_value;//    һ          
		*out_data = limit_info->old_data;
	}
	else
	{
		*out_data = data;
	}
	limit_info->old_data = *out_data;
	
	return 1;
}
//消抖滤波法
unsigned char glitch_filter(GlitchFilterType* original_data, unsigned short len, Glitch_Parameter* glitch_info,
													  GlitchFilterType* out_data)
{
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
	
	GlitchFilterType data = *original_data;
	if(data != glitch_info->old_data)
	{
		glitch_info->count++;
		if(glitch_info->count > glitch_info->target_count)
		{
			glitch_info->count = 0;
			*out_data = data;
		}
	}
	else
	{
		glitch_info->count = 0;
		*out_data = glitch_info->old_data;
	}
	glitch_info->old_data = *out_data;
	
	return 1;
}
//优点：对周期性干扰具有良好的抑制作用，适用于波动频率较高的场合
//缺点：相位滞后，灵敏度低，滞后程度取决于系数的大小，不能消除滤波频率高于采样频率的1/2的干扰信号
unsigned char lpf_filter(LpfFilterType* original_data, Lpf_Parameter* lpf_info, LpfFilterType* out_data) 
{
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
	
	*out_data =  (lpf_info->ratio * lpf_info->old_data + (255 - lpf_info->ratio) * *original_data) / 255;
	lpf_info->old_data = *out_data;

	return 1;
}
//优点：适用于需要快速响应的应用场景，可以有效地去除高频噪声和干扰，平滑信号，提取出较低频率的成分
//缺点：对于较高频率的噪声和信号成分可能无法完全消除，在截止频率附近可能会引起相位变化和振铃现象，费RAM
unsigned char second_lpf_filter(SecLpfFilterType* original_data, Seclpf_Parameter* sec_lpf_info, 
																SecLpfFilterType* out_data)
{
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
	
	static float sample_freq, cutoff_freq;
	if(sec_lpf_info->coefficient.old_sample_freq != sec_lpf_info->parameter.sample_freq || 
	   sec_lpf_info->coefficient.old_cutoff_freq != sec_lpf_info->parameter.cutoff_freq)
	{
		sec_lpf_info->coefficient.ohm = tan(3.1415 * sec_lpf_info->parameter.cutoff_freq /
																				sec_lpf_info->parameter.sample_freq);
		sec_lpf_info->coefficient.c   = 1 + 1.414 * sec_lpf_info->coefficient.ohm  + 
																		sec_lpf_info->coefficient.ohm * 
																		sec_lpf_info->coefficient.ohm;

		sec_lpf_info->coefficient.b0 = sec_lpf_info->coefficient.ohm * sec_lpf_info->coefficient.ohm / 
																	 sec_lpf_info->coefficient.c;
		sec_lpf_info->coefficient.b1 = 2.0f * sec_lpf_info->coefficient.b0;
		sec_lpf_info->coefficient.b2 = sec_lpf_info->coefficient.b0;

		sec_lpf_info->coefficient.a1 = 2.0f * (sec_lpf_info->coefficient.ohm * 
																	 sec_lpf_info->coefficient.ohm - 1.0f) / sec_lpf_info->coefficient.c;
		sec_lpf_info->coefficient.a2 = (1.0f - 1.414 * sec_lpf_info->coefficient.ohm + 
																		sec_lpf_info->coefficient.ohm * sec_lpf_info->coefficient.ohm) / 
																		sec_lpf_info->coefficient.c;
		
		sec_lpf_info->coefficient.old_sample_freq = sec_lpf_info->parameter.sample_freq;
		sec_lpf_info->coefficient.old_cutoff_freq = sec_lpf_info->parameter.cutoff_freq;
	}

	*out_data = sec_lpf_info->coefficient.b0 * *original_data + 
							sec_lpf_info->coefficient.b1 * sec_lpf_info->old_originalData1 + 
							sec_lpf_info->coefficient.b2 * sec_lpf_info->old_originalData2 - 
							sec_lpf_info->coefficient.a1 * sec_lpf_info->old_outData1 - 
							sec_lpf_info->coefficient.a2 * sec_lpf_info->old_outData2;

	sec_lpf_info->old_originalData2 = sec_lpf_info->old_originalData1;
	sec_lpf_info->old_originalData1 = *original_data;
	sec_lpf_info->old_outData2 = sec_lpf_info->old_outData1;
	sec_lpf_info->old_outData1 = *out_data;
	
	return 1;
}

unsigned char hpf_filter(HpfFilterType* original_data, Hpf_Parameter* hpf_info, HpfFilterType* out_data)
{
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
	
	*out_data =  hpf_info->ratio * (hpf_info->old_output_data + *original_data - hpf_info->old_input_data) / 255;
	hpf_info->old_input_data  = *original_data;
	hpf_info->old_output_data = *out_data;
	
	return 1;
}
/*	预测
	X(k|k-1)		是利用k -1时刻预测的当前状态结果
	X(k-1|k-1)	是k-1时刻最优值
	A						是作用在X(k-1|k-1)状态下的变换矩阵，它是算法对状态变量进行预测的依据
	B						是作用在控制量上的变换矩阵，在大多数实际情况下并没有控制增益
	U(k)				是当前状态的控制增益，一般没有这个变量，可以设为0
	1. X(k|k-1) = A * X(k-1|k-1) + BU(k)					A=1, BU(k) = 0
	P(k|k-1)		是k时刻系统协方差矩阵
	P(k-1|k-1)	是k-1时刻系统协方差矩阵
	Q(k)				是系统过程噪声的协方差。初始协方差矩阵P只要不是为0它的取值对滤波效果影响很小，都能很快收敛
	2. P(k|k-1) = A * P(k-1|k-1) * A' + Q(k) 				A=1  
*/
/*	更新  
	K(k)		是卡尔曼增益，是滤波的中间结果
	H(k)		是对象的预测矩阵
	R				是对象测量噪声的协方差矩阵,只是一个数值，作为已知条件,这个值过大过小都会使滤波效果变差
	3. K(k)  = P(k|k-1) * H'/ [H * P(k|k-1) * H' + R] 		H=1  
	X(k|k)	是k时刻状态变量最优估计值
	Z(k)		是对象的测量值
	4. X(k|k) = X(k|k-1) + Kg(k) * [Z(k) - H * X(k|k-1)]	H=1  
	P(k|k)	为k时刻协方差矩阵
	I				为单位矩阵
	5. P(k|k) = (I - K(k) * H) * P(k|k-1)					H=1, I=1  
*/
unsigned char fkalman_filter(fKalmanFilterType* original_data, fKalman_Parameter* kalman_info, 
														 fKalmanFilterType* out_data)
{
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
	
	//估算协方差方程：当前 估算协方差 = 上次更新协方差 + 过程噪声协方差
	kalman_info->P = kalman_info->P + kalman_info->Q;
	//卡尔曼增益方程：当前 卡尔曼增益 = 当前 估算协方差 / （当前 估算协方差 + 测量噪声协方差）
	kalman_info->K = kalman_info->P / (kalman_info->P + kalman_info->R);
	//更新最优值方程：当前 最优值 = 当前 估算值 + 卡尔曼增益 * （当前 测量值 - 当前 估算值）
	*out_data 		 = *out_data + kalman_info->K * (*original_data - *out_data); //当前 估算值 = 上次 最优值
	//更新 协方差 = （1 - 卡尔曼增益） * 当前 估算协方差。
	kalman_info->P = (1 - kalman_info->K) * kalman_info->P;
 
	return 1;
}
/*
unsigned char kalman_filter(KalmanFilterType* original_data, Kalman_Parameter* kalman_info, 
														KalmanFilterType* out_data)
{
	if((original_data == NULL)||(out_data == NULL))  
		return 0;
	
	//TODO

	return 1;
}
*/

