#ifndef __pwm_H_
#define __pwm_H_

#include "stm32l1xx_hal.h"
//#include "main.h"

#define PWM_PERIOD 1000
#define PWM_LIMIT 600
#define TH 4
#define P_H 60
#define I_H 1
#define I_D_H 128
#define P_C 60
#define I_C 1
#define I_D_C 128

//TIM_OCPOLARITY_HIGH & TIM_OCPOLARITY_LOW
//TIM_CHANNEL_x
//void PWM_SetPolarity(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t OCPolarity);
static uint8_t TemperatureLinearizeTable[]={
		245,233,211,198,189,182,177,172,167,164,160,157,
		154,152,149,147,145,143,141,139,137,136,134,133,
		131,130,129,127,126,125,124,122,121,120,119,118,
		117,116,115,114,113,112,112,111,110,109,108,107,
		106,106,105,104,103,103,102,101,101,100,99,98,98,
		97,96,96,95,94,94,93,93,92,91,91,90,90,89,88,88,
		87,87,86,86,85,84,84,83,83,82,82,81,81,80,80,79,
		78,78,77,77,76,76,75,75,74,74,73,73,72,72,71,71,
		70,70,69,69,68,68,67,67,66,66,65,65,64,64,64,63,
		63,62,62,61,61,60,60,59,59,58,58,57,57,56,56,55,
		55,54,53,53,52,52,51,51,51,50,50,49,49,48,48,47,
		47,47,46,46,45,45,45,44,44,43,43,43,42,42,41,41,
		40,40,40,39,39,39,38,38,37,37,37,36,36,35,35,35,
		34,34,33,33,33,32,32,31,31,31,30,30,30,29,29,28,
		28,28,27,27,26,26,26,25,25,24,24,24,23,23,22,22,
		21,21,21,20,20,19,19,18,18,18,17,17,16,16,15,15,
		14,14,13,12,12,11,11,10,10,9,8,7,7,6,5,4,3,2,0
	};

static uint8_t _HEATING	= 1;
static uint8_t _COOLING = 0;

static uint16_t	_PELTIER_1 =	0x01;
static uint16_t	_PELTIER_2 =	0x02;
static uint16_t	_PELTIER_3 =	0x04;
static uint16_t	_PELTIER_4 =	0x08;
static uint16_t	_PELTIER_5 =	0x10;
//void PWM_Peltier_SetHC(uint16_t PeltierN, uint8_t HC);

//void PWM_Init_PeltierArray();
//N the number of data
void FourPtAvg(uint32_t N, uint8_t * ADC_data, uint32_t ADC_data_buffer[][5], uint32_t * _ADC_buffer_pt, uint32_t * avg_data);
void PWM_Peltier_Control(uint32_t * avg_in_data, uint32_t * avg_fb_data, int * dataSum);
void PWM_Peltier_SetPWM(uint32_t PeltierN, uint32_t HC, uint32_t PWM);
void PWM_Peltier_Start(uint32_t PeltierN);
//PeltierArray = PELTIER_1 + PELTIER_2 + PELTIER_3 +....
//PeltierArrayHC_** = bxxxxx
//PeltierArrayHC_now is a global variable that stores the HC information of Peltier coolers
//PeltierArrayPWM is a global array that stores the PWM information of Peltier coolers
//void PWM_Peltier_SetPeltierArray(uint16_t PeltierArray, uint16_t PeltierArrayHC_next, uint16_t * PeltierArrayHC_now, uint16_t * PeltierArrayPWM);
void PWM_Peltier_SetPeltierArray(uint32_t PeltierArray, uint32_t PeltierArrayHC, uint32_t * PeltierArrayPWM);

#endif
