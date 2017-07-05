///**
//  ********************************  STM32F0xx  *********************************
//  * @�ļ���     �� process.c
//  * @����       �� HarryZeng
//  * @��汾     �� V1.5.0
//  * @�ļ��汾   �� V1.0.0
//  * @����       �� 2017��04��21��
//  * @ժҪ       �� ���ݴ���
//  ******************************************************************************/
///*----------------------------------------------------------------------------
//  ������־:
//  2017-04-21 V1.0.0:��ʼ�汾
//  ----------------------------------------------------------------------------*/
///* ������ͷ�ļ� --------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __process_H
#define __process_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "stm32f0xx.h"

#define ADC_IN_Pin 				GPIO_Pin_0
#define ADC_IN_GPIO_Port 	GPIOA
#define PWM1_Pin 					GPIO_Pin_2
#define PWM1_GPIO_Port 		GPIOA
#define KG_Pin 						GPIO_Pin_3
#define KG_GPIO_Port 			GPIOA
#define SET_Pin 					GPIO_Pin_4
#define SET_GPIO_Port 		GPIOA
#define PWMX_Pin 					GPIO_Pin_8
#define PWMX_GPIO_Port		GPIOA
#define PWMY_Pin 					GPIO_Pin_9
#define PWMY_GPIO_Port 		GPIOA
#define PWMZ_Pin 					GPIO_Pin_10
#define PWMZ_GPIO_Port 		GPIOA
#define SC_Pin 						GPIO_Pin_11
#define SC_GPIO_Port 			GPIOA
#define OUT_Pin 					GPIO_Pin_12
#define OUT_GPIO_Port 		GPIOA	 
	 
	 
#define 	FLASH_START_ADDR 	 						0x08007000

/*Ӧ��ֵ*/	
#define difference_20  20
#define difference_50  50
#define difference_80  80

#define PWM1_HIGH  	12
#define PWM1_LOW  	0
#define PWMx_HIGH  	5
#define PWMx_LOW  	0


/*PWMͨ��״̬���� �궨��*/
#define PWM1_ON				TIM_SetCompare3(TIM2, PWM1_HIGH);					
#define PWM1_OFF			TIM_SetCompare3(TIM2,PWM1_LOW)

#define PWMX_ON				TIM_SetCompare1(TIM1,PWMx_HIGH)
#define PWMX_OFF			TIM_SetCompare1(TIM1,PWMx_LOW)

#define PWMY_ON				TIM_SetCompare2(TIM1,PWMx_HIGH)
#define PWMY_OFF			TIM_SetCompare2(TIM1,PWMx_LOW)

#define PWMZ_ON				TIM_SetCompare3(TIM1,PWMx_HIGH)
#define PWMZ_OFF			TIM_SetCompare3(TIM1,PWMx_LOW)

#define SETPin				GPIO_ReadInputDataBit(SET_GPIO_Port,SET_Pin)

#define shortKEY 	50 
#define middleKEY	100
#define longKEY		150	 
	 
extern uint32_t ADC_value;	 
	 
typedef enum
{
  PWMX = 0U,
  PWMY,
	PWMZ
}PWM_Number;
	 

typedef enum
{
  PWM_ON = 0U,
	PWM_OFF
}PWM_STATE;
	 


void DataProcess(void);



#ifdef __cplusplus
}
#endif
#endif 
/**** Copyright (C)2017 HarryZeng. All Rights Reserved **** END OF FILE ****/

