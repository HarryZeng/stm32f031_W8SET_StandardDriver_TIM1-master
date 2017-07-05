#include  "stm32f0xx.h"
#include "stdio.h"
#include "process.h"

#define DMA_BUFFER_SIZE     1  


//  if ((GPIOx->ODR & GPIO_Pin) != (uint32_t)Bit_RESET)
//  {
//    bitstatus = (uint8_t)Bit_SET;
//  }
//  else
//  {
//    bitstatus = (uint8_t)Bit_RESET;
//  }


//if (BitVal != Bit_RESET)
//  {
//    GPIOx->BSRR = GPIO_Pin;
//  }
//  else
//  {
//    GPIOx->BRR = GPIO_Pin ;
//  }

#define _Gpio_12_set  GPIO_WriteBit(GPIOA, GPIO_Pin_12, (BitAction)!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_12))
#define _Gpio_7_set  GPIO_WriteBit(GPIOA, GPIO_Pin_7, (BitAction)!GPIO_ReadOutputDataBit(GPIOA, GPIO_Pin_7))

uint8_t sample_finish = 0;  
int16_t adc_dma_tab[6] = { 0 };  
uint8_t sample_index = 0;  
uint8_t TIM1step=0;

//?????  
int16_t sample_1[128] = { 0 };  
int16_t sample_2[128] = { 0 };  
int16_t sample_3[128] = { 0 };  
int16_t sample_4[128] = { 0 };  
int16_t sample_5[128] = { 0 };  
int16_t sample_6[128] = { 0 };  
  

RCC_ClocksTypeDef   SysClock;
extern void scan_key(void);
extern uint16_t scan_tick;
extern uint8_t EnterSelfFlag;
extern uint8_t 	ShortCircuitCounter;
extern uint8_t 	ShortCircuit;
void adc_config(void) ;
void user_adc_init(void)  ;
void adc_gpio_init(void);
void adc_dma_init(void) ;
void adc_timer_init(void)  ;
void USART_Config(void);
void GPIOA_Config(void);

void GPIO_DEINIT_ALL(void);
/*****************************************/
///////////////////////////////////////////////////////////////////////////////////
/**
  * @brief  //�������´���,֧��printf����,������Ҫѡ��use MicroLIB
  */
int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
		;
	USART_SendData(USART1, (unsigned char) ch);
  return (ch);
}

void user_adc_init(void)  
{  
    adc_gpio_init();  
    adc_config();               // ??????????,????????????????????  
    adc_dma_init();             //  
    adc_timer_init();           //  
		GPIOA_Config();
		GPIO_DEINIT_ALL();
    //USART_Config();
}

void USART_Config(void)
{
        USART_InitTypeDef USART_InitStructure;
        /*???USART1??*/
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        USART_InitStructure.USART_BaudRate = 115200;//???????
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//?????
        USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//??????
        USART_InitStructure.USART_Parity = USART_Parity_No;//?????
        USART_InitStructure.USART_StopBits = USART_StopBits_1;//?????
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;//?????
        USART_Init(USART1, &USART_InitStructure);
        USART_Cmd(USART1, ENABLE);//???? 1
}
/*GPIOA????? */
void GPIOA_Config(void)
{
//        GPIO_InitTypeDef GPIO_InitStructure;
//        /*???GPIOA??*/
//        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//        GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);
//        GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);
//        /* ??PA9 ,PA10*/
//        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
//        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //??????
//        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
//        GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/*?? 1 ????*/
void UART_send_byte(uint8_t byte)
{
        while(!((USART1->ISR)&(1<<7)));//?????
        USART1->TDR=byte; //??????
}
/*?????*/
void UART_Send(uint8_t *Buffer, uint32_t Length)
{
        while(Length != 0)
        {
                while(!((USART1->ISR)&(1<<7)));//?????
                USART1->TDR= *Buffer;
                Buffer++;
                Length--;
        }
}

uint8_t UART_Recive(void)
{
        while(!(USART1->ISR & (1<<5)));
        return(USART1->RDR);
}
  
void adc_config()  
{  
    ADC_InitTypeDef adc_init_structure;  

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);            //??ADC?? 
    
		ADC_DeInit(ADC1);                                               //??ADC  
    ADC_StructInit(&adc_init_structure);                            //???ADC???  
  
    adc_init_structure.ADC_ContinuousConvMode = DISABLE;            //????????  
    adc_init_structure.ADC_DataAlign = ADC_DataAlign_Right;         //???????  
    adc_init_structure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO; //???????TIM2  
    adc_init_structure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;//?????  
    adc_init_structure.ADC_Resolution = ADC_Resolution_12b;         //12????  
    adc_init_structure.ADC_ScanDirection = ADC_ScanDirection_Upward;//????0-18??  
		
    ADC_Init(ADC1, &adc_init_structure);  
		
    ADC_OverrunModeCmd(ADC1, ENABLE);                               //????????  
    ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_28_5Cycles);               //??????,????125nS  
    ADC_GetCalibrationFactor(ADC1);                                 //?????ADC  
    ADC_Cmd(ADC1, ENABLE);                                          //??ADC1  
    while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET);         //??ADC1????  
  
    ADC_DMACmd(ADC1, ENABLE);                                       //??ADC_DMA  
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);           //??DMA?????????  
    ADC_StartOfConversion(ADC1);                                    //??????(??)  
}  
  
void adc_gpio_init()  
{  
    GPIO_InitTypeDef gpio_init_structure;  
    //??GPIO??  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA , ENABLE);  
  
    GPIO_StructInit(&gpio_init_structure);  
    //GPIOA                                                         //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = (GPIO_Pin_0);  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_AN;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_UP;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure);  
}  
  
void adc_dma_init()  
{  
    DMA_InitTypeDef dma_init_structure;  
    NVIC_InitTypeDef nvic_init_structure;  
  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);              //??DMA??  
  
    nvic_init_structure.NVIC_IRQChannel = DMA1_Channel1_IRQn;       //??DMA1????  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //????  
    nvic_init_structure.NVIC_IRQChannelPriority = 0;                //?????0  
    NVIC_Init(&nvic_init_structure);  
  
    DMA_DeInit(DMA1_Channel1);                                      //��λDMA1_channel1  
    DMA_StructInit(&dma_init_structure);                            //��ʼ��DMA�ṹ�� 
  
    dma_init_structure.DMA_BufferSize = DMA_BUFFER_SIZE;            //DMA���������С  
    dma_init_structure.DMA_DIR = DMA_DIR_PeripheralSRC;             //DMA����:������Ϊ����Դ 
    dma_init_structure.DMA_M2M = DISABLE;                           //�ڴ浽�ڴ����
    dma_init_structure.DMA_MemoryBaseAddr = (uint32_t)&adc_dma_tab[0];//��������������ʼ��ַ
    dma_init_structure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//���ݴ�С����ΪHalfword  
    dma_init_structure.DMA_MemoryInc = DMA_MemoryInc_Enable;        //�ڴ��ַ���� 
    dma_init_structure.DMA_Mode = DMA_Mode_Circular;                //DMAѭ��ģʽ,��ɺ����¿�ʼ���� 
    dma_init_structure.DMA_PeripheralBaseAddr = (uint32_t) &(ADC1->DR);//ȡֵ�������ַ 
    dma_init_structure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//����ȡֵ��С����ΪHalfword  
    dma_init_structure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ��������
    dma_init_structure.DMA_Priority = DMA_Priority_High;             //DMA���ȼ�����Ϊ��  
    DMA_Init(DMA1_Channel1, &dma_init_structure);  
  
    DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);                  //ʹ��DMA�ж�
    DMA_ClearITPendingBit(DMA_IT_TC);                                //���һ��DMA�жϱ�־  
    DMA_Cmd(DMA1_Channel1, ENABLE);                                  //ʹ��DMA1  
}  
  
void adc_timer_init()  
{  
    TIM_TimeBaseInitTypeDef timer_init_structure;  
		TIM_OCInitTypeDef timer_OCinit_structure; 
    NVIC_InitTypeDef nvic_init_structure;  
  	GPIO_InitTypeDef gpio_init_structure;  
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);            //??TIM2??  //??GPIO??  
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  
  
    GPIO_StructInit(&gpio_init_structure);  
    //GPIOA                                                         //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_2;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_AF;                   //????(??)??  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_NOPULL;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure);
	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_2);
	
    nvic_init_structure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;                //ʹ��TIM1�ж�ͨ��  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //ʹ��TIM1�ж�  
    nvic_init_structure.NVIC_IRQChannelPriority = 2;                //���ȼ�Ϊ0  
		
    NVIC_Init(&nvic_init_structure);  
		
		
		nvic_init_structure.NVIC_IRQChannel = TIM2_IRQn;                //ʹ��TIM2�ж�ͨ��  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //ʹ��TIM2�ж�  
    nvic_init_structure.NVIC_IRQChannelPriority = 0;                //���ȼ�Ϊ0  
		
    NVIC_Init(&nvic_init_structure); 
		
		
		nvic_init_structure.NVIC_IRQChannel = TIM3_IRQn;                //ʹ��TIM3�ж�ͨ��  
    nvic_init_structure.NVIC_IRQChannelCmd = ENABLE;                //ʹ��TIM3�ж�  
    nvic_init_structure.NVIC_IRQChannelPriority = 4;                //���ȼ�Ϊ4
		
    NVIC_Init(&nvic_init_structure); 

		/*TIM3*/
		TIM_DeInit(TIM3);                                               //��λTIM3
    TIM_TimeBaseStructInit(&timer_init_structure);                  //��ʼ��TIM�ṹ��  
  
    timer_init_structure.TIM_ClockDivision = TIM_CKD_DIV1;          //ϵͳʱ��,����Ƶ,24M  
    timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //���ϼ���ģʽ  
    timer_init_structure.TIM_Period = 2000;                          //ÿ300 uS����һ���ж�,??ADC  
    timer_init_structure.TIM_Prescaler = 15;                      //����ʱ�ӷ�Ƶ,f=1M,systick=1 uS  
    timer_init_structure.TIM_RepetitionCounter = 0x00;              //����0+1��update�¼������ж� 
		
    TIM_TimeBaseInit(TIM3, &timer_init_structure);  
		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);                      //ʹ��TIM3�ж�
    TIM_Cmd(TIM3, ENABLE);                                          //ʹ��TIM3
	
		/*TIM2*/
    TIM_DeInit(TIM2);                                               //��λTIM2  
    TIM_TimeBaseStructInit(&timer_init_structure);                  //��ʼ��TIM�ṹ��  
  
    timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;      //���ϼ���ģʽ  
    timer_init_structure.TIM_Period = 120;                          //ÿ300 uS����һ���ж�,??ADC  
    timer_init_structure.TIM_Prescaler = 7;                      //����ʱ�ӷ�Ƶ,f=1M,systick=1 uS  
    timer_init_structure.TIM_RepetitionCounter = 0;              //����0+1��update�¼������ж� 
		
    TIM_TimeBaseInit(TIM2, &timer_init_structure);  
  
		timer_OCinit_structure.TIM_OCMode = TIM_OCMode_PWM1;
		timer_OCinit_structure.TIM_OutputState = TIM_OutputState_Enable;
		timer_OCinit_structure.TIM_Pulse = PWM1_HIGH;
		timer_OCinit_structure.TIM_OCPolarity = TIM_OCPolarity_High;
		
		
		TIM_OC3Init(TIM2,&timer_OCinit_structure);
		TIM_OC3PreloadConfig(TIM2,TIM_OCPreload_Enable);
		TIM_ARRPreloadConfig(TIM2,ENABLE);
		
		
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);                      //ʹ��TIM2�ж�
		TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);           //ѡ��TIM1��updateΪ����Դ  
		TIM_SelectInputTrigger(TIM2, TIM_TS_ITR0);
		TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Gated);//����ģʽֻ�������ſ�����ͣ�����Կ���
		TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);//����ģʽMSM

		TIM_Cmd(TIM2, ENABLE);


/* TIM1 ??? ---------------------------------------------------
   TIM1 ????(TIM1CLK) ??? APB2 ?? (PCLK2)    
    => TIM1CLK = PCLK2 = SystemCoreClock
   TIM1CLK = SystemCoreClock, Prescaler = 0, TIM1 counter clock = SystemCoreClock
   SystemCoreClock ?48 MHz 
   
   ??????? 4 ?PWM ???17.57 KHz:
     - TIM1_Period = (SystemCoreClock / 17570) - 1
   ??1??????? 50%
   ??2??????? 37.5%
   ??3??????? 25%
   ??4??????? 12.5%
   ????????????:
     - ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	*/  
    /*???????,???????????????*/
  //TimerPeriod = (SystemCoreClock / 17570 ) - 1;
  //TimerPeriod = (SystemCoreClock / DEF_PWMFRE ) - 1;
  //TimerPeriod = (SystemCoreClock / DEF_PWMFRE);
  /* TIM1 ???? */
  
  
  /* Time ??????*/
  timer_init_structure.TIM_Prescaler = 7;
  timer_init_structure.TIM_CounterMode = TIM_CounterMode_Up;  /* Time ????????????*/
  timer_init_structure.TIM_Period = 120;
  timer_init_structure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &timer_init_structure);

  /* ??1,2,3,4?PWM ???? */
  timer_OCinit_structure.TIM_OCMode = TIM_OCMode_PWM1;
  timer_OCinit_structure.TIM_OutputState = TIM_OutputState_Enable ;//TIM_OutputState_Enable; //PWM?????
  timer_OCinit_structure.TIM_OutputNState = TIM_OutputNState_Disable ;//TIM_OutputNState_Enable; //??PWM?????
  timer_OCinit_structure.TIM_OCPolarity = TIM_OCPolarity_High;  //PWM 1?????
  timer_OCinit_structure.TIM_OCNPolarity = TIM_OCNPolarity_Low; //PWM?? 0?????
  timer_OCinit_structure.TIM_OCIdleState = TIM_OCIdleState_Set;
  timer_OCinit_structure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  timer_OCinit_structure.TIM_Pulse = 0; //?????
  TIM_OC1Init(TIM1, &timer_OCinit_structure);//????1??
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	timer_OCinit_structure.TIM_Pulse = 0; //?????
  TIM_OC2Init(TIM1, &timer_OCinit_structure);//????1??
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	timer_OCinit_structure.TIM_Pulse = 0; //?????
  TIM_OC3Init(TIM1, &timer_OCinit_structure);//????1??
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);

  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);                      //ʹ��TIM1�ж�
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	
  /* TIM1 ?????*/
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable);							//ѡ��TIM1��timerΪ����Դ  
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC1Ref);							//ѡ��TIM1��timerΪ����Դ  
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC2Ref);							//ѡ��TIM1��timerΪ����Դ  
	//TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC3Ref);							//ѡ��TIM1��timerΪ����Դ  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);     //���update�¼��жϱ�־
	TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);//����ģʽMSM  
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	//TIM_SelectOnePulseMode(TIM1,TIM_OPMode_Single);
	
  TIM_Cmd(TIM1, ENABLE);
	
  /* TIM1 ????? */
  

}  

///*����TIM1����OnePuleģʽ�����ڿ����������*/

//void ChangeTIM1ToOnePulse(int Counter)
//{
//	TIM_TimeBaseInitTypeDef timer_init_structure;
//	TIM_Cmd(TIM1, DISABLE);
//	
//	timer_init_structure.TIM_RepetitionCounter = Counter;
//  TIM_TimeBaseInit(TIM1, &timer_init_structure);
//	
//	
//	TIM_SelectOnePulseMode(TIM1,TIM_OPMode_Single);
//	TIM_Cmd(TIM1, DISABLE);
//}

    
/****************************??????****************************/  

void TIM2_IRQHandler()  
{  
    if(TIM_GetITStatus(TIM2, TIM_FLAG_Update))            //�жϷ���update�¼��ж�  
    {  
				//_Gpio_7_set;
        TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);     //���update�¼��жϱ�־
    }  
}  

void TIM1_BRK_UP_TRG_COM_IRQHandler()  
{  
    if(TIM_GetITStatus(TIM1, TIM_FLAG_Update))            //�жϷ���update�¼��ж�  
    {
				if(ShortCircuit)
					ShortCircuitCounter++;
				else
					ShortCircuitCounter=0;
				
				if(EnterSelfFlag)
				{
					TIM1step++;
					if(TIM1step==1)
					{
						PWMX_ON;
						PWMY_OFF;
						PWMZ_OFF;
					}
					else if(TIM1step==2)
					{
						PWMX_OFF;
						PWMY_ON;
						PWMZ_OFF;
					}
					else if(TIM1step==3)
					{
						PWMX_OFF;
						PWMY_OFF;
						PWMZ_ON;
					}
				}
      TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);     //���update�¼��жϱ�־
    }  
} 

extern uint32_t   ShortCircuitLastTime;
void TIM3_IRQHandler()
{
	  if(TIM_GetITStatus(TIM3, TIM_IT_Update))            //�жϷ���update�¼��ж�  
    { 
			scan_tick++;
			ShortCircuitLastTime++;
			if(scan_tick>=20)  /*20ms*/
			{
				scan_tick = 0;
				scan_key();
			}
			TIM_ClearITPendingBit(TIM3, TIM_IT_Update);     //���update�¼��жϱ�־
		}
}


extern uint8_t SelfGetADCWell;
uint8_t ADCIndex=0;
uint8_t DMAIndex=0;

int16_t selfADCValue[12];

void DMA1_Channel1_IRQHandler()  
{  
//		if(DMA_GetITStatus(DMA_IT_HT))
//				;
    if(DMA_GetITStatus(DMA_IT_TC))                      //�ж�DMA��������ж�  
    {   
       //sample_1[sample_index] = (adc_dma_tab[0]*3300)/4096;
			if(EnterSelfFlag&&(DMAIndex==0))
			{
				selfADCValue[ADCIndex++] = adc_dma_tab[0];
				if(ADCIndex>=12)
				{
					SelfGetADCWell=1;
					ADCIndex = 0;
					DMAIndex=1;
				}
			}
					
				sample_finish = 1;
    }  
    DMA_ClearITPendingBit(DMA_IT_TC);                   //���DMA�жϱ�־λ  
}  


void GPIO_INIT(void)
{
	
	    GPIO_InitTypeDef gpio_init_structure;  
    //??GPIO??  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  
  
    GPIO_StructInit(&gpio_init_structure);  
    //GPIOA                                                         //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = GPIO_Pin_12;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_OUT;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_NOPULL;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure);  
	
		gpio_init_structure.GPIO_Pin = GPIO_Pin_4; 
		gpio_init_structure.GPIO_Mode = GPIO_Mode_IN;                   //????(??)??  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
		gpio_init_structure.GPIO_PuPd= GPIO_PuPd_DOWN;                    //??
    GPIO_Init(GPIOA, &gpio_init_structure);  
	
	    //GPIOA                                                         //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_11;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_IN;                   //????(??)??  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_NOPULL;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure);
}

void GPIO_DEINIT_ALL(void)
{
	
	   GPIO_InitTypeDef gpio_init_structure;  
    //??GPIO??  
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOF, ENABLE);  
  
    GPIO_StructInit(&gpio_init_structure);  
                                                        //PA-0~3??ADC  
    gpio_init_structure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_OUT;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_DOWN;                    //??  
    GPIO_Init(GPIOA, &gpio_init_structure); 
	
    gpio_init_structure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_OUT;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_DOWN;                    //??  
    GPIO_Init(GPIOB, &gpio_init_structure);  

    gpio_init_structure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;  
    gpio_init_structure.GPIO_Mode = GPIO_Mode_OUT;                   //????(??)??  
    gpio_init_structure.GPIO_OType = GPIO_OType_PP;                 //????  
    gpio_init_structure.GPIO_Speed = GPIO_Speed_2MHz;              //Fast speed  
    gpio_init_structure.GPIO_PuPd= GPIO_PuPd_DOWN;                    //??  
    GPIO_Init(GPIOF, &gpio_init_structure); 
}


void RCC_Configuration(void)
{
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_4);
	RCC_PLLCmd(ENABLE);
	RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div2);
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY)==RESET)
	{
		
	}
	
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
	
	while(RCC_GetSYSCLKSource()!=0x08)
	{
	}
}

void DelaymsSet(int16_t ms)
{
		while(1)
		{
			ms--;
			if(ms<=0)
				break;
		}
}

int main(void)
{
	RCC_Configuration();
	GPIO_INIT();
	user_adc_init();
	RCC_GetClocksFreq(&SysClock);
	DelaymsSet(5000); 
	
	DataProcess();

}


