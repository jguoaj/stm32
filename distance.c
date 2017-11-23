
#include "distance.h"
#include "delay.h"


#define	TRIG_PORT_1      GPIOA		
#define	ECHO_PORT_1      GPIOA		
#define	TRIG_PORT_2      GPIOA		
#define	ECHO_PORT_2      GPIOA	
#define	TRIG_PIN_1     GPIO_Pin_3      
#define	ECHO_PIN_1     GPIO_Pin_4
#define	TRIG_PIN_2     GPIO_Pin_5     
#define	ECHO_PIN_2     GPIO_Pin_2	

void sensorInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = TRIG_PIN_1;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(TRIG_PORT_1, &GPIO_InitStructure);  
  GPIO_SetBits(TRIG_PORT_1,TRIG_PIN_1);  
  
  GPIO_InitStructure.GPIO_Pin = TRIG_PIN_2;       
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(TRIG_PORT_2, &GPIO_InitStructure);  
  GPIO_SetBits(TRIG_PORT_2,TRIG_PIN_2);  
      
  GPIO_InitStructure.GPIO_Pin = ECHO_PIN_1;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(ECHO_PORT_1, &GPIO_InitStructure); 
  
  GPIO_InitStructure.GPIO_Pin = ECHO_PIN_2;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(ECHO_PORT_2, &GPIO_InitStructure);  
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  TIM_DeInit(TIM2);
  TIM_TimeBaseStructure.TIM_Period = 49999;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_DeInit(TIM4);
  TIM_TimeBaseStructure.TIM_Period = 49999;
  TIM_TimeBaseStructure.TIM_Prescaler = 71;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
}

float getDistance1(void)
{
  GPIO_ResetBits(TRIG_PORT_1,TRIG_PIN_1);
  delay_us(20);
  GPIO_SetBits(TRIG_PORT_1,TRIG_PIN_1);
 
  TIM2->CNT=0;
  while(GPIO_ReadInputDataBit(ECHO_PORT_1,ECHO_PIN_1)==0);
 
  TIM_Cmd(TIM2, ENABLE);
 
  while(GPIO_ReadInputDataBit(ECHO_PORT_1,ECHO_PIN_1)==1);
 
  TIM_Cmd(TIM2, DISABLE);

 
  u16 count=TIM2->CNT;
  float length=count/58.0-1.1;
  delay_ms(50);
  return length;
}

float getDistance2(void)
{
  GPIO_ResetBits(TRIG_PORT_2,TRIG_PIN_2);
  delay_us(20);
  GPIO_SetBits(TRIG_PORT_2,TRIG_PIN_2);
 
  TIM4->CNT=0;
  while(GPIO_ReadInputDataBit(ECHO_PORT_2,ECHO_PIN_2)==0);
 
  TIM_Cmd(TIM4, ENABLE);
 
  while(GPIO_ReadInputDataBit(ECHO_PORT_2,ECHO_PIN_2)==1);
 
  TIM_Cmd(TIM4, DISABLE);

 
  u16 count=TIM4->CNT;
  float length=count/58.0-1.1;
  delay_ms(50);
  return length;
}

















