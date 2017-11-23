/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_it.h"
#include "stm32f10x_usart.h"
#include "lcd.h"
#include "i2c_ee.h"
#include "math.h"


/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC_ADC1_DMA
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)
#define HMC5883L_Addr	0x3C
#define L3G4200_Addr	0xD2
#define BMP085_Addr	0xEE
#define ADXL345_Addr	0xA6

//Private typedef for Timer
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_InitTypeDef I2C_InitStructure;
ErrorStatus HSEStartUpStatus;
uint16_t i;
/* Private function prototypes -----------------------------------------------*/
//void LongDelay(u32 nCount);
void GPIO_wakeup(void);
void GPIO_Configuration(void);    
void RCC_Configuration(void);
void HMC_Init(void);
void TIM_Init(void);
void Delayms(u32 m);
void Delayus(u32 m);

int TwosHex2int(uint8_t, uint8_t);
int Compass_get(void);
bool Compass_compare(int, int);

void turn_left(void);
void turn_right(void);
void turn_forward(void);
void turn_reverse(void);
void turn_brake(void);
void turn_back(void);
//void turn_left_testing(void);
//void turn_right_testing(void);
//void turn_forward_testing(void);
//void turn_reverse_testing(void);

void sensor_init();
float get_distance();

void servo_service(); 
void servo_init();
void servo_left();         // rotate servo by around 45 degree left
void servo_right();        // rotate servo by around 45 degree right
void servo_middle();       // rotate servo to the middle

void speed_init();
float speed_measure();  // using 1s and TIM6 to measure speed right after start

void usart_rxtx();
void NVIC_Configuration();
void USART_GPIO_Configuration();
void USART_Configuration();
void USART1_IRQHandler();
void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);


// global variable 
int initial_angle = 0;
int angle = 0;
int total_time = 0;
char bluetooth_data;
int MOTOR_SPEED = 5500;

void speed_level_1(){
    MOTOR_SPEED = 5000;
}
void speed_level_2(){
    MOTOR_SPEED = 5500;
}
void speed_level_3(){
    MOTOR_SPEED = 6000;
}

void TIM7_IRQHandler(void)
{
  // Clear TIM7 update interrupt 
  TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
  
  // increase total_time
  total_time = total_time + 30;
}

// 1: forward  2: back  3: brake  4: left  5: right
void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET){
        bluetooth_data = USART_ReceiveData(USART1);
        
        switch(bluetooth_data) {
            case 'w' :    turn_forward(); break; 
            case 's' :    turn_back();    break; 
            case 'a' :    turn_left();    break; 
            case 'd' :    turn_right();   break; 
            case 'l' :    turn_brake();   break; 
            
            case '1' :    speed_level_1();   break; 
            case '2' :    speed_level_2();   break; 
            case '3' :    speed_level_3();   break; 
            default:    break;
      }
   }
}

// main function starts here

int main(void)
{
  
  SystemInit();
  
  /* Enable FSMC, GPIOA, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */

  
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | 
                         RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);
  
  STM3210E_LCD_Init(); 
 
  //Initialization 
  GPIO_wakeup();          // GPIO wakeup
  GPIO_Configuration();   // GPIO Init
  RCC_Configuration();    // RCC Init
  I2C_EE_Init();          // I2C Init
  HMC_Init();             // HMC Init   :Timer bug lies in here. If GY-80 is there, it works
  
  
  sensor_init();
  speed_init();
  servo_init();
  TIM_Init();             // Timer Init :Disable TIM4, TIM5 in that
                          // After testing PWM, the value for PA.2, PA.3, PB.9, PB.8 are 3.3V
                          // So I decided to change the Pulse to give 0 V to stop the car
  
  
  //TIM5_CH4, TIM5_CH3, TIM4_CH3, TIM4_CH4
  /*
  Don't use:   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);   GY-80 or other program will use these.
  Use          Time_Cmd(TIM4, DISABLE)
  Actually both of them are able to stop and begin the PWM signal
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, DISABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  TIM_Cmd(TIM4, DISABLE);
  TIM_Cmd(TIM4, ENABLE);  
  */
  
  LCD_DrawString(0,0,"Ready:",6);
  
  //Program Wakeup
  while(!GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0));
  
  // Initialization
  float my_distance = 0;
  initial_angle = Compass_get();
  
  // intial_angle testing
  LCD_DrawString(0,0,"Angle:",6);
  int vv1,vv2,vv3,vv4,vv5;
  vv3 = (int)(initial_angle) / 100;
  vv2 = ((int)(initial_angle) - vv3*100)/10;
  vv1 = (int)(initial_angle) % 10;

  LCD_DrawChar(0,0x30,HexValueOffset[vv3]);
  LCD_DrawChar(0,0x38,HexValueOffset[vv2]);
  LCD_DrawChar(0,0x40,HexValueOffset[vv1]);
  Delayms(100);
  
  
  turn_forward();
  float SPEED = speed_measure();
  
  LCD_DrawString(3,0x0,"Speed:",6);
  vv3 = (int)(SPEED) / 100;
  vv2 = ((int)(SPEED) - vv3*100)/10;
  vv1 = (int)(SPEED) % 10;
  LCD_DrawChar(3,0x30,HexValueOffset[vv3]);
  LCD_DrawChar(3,0x38,HexValueOffset[vv2]);
  LCD_DrawChar(3,0x40,HexValueOffset[vv1]);
  LCD_DrawString(3,0x48,"cm/s",4);
  Delayms(100);
  
  TIM7->CNT=0;
  
  TIM_Cmd(TIM7, ENABLE);
  
  /*
  Delayms(100);
  
  TIM_Cmd(TIM7, DISABLE);
  
  vv5 = (int)(TIM7->CNT) / 10000;
  vv4 = ((int)(TIM7->CNT) - vv5*10000)/1000;
  vv3 = ((int)(TIM7->CNT) - vv5*10000 - vv4*1000)/100;
  vv2 = ((int)(TIM7->CNT) % 100)/10;
  vv1 = (int)(TIM7->CNT) % 10;
  LCD_DrawChar(3,0x20,HexValueOffset[vv5]);
  LCD_DrawChar(3,0x28,HexValueOffset[vv4]);
  LCD_DrawChar(3,0x30,HexValueOffset[vv3]);
  LCD_DrawChar(3,0x38,HexValueOffset[vv2]);
  LCD_DrawChar(3,0x40,HexValueOffset[vv1]);
  Delayms(100);
  */
  
  //usart_rxtx();
  
  while (1)
  {  
    // Do bluetooth control to let it stop
    if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)){
      turn_brake();
      total_time = total_time + (TIM7->CNT)/1200 + 3; 
      vv5 = total_time / 10000;
      vv4 = (total_time - vv5*10000)/1000;
      vv3 = (total_time - vv5*10000 - vv4*1000)/100;
      vv2 = (total_time % 100)/10;
      vv1 = total_time % 10;
      LCD_DrawChar(6,0x20,HexValueOffset[vv5]);
      LCD_DrawChar(6,0x28,HexValueOffset[vv4]);
      LCD_DrawChar(6,0x30,HexValueOffset[vv3]);
      LCD_DrawChar(6,0x38,HexValueOffset[vv2]);
      LCD_DrawChar(6,0x40,HexValueOffset[vv1]);
      LCD_DrawString(6,0x0,"Time:",5);
      Delayms(100);
      
      usart_rxtx();
      
      while(1);
    }
    turn_forward();
    //always check if there is obstacles in front, if so, servo service()
    my_distance =  get_distance();
    if(my_distance < 30){
      turn_brake();
      servo_service();
    }
    Delayms(5);
    //Delayms(100); 
  }
}

void usart_rxtx(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    NVIC_Configuration();
    USART_GPIO_Configuration();
    USART_Configuration();
  
    /* Enable the USART1 Receive interrupt: this interrupt is generated when the
         USART1 receive data register is not empty */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
  
    // const unsigned char welcome_str[] = " Welcome to Bluetooth!\r\n";
    // print welcome information 
    // UARTSend(welcome_str, sizeof(welcome_str));
}

void USART_GPIO_Configuration(){
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  /* Configure I2C_EE pins: SCL and SDA */
  // using pins PB.10 and PB.11
  GPIO_InitStructure.GPIO_Pin = I2C_EE_SCL | I2C_EE_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(I2C_EE_GPIO, &GPIO_InitStructure);
}

void USART_Configuration(void)
{
  USART_InitTypeDef USART_InitStructure;
  
/* USART1 configuration ------------------------------------------------------*/
  USART_InitStructure.USART_BaudRate = 9600;        // Baud Rate
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &USART_InitStructure);
  
  /* Enable USART1 */
  USART_Cmd(USART1, ENABLE);
}

void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount)
{
    //
    // Loop while there are more characters to send.
    //
    while(ulCount--)
    {
        USART_SendData(USART1, (uint16_t) *pucBuffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}

// servo module
void servo_service(){
  
  // using GY_80 to determine which direction to check first
  angle = Compass_get();
  
  // return true : check right first | false : check left first
  if(Compass_compare(angle, initial_angle))
  {                                       // check right first
      servo_right();                      // rotate servo to right
      if(get_distance() >= 30){           // if there is no obstacle at right
          servo_middle();                   // rotate servo back
          turn_right();                  
          return;
      }
      else{                                                     
          servo_middle();                   // rotate servo to check left 
          servo_left();
          if(get_distance() >= 30){       // if there is no obstacle at left
              servo_middle();              // rotate servo back
              turn_left();
              return;
          }
          else{                           // if there is obstacle at both right and left
              servo_middle();
              turn_reverse();
              return;
          }
      }
  }
  else{                                   // check left first
      servo_left();                       // rotate servo to left
      if(get_distance() >= 30){           // if there is no obstacle at left
          servo_middle();                  // rotate servo back
          turn_left();                  
          return;
      }
      else{                                                     
          servo_middle();                  // rotate servo to check right
          servo_right();
          if(get_distance() >= 30){       // if there is no obstacle at right
              servo_middle();               // rotate servo back
              turn_right();
              return;
          }
          else{                           // if there is obstacle at both right and left
              servo_middle();
              turn_reverse();
              return;
          }
      }
  }
}
void servo_init(){
  GPIO_InitTypeDef GPIO_InitStructure;
    
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

//Configure PA.6 as Alternate Output at 50MHz for TIM3_CH1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //Setting the TIM3 timebase
  TIM_TimeBaseStructure.TIM_Period = 199;  
  TIM_TimeBaseStructure.TIM_Prescaler = 7199;   // 7200*2 = 14400
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
 //Setting the TIM3 output compare
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse =14;     
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  
  //Enable the TIM3
  TIM_Cmd(TIM3, ENABLE);
}
void servo_left(){     
   TIM_OCInitStructure.TIM_Pulse =21; 
   TIM_OC1Init(TIM3, &TIM_OCInitStructure);
   Delayms(100);
}
void servo_right(){
   TIM_OCInitStructure.TIM_Pulse =8; 
   TIM_OC1Init(TIM3, &TIM_OCInitStructure);
   Delayms(100);
}
void servo_middle(){
   TIM_OCInitStructure.TIM_Pulse =14; 
   TIM_OC1Init(TIM3, &TIM_OCInitStructure);
   Delayms(100);
}

//Ultrasound module
void sensor_init(){
  //using TIM2 to count value
  //TRIG: PB.6
  //ECHO: PA.4    using IPD.
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  
  //Configure PB.6 as Output PushPull at 50MHz
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_SetBits(GPIOB, GPIO_Pin_6); 
  
  //Setting the TIM2 timebase
  TIM_DeInit(TIM2);   //reset register value
  TIM_TimeBaseStructure.TIM_Period = 49999;  // 72000000/1440 - 1
  TIM_TimeBaseStructure.TIM_Prescaler = 71;  // 72000000/72 = 1M Hz, prescaler sets the frequency
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  //PA.4    IPD.
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
}

float get_distance(){
  GPIO_ResetBits(GPIOB, GPIO_Pin_6);
  Delayus(4);   //delay 20us
  GPIO_SetBits(GPIOB, GPIO_Pin_6);
  
  TIM2->CNT=0;
  
  while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0);
 
  TIM_Cmd(TIM2, ENABLE);
 
  while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1);
 
  TIM_Cmd(TIM2, DISABLE);

  u16 count=TIM2->CNT;          // one count is 1 us
  float distance = count*0.017; //(10^-6)*340*100/2
  //Delayms(50);
  return distance;
}

void speed_init(){
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  
  // PA.5   IPD   IN_FLOATING
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure); 
  
  //Setting TIM6 timebase
  TIM_DeInit(TIM6);   //reset register value
  TIM_TimeBaseStructure.TIM_Period = 9999;  
  TIM_TimeBaseStructure.TIM_Prescaler = 35999;  // 72000000/36000 = 2000 Hz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
  
  /* Enable the TIM7 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  //Setting TIM7 timebase
  TIM_DeInit(TIM7);   //reset register value
  //TIM_TimeBaseStructure.TIM_Period = 65530;   // maximum 65531/1098.649577 = 59.64686227 s
  TIM_TimeBaseStructure.TIM_Period = 35999;     // 30 second one interrupt
  TIM_TimeBaseStructure.TIM_Prescaler = 59999;  // 72000000/60000 = 1200 Hz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
  
  TIM_OCStructInit(&TIM_OCInitStructure);
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_Pulse = 0x0;  
  TIM_OC1Init(TIM7, &TIM_OCInitStructure);

  /* TIM7 disable counter */
  TIM_Cmd(TIM7, DISABLE);
  
  /* Immediate load of TIM7 Precaler value */  
  TIM_PrescalerConfig(TIM7, 59999, TIM_PSCReloadMode_Immediate);

  /* Clear TIM7 update pending flag */
  TIM_ClearFlag(TIM7, TIM_FLAG_Update);

  /* Enable TIM7 Update interrupt */
  TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
}

// 0.5s may not be sufficient, may change to 1s
float speed_measure(){
  float count = 0;
  
  TIM6->CNT=0;
  
  TIM_Cmd(TIM6, ENABLE);
  
  // count+1 when PA.5 encounters a rising edge
  // (1/2000)*2000 = 1s
  while(TIM6->CNT <= 2000){
      while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 1);
          count = count + 1;
      while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) == 0);
  }
 
  TIM_Cmd(TIM6, DISABLE);  
  
  // 1s contains #count high values -> f = count / #holes(20)
  // diameter d = 6.5cm  v = wr = 2*pi*f*(d/2) = pi*f*d
  // 6 is the offset
  float speed = 3.1415926*(count/20)*6.5 + 6;   
  return speed;
}


//turn_function
void turn_forward(){
  TIM_Cmd(TIM7, ENABLE);
  //left
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //right
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
}
//going backward for a while, rotate the car, going to the rear
void turn_reverse(){
  TIM_Cmd(TIM7, DISABLE);
  //left
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //right
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  Delayms(150);
  turn_forward();
}
void turn_left(){
  TIM_Cmd(TIM7, DISABLE);
  //left
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //right
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  Delayms(65);
  turn_forward();
}
void turn_right(){
  TIM_Cmd(TIM7, DISABLE);
  //left
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //right
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  Delayms(65);
  turn_forward();
}
void turn_brake(){
  TIM_Cmd(TIM7, DISABLE);
  //left
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //right
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
}

void turn_back(){
  TIM_Cmd(TIM7, DISABLE);
  //left
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  //right
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_Pulse = MOTOR_SPEED;
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
}

void TIM_Init(){
  /* Generate a PWM of 10kHz, 30% duty cycle via TIM4_CH4 */
  /* Generate a PWM of 10kHz, 30% duty cycle via TIM4_CH3 */
  /* Generate a PWM of 10kHz, 30% duty cycle via TIM5_CH4 */
  /* Generate a PWM of 10kHz, 30% duty cycle via TIM5_CH3 */
  
  GPIO_InitTypeDef GPIO_InitStructure;
  //Enable clocks for TIM4, GPIOA, AFIO.. what is AFIO
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  //Configure PB.9 as Alternate Output at 50MHz for TIM4_CH4
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //Setting the TIM4 timebase
  TIM_TimeBaseStructure.TIM_Period = 7199;     // 72000000/10k - 1
  //TIM_TimeBaseStructure.TIM_Period = 11999;  // 72000000/6000 - 1
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  //Setting the TIM4 output compare
  //For TIM4_CH4
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;     // 7200 * 0.3 = 2160
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  
  //Enable the TIM4
  TIM_Cmd(TIM4, ENABLE);
 
  //  TIM4_CH3
  //Enable clocks for TIM4, GPIOB, AFIO.. what is AFIO
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  //Configure PB.8 as Alternate Output at 50MHz for TIM4_CH3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //Setting the TIM4 timebase
  TIM_TimeBaseStructure.TIM_Period = 7199;  // 72000000/10k - 1
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  //Setting the TIM4 output compare
  //For TIM4_CH3
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;     // 7200 * 0.3 = 2160
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  
  //Enable the TIM4
  TIM_Cmd(TIM4, ENABLE);
  
  //  TIM5_CH4
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  TIM_TimeBaseStructure.TIM_Period = 7199;  // 72000000/10k - 1
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;     // 7200 * 0.3 = 2160
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  //For TIM5_CH4
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
  
  //Enable the TIM5
  TIM_Cmd(TIM5, ENABLE);
  
  //  TIM5_CH3
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  TIM_TimeBaseStructure.TIM_Period = 7199;  // 72000000/10k - 1
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;     // 7200 * 0.3 = 2160
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  //For TIM5_CH3
  TIM_OC3Init(TIM5, &TIM_OCInitStructure);
  
  //Enable the TIM5
  TIM_Cmd(TIM5, ENABLE);
  
} 

int TwosHex2int(uint8_t MSB, uint8_t LSB)
{
  int sum=0;
  if(( MSB & 0x80) != 0)
    sum = sum - (int)(pow(2,15));
  
  for(int i=1; i<=7; i++)
    if( ( MSB & (0x80 >> i)) != 0 )
      sum = sum + (int)(pow(2,15-i));
  
  for(int i=0; i<=7; i++)
    if( ( LSB & (0x80 >> i)) != 0 )
      sum = sum + (int)(pow(2,7-i));
  
  return sum;
}

int Compass_get()
{
    /* Please add code below to complete the LAB6 */
    /* You might want to create your own functions */
    
    // Write the Mode Register (02) with value 01.
    I2C_ByteWrite(HMC5883L_Addr, 0x02, 0x01); 
    
    // Wait 6 ms or monitor status register or DRDY hardware interrupt pin
    Delayms(10);
    
    // Read X, Z, Y values from registers
    uint8_t X_MSB = I2C_ByteRead(HMC5883L_Addr, 0x03);
    uint8_t X_LSB = I2C_ByteRead(HMC5883L_Addr, 0x04);
    //uint8_t Z_MSB = I2C_ByteRead(HMC5883L_Addr, 0x05);
    //uint8_t Z_LSB = I2C_ByteRead(HMC5883L_Addr, 0x06);
    uint8_t Y_MSB = I2C_ByteRead(HMC5883L_Addr, 0x07);
    uint8_t Y_LSB = I2C_ByteRead(HMC5883L_Addr, 0x08);
    
    // Convert three 16-bit 2??s compliment hex values to decimal values and assign to X, Z, Y, respectively.
    int X = TwosHex2int(X_MSB, X_LSB);
    int Y = TwosHex2int(Y_MSB, Y_LSB);
    //int Z = TwosHex2int(Z_MSB, Z_LSB);

    // get the angle
    double temp_angle=0;
    double division_xy = (double)X/(double)Y;
    if(X==0 && Y>=0)      temp_angle=0;
    else if(X==0 && Y<0)  temp_angle=180;
    else{
    if(X*Y<0) temp_angle = atan( -division_xy ) * 180.0 /(3.1415926);
    if(X*Y>0) temp_angle = atan( division_xy ) * 180.0 /(3.1415926);
    
    if( Y<0 && X>0 ) temp_angle = 180-temp_angle;
    if( Y<0 && X<0 ) temp_angle = 180+temp_angle;
    if( Y>0 && X<0 ) temp_angle = 360-temp_angle ;
    }
    return ( (int)(temp_angle) );
}

// return true : check right first | false : check left first
bool Compass_compare(int temp_angle, int ini){
    if(ini >= 0 && ini <= 180){
        if( (temp_angle >= ini) && (temp_angle <= (ini+180)) )
            return TRUE;
        else  return FALSE;
    }
    else{
        if( (temp_angle >= (ini-180)) && (temp_angle <= ini) )
            return FALSE;
        else  return TRUE;
    }
}

void GPIO_wakeup(void){
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Joy_Stick GPIO Initialization
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  //GPIO_Init(GPIOB, &GPIO_InitStructure);
  */
}

void HMC_Init(void)
{
  I2C_ByteWrite(HMC5883L_Addr, 0x00, 0x70);
  I2C_ByteWrite(HMC5883L_Addr, 0x01, 0xA0);
  Delayms(10); 
}


void RCC_Configuration(void)
{
    /* RCC system reset(for debug purpose) */
//  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  { 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  
#ifndef STM32F10X_CL  
    /* PLLCLK = 8MHz * 7 = 56 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);

#else
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {}

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 7 = 56 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_7);
#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
}
/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
}
#endif

void LongDelay(u32 nCount)
{
  for(; nCount != 0; nCount--);
}

void Delayms(u32 m)
{
  u32 i;
  
  for(; m != 0; m--)	
       for (i=0; i<50000; i++);
}

//generalized from Delayms()
void Delayus(u32 m)
{
  u32 i;
  
  for(; m != 0; m--)	
       for (i=0; i<50; i++);
}
  
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
