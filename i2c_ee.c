/* Includes ------------------------------------------------------------------*/
#include "i2c_ee.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup I2C_EEPROM
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C_Speed              200000
#define I2C_SLAVE_ADDRESS7     0xD2

#if defined (EE_M24C08)
 #define I2C_FLASH_PAGESIZE    16
#elif defined (EE_M24C64_32)
 #define I2C_FLASH_PAGESIZE    32
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t EEPROM_ADDRESS;

/* Private function prototypes -----------------------------------------------*/
void GPIO_EE_Configuration(void);
void I2C_Configuration(void);
void LongEEDelay(u32 nCount);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure the used I/O ports pin
  * @param  None
  * @retval None
  */
void GPIO_EE_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
  
  /* Configure I2C_EE pins: SCL and SDA */
  GPIO_InitStructure.GPIO_Pin =  I2C_EE_SCL | I2C_EE_SDA; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_Init(I2C_EE_GPIO, &GPIO_InitStructure);
}

/**
  * @brief  I2C Configuration
  * @param  None
  * @retval None
  */
void I2C_Configuration(void)
{
  I2C_InitTypeDef  I2C_InitStructure; 
  
  /* I2C configuration */
  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS7;
  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_InitStructure.I2C_ClockSpeed = I2C_Speed;
  
  /* I2C Peripheral Enable */
  I2C_Cmd(I2C_EE, ENABLE);
  /* Apply I2C configuration after enabling it */
  I2C_Init(I2C_EE, &I2C_InitStructure);
}

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void I2C_EE_Init()
{
  /* I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(I2C_EE_CLK, ENABLE);   
  
  /* GPIO Periph clock enable */
  RCC_APB2PeriphClockCmd(I2C_EE_GPIO_CLK, ENABLE);    
  
  /* GPIO configuration */
  GPIO_EE_Configuration();

  /* I2C configuration */
  I2C_Configuration();

}


void I2C_ByteWrite(uint8_t SlaveAddr, uint8_t WriteAddr, uint8_t WriteByte)
{

  /* Send STRAT condition */
  I2C_GenerateSTART(I2C_EE, ENABLE);

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));  

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C_EE, SlaveAddr, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  I2C_SendData(I2C_EE, WriteAddr);
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2C_EE, WriteByte); 
   
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2C_EE, ENABLE);

  /* Delay */
  LongEEDelay(0x50);
}

uint8_t I2C_ByteRead(uint8_t SlaveAddr, uint8_t ReadAddr)
{ 
   uint8_t DataNotReady = 1;
   uint8_t temp;
   
  /* While the bus is busy */
  while(I2C_GetFlagStatus(I2C_EE, I2C_FLAG_BUSY));
  
  /* Send START condition */
  I2C_GenerateSTART(I2C_EE, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));
   
  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2C_EE, SlaveAddr, I2C_Direction_Transmitter);

  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

  /* Send the EEPROM's internal address to read from: Only one byte address */
  I2C_SendData(I2C_EE, ReadAddr);  

  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STRAT condition a second time */  
  I2C_GenerateSTART(I2C_EE, ENABLE);
  
  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_MODE_SELECT));
  
  /* Send EEPROM address for read */
  I2C_Send7bitAddress(I2C_EE, SlaveAddr, I2C_Direction_Receiver);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
  
  /* While there is data to be read */
  while(DataNotReady)  
  {
    /* Disable Acknowledgement */
    I2C_AcknowledgeConfig(I2C_EE, DISABLE);
      
    /* Send STOP Condition */
    I2C_GenerateSTOP(I2C_EE, ENABLE);
    
    if(I2C_CheckEvent(I2C_EE, I2C_EVENT_MASTER_BYTE_RECEIVED))  
    {      
      /* Read a byte from the EEPROM */
      temp=I2C_ReceiveData(I2C_EE);

      /* Decrement the read bytes counter */
      DataNotReady=0;        
    }   
  }
  
  /* Enable Acknowledgement to be ready for another reception */
  I2C_AcknowledgeConfig(I2C_EE, ENABLE);

  /* Delay */
  LongEEDelay(0x4000);

  return temp;
}

/**
  * @brief  Wait for EEPROM Standby state
  * @param  None
  * @retval None
  */
void I2C_EE_WaitEepromStandbyState(void)      
{
  __IO uint16_t SR1_Tmp = 0;

  do
  {
    /* Send START condition */
    I2C_GenerateSTART(I2C_EE, ENABLE);

    /* Read I2C_EE SR1 register to clear pending flags */
    SR1_Tmp = I2C_ReadRegister(I2C_EE, I2C_Register_SR1);

    /* Send EEPROM address for write */
    I2C_Send7bitAddress(I2C_EE, EEPROM_ADDRESS, I2C_Direction_Transmitter);

  }while(!(I2C_ReadRegister(I2C_EE, I2C_Register_SR1) & 0x0002));
  
  /* Clear AF flag */
  I2C_ClearFlag(I2C_EE, I2C_FLAG_AF);
  
  /* STOP condition */    
  I2C_GenerateSTOP(I2C_EE, ENABLE);  
}


void LongEEDelay(u32 nCount)
{
  for(; nCount != 0; nCount--);
}

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
