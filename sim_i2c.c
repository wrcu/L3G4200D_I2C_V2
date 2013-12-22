/*********************************Copyright (c)*********************************
**                               
**
**--------------File Info-------------------------------------------------------
** File Name:               sim_i2c.c
** Last modified Date:      
** Last Version:            
** Description:             模擬I2C接口(默認100kbps)
**
**------------------------------------------------------------------------------
** Created By:              wanxuncpx
** Created date:            
** Version:                 
** Descriptions:            
**
*******************************************************************************/

/******************************************************************************
更新說明:
    
******************************************************************************/

/******************************************************************************
*********************************  應 用 資 料 ********************************
******************************************************************************/

/******************************************************************************
********************************* 文件引用部分 ********************************
******************************************************************************/
#include "sim_i2c.h"


/******************************************************************************
******************************* 自定義參數配置 ********************************
******************************************************************************/


/******************************************************************************
********************************* 數 據 聲 明 *********************************
******************************************************************************/
/*---------------------* 
*    IMPORT:由外提供   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:向外提供   * 
*----------------------*/
//none

/******************************************************************************
********************************* 函 數 聲 明 *********************************
******************************************************************************/
char  test=0;
/*---------------------* 
*    IMPORT:由外提供   * 
*----------------------*/
//none

/*---------------------* 
*    EXPORT:向外提供   * 
*----------------------*/
//none



/******************************************************************************
*********************************  程序開始  **********************************
******************************************************************************/
/******************************************************************************
/ 函數功能:ms級延時函數
/ 修改日期:none
/ 輸入參數:none
/ 輸出參數:none
/ 使用說明:none
******************************************************************************/
void Delayms(uint32_t time)
{
   uint16_t i=0;  
   while(time--)
   {
      i=8000;
      while(i--);
   }  
}

/******************************************************************************
/ 函數功能:I2C接口配置程序
/ 修改日期:none
/ 輸入參數:none
/ 輸出參數:none
/ 使用說明:none
******************************************************************************/
void I2C_config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
    
  //關閉JTAG使能SWD以釋放IO口
/*  uint32_t u32Temp;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB , ENABLE);
  u32Temp = AFIO->MAPR;
  u32Temp &= ~AFIO_MAPR_SWJ_CFG ;
  u32Temp |= AFIO_MAPR_SWJ_CFG_1;
  AFIO->MAPR = u32Temp;
*/
    GPIO_InitTypeDef GPIO_InitStruct;
    I2C_InitTypeDef I2C_InitStruct;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL pini 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA pini

    I2C_InitStruct.I2C_ClockSpeed = 100000;    //i2c hızı normal modda                
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);

    I2C_Cmd(I2C1, ENABLE);
  
  //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB , ENABLE);
  //GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);
  
  //配置I2C的SCL引腳和SDA引腳
  /*RCC_APB2PeriphClockCmd(AHRS_I2C_GPIO_CLK , ENABLE);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin =  AHRS_I2C_SclPin;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_InitStructure.GPIO_Pin =   AHRS_I2C_SdaPin;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  SCL_H;SDA_H;    //總線釋放
  
  //配置其他IO?
  RCC_APB2PeriphClockCmd(LED1_GPIO_CLK | LED2_GPIO_CLK , ENABLE);
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin =  LED1_GPIO_PIN;
  GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =  LED2_GPIO_PIN;
  GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);
  
  //配置輸入
  RCC_APB2PeriphClockCmd(KEY_GPIO_CLK, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Pin =  KEY_GPIO_PIN;
  GPIO_Init(KEY_GPIO_PORT, &GPIO_InitStructure);*/
}

/******************************************************************************
/ 函數功能:Simulation IIC Timing series delay
/ 修改日期:none
/ 輸入參數:none
/ 輸出參數:none
/ 使用說明:none
******************************************************************************/
__inline void I2C_delay(void)
{
        
   u8 i=I2C_DELAY_VAL; //這裡可以優化速度   ，經測試最低到5還能寫入
   while(i) 
   { 
     i--; 
   }  
}


/******************************************************************************
/ 函數功能:延時5ms時間
/ 修改日期:none
/ 輸入參數:none
/ 輸出參數:none
/ 使用說明:none
******************************************************************************/
void delay5ms(void)
{
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}

/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather  Start
****************************************************************************** */
bool I2C_Start(void)
{
    SDA_H;
    SCL_H;
    //GPIO_SetBits(GPIOB,GPIO_Pin_7);  //sda=1
    //GPIO_SetBits(GPIOB,GPIO_Pin_6);  //scl=1
    
    I2C_delay();
    if(!SDA_read)return false;  //SDA線為低電平則總線忙,退出
    SDA_L;
    I2C_delay();
    if(SDA_read) return false;  //SDA線為高電平則總線出錯,退出
    SDA_L;
    I2C_delay();
    return true;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_Ack(void)
{   
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_NoAck(void)
{   
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather  Reserive Slave Acknowledge Single
****************************************************************************** */
bool I2C_WaitAck(void)   //返回為:=1有ACK,=0無ACK
{
    SCL_L;
    I2C_delay();
    SDA_H;          
    I2C_delay();
    SCL_H;
    I2C_delay();
    if(SDA_read)
    {
      SCL_L;
      I2C_delay();
      return false;
    }
    SCL_L;
    I2C_delay();
    return true;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C_SendByte(u8 SendByte) //數據從高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L;
        I2C_delay();
      if(SendByte&0x80)
        SDA_H;  
      else 
        SDA_L;   
        SendByte<<=1;
        I2C_delay();
        SCL_H;
        I2C_delay();
    }
    SCL_L;
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
uint8_t I2C_RadeByte(void)  //數據從高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H;              
    while(i--)
    {
        ReceiveByte<<=1;      
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();  
        if(SDA_read)
        {
          ReceiveByte|=0x01;
        }
    }
    SCL_L;
    return ReceiveByte;
} 
/******************************************************************************
/ 函數功能:單字節寫入
/ 修改日期:none
/ 輸入參數:
/   @arg SlaveAddress   從器件地址
/   @arg REG_Address    寄存器地址
/   @arg REG_data       欲寫入的字節數據
/ 輸出參數: 讀出的字節數據
/ 使用說明:這時一個完整的單字節讀取函數
******************************************************************************/
bool Single_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
    if(!I2C_Start())return false;
    I2C_SendByte(SlaveAddress);   //發送設備地址+寫信號//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//設置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return false;}
    I2C_SendByte(REG_Address );   //設置低起始地址      
    I2C_WaitAck();  
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    //delay5ms();
    return true;
}

/******************************************************************************
/ 函數功能:單字節寫入
/ 修改日期:none
/ 輸入參數:
/   @arg SlaveAddress   從器件地址
/   @arg REG_Address    寄存器地址
/   @arg REG_data       欲寫入的字節數據
/ 輸出參數: 讀出的字節數據
/ 使用說明:這時一個完整的單字節讀取函數
******************************************************************************/
bool Fast_Write(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t REG_data)
{
    if(!I2C_Start())return false;
    I2C_SendByte(SlaveAddress);   //發送設備地址+寫信號//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//設置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop(); return false;}
    I2C_SendByte(REG_Address );   //設置低起始地址      
    I2C_WaitAck();  
    I2C_SendByte(REG_data);
    I2C_WaitAck();   
    I2C_Stop(); 
    return true;
}



/******************************************************************************
/ 函數功能:單字節寫入
/ 修改日期:none
/ 輸入參數:
/   @arg SlaveAddress   從器件地址
/   @arg REG_Address    寄存器地址
/ 輸出參數: 讀出的字節數據
/ 使用說明:這時一個完整的單字節讀取函數
******************************************************************************/
uint8_t Single_Read(uint8_t SlaveAddress,uint8_t REG_Address)
{   
    uint8_t REG_data;       
    if(!I2C_Start())return false;
    I2C_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//設置高起始地址+器件地址 
    if(!I2C_WaitAck()){I2C_Stop();return false;}
    I2C_SendByte((u8) REG_Address);   //設置低起始地址      
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();

    REG_data= I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    //return true;
    return REG_data;
}

/******************************************************************************
/ 函數功能:多字節讀出函數
/ 修改日期:none
/ 輸入參數:
/   @arg SlaveAddress   從器件地址
/   @arg REG_Address    寄存器地址
/   @arg ptChar         輸出緩衝
/   @arg size           讀出的數據個數,size必須大於=1
/ 輸出參數: 成功失敗標記
/ 使用說明:none
******************************************************************************/
bool Mult_Read(uint8_t SlaveAddress,uint8_t REG_Address,uint8_t * ptChar,uint8_t size)
{
    uint8_t i;
    
    if(size < 1)return false;
    if(!I2C_Start())return false;
    I2C_SendByte(SlaveAddress);
    if(!I2C_WaitAck()){I2C_Stop();return false;}
    I2C_SendByte(REG_Address);    
    I2C_WaitAck();
    
    I2C_Start();
    I2C_SendByte(SlaveAddress+1);
    I2C_WaitAck();
    
    //連續讀出ax,ay,az數據
    for(i=1;i<size; i++)
    {
        *ptChar++ = I2C_RadeByte();
        I2C_Ack();
    }
    *ptChar++ = I2C_RadeByte();
    I2C_NoAck();
    I2C_Stop();
    return true;    
}




