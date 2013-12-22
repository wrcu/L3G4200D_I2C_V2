#include "sim_i2c.h"
#include "sensor.h"
#include <stdio.h>
#include <math.h>
#include "stm32f4_discovery.h"

/*---------------------* 
*       數據定義       * 
*----------------------*/
tg_L3G4200D_TYPE l3g4200d;

/*
********************************************************************************
** 函數名稱 ： main(void)
** 函數功能 ： 主函數
** 輸    入	： 無
** 輸    出	： 無
** 返    回	： 無
********************************************************************************
*/
///*
int main(void)
{  
    char keyVal;
    static bool sw_adxl345 = true,sw_l3g4200d = true, sw_hmc5883l=true,sw_bmp085 =true;
    
    SystemInit();
    USART3_Config();
    I2C_config();
    L3G4200D_Init();
    printf("Initial successed!\n\r");
    while(1)
    {
       // L3G4200D_MultRead(&l3g4200d);   //讀陀螺儀數據(速度:較快)
	L3G4200D_Read(&l3g4200d);
	L3G4200D_Printf(&l3g4200d);
        Delayms(10);    
    }
}


