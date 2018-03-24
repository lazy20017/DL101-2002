/* 直接操作寄存器的方法控制IO */
#include "stm32l4xx_hal.h"
#include "main.h"
#include "my_gloabal_val.h"

#define	digitalHi(p,i)			{p->BSRR=i;}			//设置为高电平		
#define digitalLo(p,i)			{p->BRR=i;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态

#if USE_LED14_STATUS==1

#define LED1_TOGGLE						digitalToggle(LED1_GPIO_Port,LED1_Pin)
#define LED1_OFF							digitalHi(LED1_GPIO_Port,LED1_Pin)
#define LED1_ON								digitalLo(LED1_GPIO_Port,LED1_Pin)
#define LED2_TOGGLE						digitalToggle(LED2_GPIO_Port,LED2_Pin)
#define LED2_OFF							digitalHi(LED2_GPIO_Port,LED2_Pin)
#define LED2_ON								digitalLo(LED2_GPIO_Port,LED2_Pin)
#define LED3_TOGGLE						digitalToggle(LED3_GPIO_Port,LED3_Pin)
#define LED3_OFF							digitalHi(LED3_GPIO_Port,LED3_Pin)
#define LED3_ON								digitalLo(LED3_GPIO_Port,LED3_Pin)
#define LED4_TOGGLE						digitalToggle(LED4_GPIO_Port,LED4_Pin)
#define LED4_OFF							digitalHi(LED4_GPIO_Port,LED4_Pin)
#define LED4_ON								digitalLo(LED4_GPIO_Port,LED4_Pin)
#define WDI1_TOGGLE						digitalToggle(WDI1_GPIO_Port,WDI1_Pin)
#define WDI1_OFF							digitalHi(WDI1_GPIO_Port,WDI1_Pin)
#define WDI1_ON								digitalLo(WDI1_GPIO_Port,WDI1_Pin)

#else

#define LED1_TOGGLE						;
#define LED1_OFF							;
#define LED1_ON								;

#define LED2_TOGGLE						;
#define LED2_OFF							;
#define LED2_ON								;

#define LED3_TOGGLE						;
#define LED3_OFF							;
#define LED3_ON								;

#define LED4_TOGGLE						;
#define LED4_OFF							;
#define LED4_ON								;

#define WDI1_TOGGLE						digitalToggle(WDI1_GPIO_Port,Source_Ctrl_Pin)
#define WDI1_OFF							digitalLo(WDI1_GPIO_Port,Source_Ctrl_Pin)
#define WDI1_ON								digitalHi(WDI1_GPIO_Port,Source_Ctrl_Pin)

#endif




#define CC1101_PWR_ON   HAL_GPIO_WritePin(C433_CTRL_GPIO_Port,C433_CTRL_Pin,GPIO_PIN_RESET)      //CC1101供电
#define CC1101_PWR_OFF  HAL_GPIO_WritePin(C433_CTRL_GPIO_Port,C433_CTRL_Pin,GPIO_PIN_SET)        //CC1101断电


//----常用管脚设定

//电源管理

#define GANdianchi_ON  digitalLo(Gandianchi_V3V3_Ctrl_GPIO_Port,Gandianchi_V3V3_Ctrl_Pin) //开干电池，默认，低电平
#define GANdianchi_OFF  digitalHi(Gandianchi_V3V3_Ctrl_GPIO_Port,Gandianchi_V3V3_Ctrl_Pin) //关干电池

#define EN25505_ON  digitalLo(EN_2505_Ctrl_GPIO_Port,EN_2505_Ctrl_Pin) //开EN25505,工作，有输出，默认
#define EN25505_OFF  digitalHi(EN_2505_Ctrl_GPIO_Port,EN_2505_Ctrl_Pin) //关EN25505，不工作，无输出

#define CT_to_BQ25505_ON 	digitalLo(ZAIXIN_IN2505_Ctrl_GPIO_Port,ZAIXIN_IN2505_Ctrl_Pin) //CT给25505充电,输出0，打开mos导通
#define CT_to_BQ25505_OFF digitalHi(ZAIXIN_IN2505_Ctrl_GPIO_Port,ZAIXIN_IN2505_Ctrl_Pin)  //CT给25505不充电，默认

#define CT_Source_short_ON   	digitalHi(Source_Ctrl_GPIO_Port,Source_Ctrl_Pin) //CT取电，短路，放电
#define CT_Source_short_OFF   digitalLo(Source_Ctrl_GPIO_Port,Source_Ctrl_Pin) //CT取电，开路，给BQ25505,默认

void DisableGPIO(void);
