/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)			{p->BSRR=i;}			//设置为高电平		
#define digitalLo(p,i)			{p->BRR=i;}				//输出低电平
#define digitalToggle(p,i)		{p->ODR ^=i;}			//输出反转状态

//#define LED3_TOGGLE						digitalToggle(GPIOA,GPIO_PIN_5)
//#define LED3_OFF							digitalLo(GPIOA,GPIO_PIN_5)
//#define LED3_ON								digitalHi(GPIOA,GPIO_PIN_5)

#define SPI3_CS_ENALBLE       digitalLo(GPIOC,GPIO_PIN_9)
#define SPI3_CS_DISABLE       digitalHi(GPIOC,GPIO_PIN_9)

#define WDI2_TOGGLE						digitalToggle(GPIOC,GPIO_PIN_5)
#define WDI2_OFF							digitalLo(GPIOC,GPIO_PIN_5)
#define WDI2_ON								digitalHi(GPIOC,GPIO_PIN_5)
