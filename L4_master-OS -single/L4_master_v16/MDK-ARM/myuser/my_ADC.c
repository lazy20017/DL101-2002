#include "my_ADC.h"
#include "my_gloabal_val.h"

extern uint16_t ADC1_GetValue[ADC1_ROW][ADC1_COLM]; //用于保存采集的值,M个通道,N次
extern uint16_t ADC1_GetValue_Aver[ADC1_COLM]; //用于保存平均值
extern double ADC1_Filer_value_buf[ADC1_COLM];  //用来存储ADC1采样到的的DC直流数据，共7个通道，真实值，转换后的



__IO uint16_t VREFINT_CAL;
__IO uint16_t MY_TS_CAL1;
__IO uint16_t MY_TS_CAL2;

/*
功能：计算ADC1的6个通道DMA采样值的平均值
*/
void filter_1(void)  //数据计算，进行求平均值,对ADC1中的4路直流电压数据进行计算平均值
{
    uint32_t  sum = 0;
    uint16_t	 i=0;
    uint8_t  count=0;
    for(i=0; i<ADC1_COLM; i++)

    {

        for (count=0; count<ADC1_ROW; count++)

        {

            sum += ADC1_GetValue[count][i];

        }

        ADC1_GetValue_Aver[i]=sum/ADC1_ROW;

        sum=0;
    }

}

/*
功能：/把ADC1的6个通道采样平均值，转换成物理值，存到数组中
*/
double MY_VDD=0.0;
void my_adc_1_convert(void)
{
    double x=0;
    filter_1(); //先算出3个采样值的平均值，只有1个数据

    //把平均值转换成对应的物理值
    MY_VDD=3.0*VREFINT_CAL/ADC1_GetValue_Aver[2];

    //ADC1_Filer_value_buf[0]=(ADC1_GetValue_Aver[0]-760)/2500.0 + 25;  //温度

    x=ADC1_GetValue_Aver[0]*MY_VDD/4096;
    ADC1_Filer_value_buf[0]=(x-0.76)*1000/2.5+30;


    //ADC1_Filer_value_buf[0]=(110.0-30.0)/(MY_TS_CAL2-MY_TS_CAL1)*(int16_t)(ADC1_GetValue_Aver[0]-MY_TS_CAL1)+30;


    ADC1_Filer_value_buf[1]=ADC1_GetValue_Aver[1]*MY_VDD/4096*3;//供电电压
    ADC1_Filer_value_buf[2]=MY_VDD; //参考电压
    ADC1_Filer_value_buf[3]=ADC1_GetValue_Aver[3]*MY_VDD/4096*(2); //干电池
    ADC1_Filer_value_buf[4]=ADC1_GetValue_Aver[4]*MY_VDD/4096*(2); //线上电压
    ADC1_Filer_value_buf[5]=ADC1_GetValue_Aver[5]*MY_VDD/4096*(2); //太阳能
    ADC1_Filer_value_buf[6]=ADC1_GetValue_Aver[6]*MY_VDD/4096*(2); //锂电池
		ADC1_Filer_value_buf[7]=ADC1_GetValue_Aver[7]*MY_VDD/4096; //1.2V

}



/*
功能：把ADC1对应的6个通道的物理值串口输出
*/
extern int my_CC1101_RSSI;
void my_adc_1_convert_dis(uint8_t convert_status)
{

    if(convert_status==1)  //如果为1，则进行ADC采样并转换
        my_adc_1_convert();
    //=============USART2==debug
    printf(" DC:GANbat=%.2f, Zaixian=% .2f, sunbat=%.2f, Libat=%.2f, V1V2=%.2f\n",ADC1_Filer_value_buf[3],ADC1_Filer_value_buf[4],ADC1_Filer_value_buf[5],ADC1_Filer_value_buf[6],ADC1_Filer_value_buf[7]);
    printf(" DC:Temp=%.2f,vbat=%.2f,vref=%.2f, RSSI=%d\n",ADC1_Filer_value_buf[0],ADC1_Filer_value_buf[1],ADC1_Filer_value_buf[2],my_CC1101_RSSI);  
	
    printf("\n");
}








