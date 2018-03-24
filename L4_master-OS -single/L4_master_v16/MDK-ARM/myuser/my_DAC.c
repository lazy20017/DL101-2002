#include "my_dac.h"
#include "dac.h"
#include "my_ADC.h"
#include "my_wave_rec.h"
#include "my_gloabal_val.h"
#include "my_extern_val.h"








/*
功能：设置DA电流中断的参考电压,电场中断参考电压
*/

extern uint16_t my_dac1_channel1_data_i; //正常时为高电平，短路为负跳变，产生一个下降沿
extern uint16_t my_dac1_channel2_data_e;  //电场跳变对应的一个门限值，产生的是一个上升沿，正常时，输出0电平，当电场跌落小于这个值时，产生上上升沿

extern uint16_t my_all_i_up_value;  //测得的全波抬升1.2v电压对应的采样平均值

extern double my_i_ratio_value;
extern double my_E_ratio_value;
extern double MY_VDD;
extern double my_all_a_adjust; //实验数据，301/387获得的，在300A电流时刻，实际值，与ADC测量值的比较值
extern double my_adjust_300_a; //y=x*a+b,最小二乘法，的系数a，b，x为ADC测量值经过校正后的值，利用二乘法进行二次校正
extern double my_adjust_300_b;

extern double my_adjust_50_a;
extern double my_adjust_50_b;

extern double my_adjust_5_a;
extern double my_adjust_5_b;

extern double my_I_100A_Radio;

extern double my_i_5a_radio;//0.857517365; 
extern double my_i_50a_radio;//0.770811922;
extern double my_i_300a_radio;//0.770811922;


//extern int16_t my_dianliu_exit_add;
extern uint16_t my_wave_write_add;
//extern uint16_t my_Time_Cyc_exit_add;
extern double ADC2_Filer_value_buf_2[][3];

uint16_t my_dac1_channel2_data_e=0;
uint16_t my_dac1_channel1_data_i=0;

#define MY_Rise_current 150  //阶跃短路电流的默认值
int my_150A_ref_int = MY_Rise_current; //短路进入中断的阶跃电流值，电流增大超过这个值，就产生中断，下降沿,此值可以修改，DAC使用，它不是短路判断依据
//uint16_t my_150A_real_int = 0; //校正后150A阶跃电压对应的DAC，输入值
int my_5A_ref_int=5; //接地漏电流产生的阶跃量，超过产生中断

uint16_t my_100E_ref_int = 100;  //电场跌落的差值，只要跌落大于这个差值，就产生中断。上升沿
uint16_t my_100E_real_int = 0; //校正后跌落电压对应的DAC，输入值




/*
功能：DAC设置函数，不断调整DAC的参考电压。原理，利用测得的线上电流有效值，加上阶跃电流有效值，产生出中断电流有效值。把此值加上抬升的1.2v对应的值，
产生DAC的输出
*/

void my_fun_Set_DAC_I_ref(void)
{
    double temp_i = 0;
		volatile	double temp_e = 0;
    uint16_t my_temp_16=0;
    uint16_t my_12v_int=0;
	
		//GPIO_PinState my_pin_status=HAL_GPIO_ReadPin(EXIT_jiedi_GPIO_Port,EXIT_jiedi_Pin);
	  //printf("====EXIT_jiedi_pin=%d Vref=%.2f\n",my_pin_status,HAL_DAC_GetValue(&hdac1,DAC_CHANNEL_2)/4096.0*3.3);
	  if(my_ADC_Count==my_ADC_Count_old)
		{
			return;
		}
		my_ADC_Count_old=my_ADC_Count;

    temp_i = ADC2_Filer_value_buf_2[0][1]; //获得12周波电流的有效值,转换后的值,2为最大值，1为有效值
    temp_e=temp_i;
#if Debug_Usart_out_DAC_normal_data==1
    printf("DAC_line_Ai=%.2f M_Ai=%.2f,HI=%.2f,M_HI=%.2f,e_aver=%.2f\n",ADC2_Filer_value_buf_2[0][1],ADC2_Filer_value_buf_2[0][2],ADC2_Filer_value_buf_2[2][1],ADC2_Filer_value_buf_2[2][2],ADC2_Filer_value_buf_2[1][0]);
#endif

    //计算电流 DA 值
    temp_i=(temp_i+my_150A_ref_int); //150A的短路阶跃电流值
    //if(temp_i>600)
    //temp_i=600;
    //else if(temp_i<150)
    //temp_i=150;

    temp_i=temp_i*1.414;

    if(temp_i>165)
        my_temp_16=(temp_i-my_adjust_300_b)/(my_adjust_300_a*my_all_a_adjust)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;
		else if(temp_i>100 && temp_i<=165)
			my_temp_16=(temp_i)/(my_all_a_adjust)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;
		else if(temp_i>10 && temp_i<=100)
        my_temp_16=(temp_i-my_adjust_50_b)/(my_adjust_50_a*my_all_a_adjust)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;
    else if(temp_i<=10)
        my_temp_16=(temp_i-my_adjust_5_b)/(my_adjust_5_a*my_all_a_adjust)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;


    my_12v_int=(1.2)/3.3*4096;
    my_dac1_channel1_data_i =  my_temp_16 +  my_12v_int;//加上抬升的值

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, my_dac1_channel1_data_i); //设置数字量
#if Debug_Usart_out_DAC_normal_data==1
    //printf("\r\n",MY_VDD);
    printf("DAC_Rise_i=[%.2f]_A,OUT_V=[%.2f]_V,MY_VDD=%.2f\n",temp_i,my_dac1_channel1_data_i/4096.0*3.3,MY_VDD); //*=*=
#endif
	
		 //电场值
		 my_fun_DAC_evref_auto_ajust();
}

/*
功能：自动调整接地中断的DAC比较信号

*/
double my_DAC_Line_I=0; //在DAC设置期间获得的电流值
double my_DAC_Line_Efild=0; //同上，电场值
double my_adjust_value_V=0.00;
double my_counst_value=0.04;// 电场参考电压的恒定偏差，0.004对应1A电流，小于0.09V就频繁进中断，默认设置0.09
void my_fun_DAC_evref_auto_ajust(void)
{
		double temp_i = 0, temp_e = 0;
    uint16_t my_temp_16=0;
    uint16_t my_12v_int=0;
	
 GPIO_PinState my_pin_status=HAL_GPIO_ReadPin(EXIT_jiedi_GPIO_Port,EXIT_jiedi_Pin);
 
 printf("====EXIT_jiedi_pin=%d Vref=%.2f\n",my_pin_status,HAL_DAC_GetValue(&hdac1,DAC_CHANNEL_2)/4096.0*3.3); //默认输入1，有中断输入为0
	
	if(my_pin_status==0)
	{
		my_adjust_value_V=my_adjust_value_V+0.1; //自适应的电压门限值0.01。一次0.01V,my_adjust_value_V为基础.5倍为0.1增长，如果是1倍，0.01V增长
		
		my_DAC_Line_I=ADC2_Filer_value_buf_2[0][1];
		my_DAC_Line_Efild=ADC2_Filer_value_buf_2[1][1];
    temp_i = ADC2_Filer_value_buf_2[0][1]; //获得12周波电流的有效值,转换后的值,2为最大值，1为有效值
    temp_e=temp_i;  //线上的电流值

    //计算电场DA 值
    temp_e=temp_e*0.04; //线上电流值经过滤波后的残留电流值
    temp_e=0; //@@@@@
    //temp_e=(temp_e+my_5A_ref_int); //my_5A_ref_int接地瞬间电流值得阶跃量

    temp_e=temp_e*1.414; //有效值，变为最大值

//    if(temp_e>100)
//        my_temp_16=(temp_e-my_adjust_300_b)/(my_adjust_300_a*my_all_a_adjust)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;
//    else
//        my_temp_16=(temp_e-my_adjust_100_b)/(my_adjust_100_a*my_all_a_adjust)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;
		
		
		if(temp_e>100)
        my_temp_16=(temp_e-my_adjust_300_b)/(my_adjust_300_a*my_i_300a_radio)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096; //电压值
    else if(temp_e>10 && temp_e<=100)
			  my_temp_16=(temp_e-my_adjust_50_b)/(my_adjust_50_a*my_i_50a_radio)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;
		else if(temp_e<=10)
        my_temp_16=(temp_e-my_adjust_5_b)/(my_adjust_5_a*my_i_5a_radio)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;

    //my_dac1_channel2_data_e=my_temp_16;
		my_12v_int=(1.20+my_adjust_value_V)/3.3*4096;//报警电压 阶跃门限，1.2V基础上调整
    my_dac1_channel2_data_e =  my_temp_16 +  my_12v_int;//报警电压，加上抬升的值
    //my_dac1_channel2_data_e=62;//@@@

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, my_dac1_channel2_data_e); //设置数字量
#if Debug_Usart_out_DAC_normal_data==1
    printf("DAC2_Rise_i=[%.2f]_A,OUT_V=[%.2f]_V,my_adjust_value_V=%.2f ===going\n",temp_e,my_dac1_channel2_data_e/4096.0*3.3,my_adjust_value_V); //*=*=
    //printf("DAC2_Rise_i=[%.2f]_A,OUT_V=[%.2f]_V \n",temp_e,(my_temp_16*1.0/4096*MY_VDD)); //*=*=
#endif

	}
	else if(my_pin_status==1)
	{
		my_DAC_Line_I=ADC2_Filer_value_buf_2[0][1];
		my_DAC_Line_Efild=ADC2_Filer_value_buf_2[1][1];
		temp_i = ADC2_Filer_value_buf_2[0][1]; //获得12周波电流的有效值,转换后的值,2为最大值，1为有效值
    temp_e=temp_i;

    //计算电场DA 值
    temp_e=temp_e*0.04;
    temp_e=0;
    //temp_e=(temp_e+my_5A_ref_int);

    temp_e=temp_e*1.414; //最大值

    if(temp_e>165)
        my_temp_16=(temp_e-my_adjust_300_b)/(my_adjust_300_a*my_i_300a_radio)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;
		else if(temp_e>100 && temp_e<=165)
			  my_temp_16=(temp_e)/(my_i_300a_radio)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;
		else if(temp_e>10 && temp_e<=100)
			  my_temp_16=(temp_e-my_adjust_50_b)/(my_adjust_50_a*my_i_50a_radio)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;
    else if(temp_e<=10)
        my_temp_16=(temp_e-my_adjust_5_b)/(my_adjust_5_a*my_i_5a_radio)/(my_i_ratio_value*my_I_100A_Radio)/MY_VDD*4096;

    //my_dac1_channel2_data_e=my_temp_16;
		my_12v_int=(1.20+my_adjust_value_V+my_counst_value)/3.3*4096; //找到报警临界门限后，添加一个恒定常值门限。
    my_dac1_channel2_data_e =  my_temp_16 +  my_12v_int;//加上抬升的值
    //my_dac1_channel2_data_e=62;//@@@

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, my_dac1_channel2_data_e); //设置数字量
#if Debug_Usart_out_DAC_normal_data==1
    printf("DAC2_Rise_i=[%.2f]_A,OUT_V=[%.2f]_V,my_adjust_value_V=%.2f ===end\n",temp_e,my_dac1_channel2_data_e/4096.0*3.3,my_adjust_value_V); //*=*=
    //printf("DAC2_Rise_i=[%.2f]_A,OUT_V=[%.2f]_V \n",temp_e,(my_temp_16*1.0/4096*MY_VDD)); //*=*=
#endif
		
		
		//my_DAC_cyc_time=17;
		my_DAC_cyc_time=17;
		
		
	}
	

	
}








