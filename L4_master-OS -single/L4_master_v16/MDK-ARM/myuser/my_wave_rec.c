#include "gpio.h"
#include "my_wave_rec.h"
#include "my_ADC.h"
#include "my_usart.h"
#include "math.h"
#include "my_gloabal_val.h"
#include "cmsis_os.h"
#include "my_extern_val.h"

//======================

//====================
//�о���
volatile uint8_t my_Fault_Current_End_Status = 0x00; //
volatile uint8_t my_Fault_E_Fild_End_Status = 0x00; //

uint8_t my_befor_500ms_normal_count = 0; //�ж�ǰ500msͣ��״̬��¼
uint8_t my_befor_short_stop_count = 0; //�ж�ǰ��״̬��ֻ����һ�βɼ�������3���������
uint8_t my_short_circuit_count = 0; //��·״̬������������Ծ������,3��������
uint8_t my_short_circuit_count2 = 0; //��·״̬������������Ծ��������2��������

uint8_t my_after_short_stop_count1 = 0; //�жϺ󣬵�һ��ͣ��״̬��¼
uint8_t my_after_short_stop_count2 = 0; //�жϺ󣬵�һ��ͣ������磬�ڶ���ͣ��Ĵ�����¼

uint8_t my_after_stop1_normal_count = 0; //��һ��ͣ�������״̬��¼

uint8_t my_E_fild_change_count = 0; //�ӵ�״̬���糡������Ծ���½�
uint8_t my_E_fild_min_count = 0;   //�ӵأ��糡��Сֵ


uint8_t my_Line_Current_stop_status = 0; //ͣ��״̬��ʶ��1Ϊͣ�磬2Ϊ�ӵ�,3Ϊ����,
uint8_t my_Line_Current_stop_last_status = 0xff; //��һ�ε�״̬��������������������µ�״̬�Ƚϣ�ͣ���˾��ϴ�,3Ϊ����,1Ϊͣ�磬2Ϊ�ӵ�
uint16_t my_Line_Efild_valu = 0; //�Եص糡��ֵ
uint16_t my_line_Current_value = 0; //���ϵ�������Чֵ���ŵ�10��ȡ����


uint16_t MY_Efile_Zero_data = 10; //�糡С�ڴ�ֵ����ʾΪ0
uint16_t MY_Efile_floor_data = 60; //�糡���ޣ�С�ڴ�ֵ����ʾ�ӵأ�Ĭ��40
float my_HA_Zero_data = 2; //�벨�����ж���Сֵ��С�����ֵ����Ϊͣ���ˡ�0.2
double my_A_Zero_data = 0; //����0ֵ����






#define E_cell 1.0        //�糡��Ӧ��У��ֵ,����ADC�����õ������ݽ��еȱȱ仯
uint16_t Current_D_value = 100; //��������ͻ�䷧ֵ,�ж����������ڴ�ֵ��ʾ��������·����,���ֵ���ж϶�·������ADCʹ�ã�Ĭ��150
uint16_t E_fild_D_value = 80; //�糡�µ�ͻ�䷧ֵ
uint16_t E_fild_threshold_value = 20; //�糡��Сֵ��С�����ֵ����Ϊ�ӵ���
//========
double my_all_a_adjust = 0.76560196; //�벨1.42655781;//ȫ��0.798980075;    //1.42655781; // 0.798980075=301/368          301/387=0.7838541670; //ʵ�����ݣ�301/387��õģ���300A����ʱ�̣�ʵ��ֵ����ADC����ֵ�ıȽ�ֵ
//295��395.7815=0.745360761
double my_adjust_300_a = 1.054171799;//1.009700101;//1.007328; //y=x*a+b,��С���˷�����ϵ��a��b��xΪADC����ֵ����У�����ֵ�����ö��˷����ж���У��
double my_adjust_300_b = -12.56341994;//-1.587214894;//-2.42403;

double my_adjust_50_a = 0.986329196; //0.994672291; //1.00022;
double my_adjust_50_b = 0.686363636; //-0.144475163;// -2.19103;

double my_adjust_5_a = 1.059577357; //0.994672291; //1.00022;
double my_adjust_5_b = -0.272350023; //-0.144475163;// -2.19103;

double my_I_100A_Radio = 1.0; //��׼���ϵ���Ϊ100Aʱ��У��ϵ�����㷨��Ĭ��ֵΪ1��100A/������ݵ�ֵ
double my_i_ratio_value = 1000; //1/2.0*1500; //�������ϵ��//��������Ϊ2ŷķ�����Ϊ1500:1,�����õ�1000
double my_value_daya = 2.0; //�ӷ�У��ϵ����Ĭ��2.0

//+++++++++++++++++++++++
//double my_i_5a_radio = 5.02 / 7.61; //5.08/6.26;//0.662251656;//0.857517365;   //5.08--6.26  303---393
//double my_i_50a_radio = 50.9 / 71.06;
//double my_i_300a_radio = 306.1 / 412.6; //293.4/377.354;//0.748091603;//0.770811922;

//double my_10A_gatedata = 15.14;
//double my_100A_gatedata = 139.92;

extern double my_i_5a_radio ; //5.08/6.26;//0.662251656;//0.857517365;   //5.08--6.26  303---393
extern double my_i_50a_radio ;
extern double my_i_300a_radio; //293.4/377.354;//0.748091603;//0.770811922;

extern double my_10A_gatedata ;
extern double my_100A_gatedata ;

//+++++++++++++++++++++++++=
//�ڳ���һ��100Aʱ�õ���У��ϵ����

double my_E_ratio_value = 1000; //�糡���ϵ��

//��12���ڣ�ÿ�����ڲ���ֵ
uint16_t WAVE_half_ave_Current1[12][3] = {0}; //�������д棬ƽ������Ч�����12�����ڵ�ÿ�����ڵ�ֵ
uint16_t WAVE_all_ave_Current1[12][3] = {0}; //�������д棬ƽ������Ч�����
uint16_t WAVE_all_ave_E_Field1[12][3] = {0};

//��ת����12���ڵĽ������������������ʵ����ֵ
double WAVE_half_ave_Current2[12][3] = {0}; //�������д棬ƽ������Ч�����12�����ڵ�ÿ�����ڵ�ֵ
double WAVE_all_ave_Current2[12][3] = {0}; //�������д棬ƽ������Ч�����
double WAVE_all_ave_E_Field2[12][3] = {0}; //�������д棬ƽ������Ч�����

//3�����壬������ʵ����ֵ
double WAVE_half_ave_Current3[12][3] = {0}; //�������д棬ƽ������Ч�����12�����ڵ�ÿ�����ڵ�ֵ
double WAVE_all_ave_Current3[12][3] = {0}; //�������д棬ƽ������Ч�����
double WAVE_all_ave_E_Field3[12][3] = {0}; //�������д棬ƽ������Ч�����

//================
extern double MY_VDD;
extern uint16_t ADC2_GetValue[ADC2_ROW][ADC2_COLM];//���ڱ���ɼ���ֵ,M��ͨ��,N��,¼��
extern double ADC2_Filer_value_buf_2[ADC2_COLM][3];
extern double ADC2_Filer_value_buf_3[ADC2_COLM][3];
extern uint16_t ADC1_GetValue[ADC1_ROW][ADC1_COLM];//���ڱ���ɼ���ֵ,M��ͨ��,N�Σ�ֱ���ź�
extern uint8_t  my_Current_exit_Status; //��ʾ���������ж�
extern uint16_t my_dianliu_exit_add;
extern uint16_t my_wave_record[WAVE_Channel_Num][WAVE_number]; //ȫ����һ�����棬���ö�ʱ�����в��εĴ洢���ٶ�20ms�ɼ�һ�Σ����ܲɼ������ٸ����ݡ�
extern uint16_t my_wave_write_add;  //ȫ��¼��һ������ָ��
extern uint16_t my_wave_record_sec2[WAVE_Channel_Num][WAVE_number_sec2]; //ȫ���Ķ�������
extern uint16_t my_wave_record_sec3[WAVE_Channel_Num][WAVE_number_sec3]; //ȫ���Ķ�������
extern uint8_t  my_wave_re_status;  //¼��״̬��0��ʾ�����ݣ�1��2,3,4,5��ʾ�������������ݣ�10��ʾ��������
extern uint8_t my_IT_Count;
extern int8_t my_IT_status;
extern uint8_t  my_E_Field_exit_Status;
extern uint16_t my_Wave_It_add;
//extern uint16_t my_Current_Exit_add;
extern uint16_t my_E_Field_exit_add;
extern uint16_t ADC2_Filer_value_buf_1[][3];


extern osMessageQId myQueue01Handle;

extern double my_DAC_Line_I; //��DAC�����ڼ��õĵ���ֵ
extern double my_DAC_Line_Efild; //ͬ�ϣ��糡ֵ

uint16_t my_all_i_up_value = 0; //��õ�ȫ��̧��1.2v��ѹ��Ӧ�Ĳ���ƽ��ֵ��ADC������ֵ��δ����ת��


#define rec_T_Count 12  //��¼���ڵ�����
uint16_t my_E_fild_time_add = 0; //�������ڲ�ѯ��ʱ��my_E_fild_time_add�洢��ѯ�ĵ�ַ

extern uint8_t  my_Time_Cyc_exit_Status;



//====
/*
���ܣ��˺���ֻ����ʾ��1�����ڵ�ƽ��ֵ�Ļ������Чֵ����ʾһ��������0�㿪ʼ��320������
����ʹ�ã����岻��
������xxΪ1��ʱ�򣬱�ʾ����ȡ1�����ڵķ���������2��ʾ����ȡ12�����ڵķ���
*/
//1��ʾ�������ݵ�2������
void my_adc2_convert_dis(uint8_t convet_status)
{

    if(convet_status == 1) //Ϊ1�������1��¼�����ݣ��浽2�����棬Ȼ�������������е�����
        my_adc2_convert2(1);

    //��Чֵ=1.1*ƽ��ֵ�����ֵ=ƽ��ֵ/0.637
    printf("1All_A:AVR=%.2f, RMS=%.2f, MAX=%.2f, \n", ADC2_Filer_value_buf_2[0][0], ADC2_Filer_value_buf_2[0][1], ADC2_Filer_value_buf_2[0][2]);
    printf("1ALL_E:AVR=%.2f, RMS=%.2f, MAX=%.2f, \n", ADC2_Filer_value_buf_2[1][0], ADC2_Filer_value_buf_2[1][1], ADC2_Filer_value_buf_2[1][2]);
    printf("1Hal_A:AVR=%.2f, RMS=%.2f, MAX=%.2f, \n", ADC2_Filer_value_buf_2[2][0], ADC2_Filer_value_buf_2[2][1], ADC2_Filer_value_buf_2[2][2]);

    //��������¼���������������¼�����ݷ���
//    printf("\n==========\n");
//    for(xj=0; xj<960; xj++) //���ڷ���¼���ĵ����ݣ���0��ַ��ʼ
//    {

//        //printf("%d\n",my_wave_record_sec2[0][xj]); //������ȫ��
//        //printf("%d\n",my_wave_record_sec2[1][xj]); //�糡��ȫ��
//        //printf("%d\n",my_wave_record_sec2[2][xj]); //�����벨
//    }
//    printf("\n==========\n");


    //����ת����Ľ����¼�����ݷ���
#if Debug_Usart_out_wavedata_960Data_2cach ==1  //ȫ������
    int xj = 0;
    double temp = 0; //�з�������
    printf("\n**************\n");
    for(xj = 0; xj < WAVE_number_sec2; xj++)
    {
        temp = (my_wave_record_sec2[0][xj] - my_all_i_up_value);
        temp = temp * MY_VDD / 4096 * (my_i_ratio_value * my_I_100A_Radio); //����С�����1λ
        printf("%.1f\n", temp);
    }
    printf("\n**************\n");
#elif Debug_Usart_out_wavedata_960Data_2cach ==2  //ȫ���糡
    int xj = 0;
    double temp = 0; //�з�������
    printf("\n**************\n");
    for(xj = 0; xj < WAVE_number_sec2; xj++)
    {
        temp = (my_wave_record_sec2[1][xj]);
        temp = temp * MY_VDD / 4096 * my_E_ratio_value; //����С�����1λ
        printf("%.1f\n", temp);
    }
    printf("\n**************\n");
#elif Debug_Usart_out_wavedata_960Data_2cach ==3 //�벨����
    int xj = 0;
    double temp = 0; //�з�������
    printf("\n**************\n");
    for(xj = 0; xj < WAVE_number_sec2; xj++)
    {
        temp = (my_wave_record_sec2[2][xj] - my_all_i_up_value);
        temp = temp * MY_VDD / 4096 * (my_i_ratio_value * my_I_100A_Radio); //����С�����1λ
        printf("%.1f\n", temp);
    }
    printf("\n**************\n");
#endif


}





//�˲���Ϊ�Զ��庯��
/*
���ܣ����ð벨�����������Чֵ 12�����ڣ���ÿ������AVR RMS MAX
*/




void fun_real_half_Current(void)
{
    uint32_t sum = 0, sum_pwr = 0, count = 0;
    int xi = 0, xj = 0, max = 0;
    for(xi = 0; xi < 12; xi++)
    {   sum = 0;
        max = 0;
        count = 0;
        sum_pwr = 0;
        //Ϊ����80�����ƽ��ֵ
        for(xj = 0; xj < 80; xj++)
        {
            if(my_wave_record_sec2[2][xi * 80 + xj] > 0)
            {
                count++;
                sum = sum + my_wave_record_sec2[2][xi * 80 + xj]; //Ϊ����80����ĺ�
                sum_pwr = sum_pwr + my_wave_record_sec2[2][xi * 80 + xj] * my_wave_record_sec2[2][xi * 80 + xj]; //����80�����ƽ����
                if(my_wave_record_sec2[2][xi * 80 + xj] > max)
                    max = my_wave_record_sec2[2][xi * 80 + xj];
            }

            //@@@@@@����ʹ��
            //printf("\n %d",my_wave_record_sec2[2][xi * 80 + xj]);

        }
        //2017-04-18

        //��������ֵ
        WAVE_half_ave_Current1[xi][0] = sum*1.0 / count;
        WAVE_half_ave_Current1[xi][1] = sqrt(sum_pwr * 1.0 / count);
        WAVE_half_ave_Current1[xi][2] = max;
        //printf("\n @@max=%d",max);

        //ת��ֵ
        WAVE_half_ave_Current2[xi][0] = sum * MY_VDD / 4096 / count; //ƽ��ֵ
        WAVE_half_ave_Current2[xi][1] = sqrt(sum_pwr * 1.0 / count) * MY_VDD / 4096; //���þ�������������Чֵ
        WAVE_half_ave_Current2[xi][2] = max * MY_VDD / 4096;
#if USE_half_adjust_zero==1
        if(WAVE_half_ave_Current1[xi][0] == WAVE_half_ave_Current1[xi][2] && WAVE_half_ave_Current1[xi][2] != 0)
        {
            //��������ֵ
            WAVE_half_ave_Current1[xi][0] = 0;
            WAVE_half_ave_Current1[xi][1] = 0;
            WAVE_half_ave_Current1[xi][2] = 0;
            //printf("\n @@max=%d",max);
            //ת��ֵ
            WAVE_half_ave_Current2[xi][0] = 0; //ƽ��ֵ
            WAVE_half_ave_Current2[xi][1] = 0; //���þ�������������Чֵ
            WAVE_half_ave_Current2[xi][2] = 0;
        }
#endif
        //����ʹ��
        //printf("half no zero count=%d\n",count);
    }

}
/*
���ܣ��������ö�������ȫ��������12�����ڵĵ�����  ÿ������ƽ��ֵ����Чֵ�����ֵ
*/

//
double temp_double = 0;
void fun_real_all_Current(void)
{
    uint32_t sum = 0, sum_pwr = 0, avr = 0, sum_avr_up = 0;

    int temp = 0;
    int xi = 0, xj = 0, max = 0;
    for(xi = 0; xi < 12; xi++) //12������
    {   sum = 0;
        max = 0;
        sum_pwr = 0;

        for(xj = 0; xj < 80; xj++) //Ϊ����80�����ƽ��ֵ
        {
            sum = sum + my_wave_record_sec2[0][xi * 80 + xj]; //Ϊ����80����ĺ�

        }
        //
        avr = sum / 80;
        sum_avr_up = sum_avr_up + avr;
        sum_pwr = 0;
        sum = 0;
        max = 0;

        //2017-04-16 ɾ��̧����ѹ1.2v
        for(xj = 0; xj < 80; xj++)
        {

            temp = avr - my_wave_record_sec2[0][xi * 80 + xj];
            //temp=my_wave_record_sec2[0][xi*80+xj];
            if(temp < 0)
                temp = -temp;
            //��Чֵ
            sum_pwr = sum_pwr + temp * temp;

            //ƽ��ֵ
            sum = sum + temp;
            //���ֵ
            if(temp > max)
                max = temp;

        }
        //����ֵ
        WAVE_all_ave_Current1[xi][0] = sum / 80.0; //ƽ��ֵ
        WAVE_all_ave_Current1[xi][1] = sqrt(sum_pwr * 1.0 / 80.0); //���þ�������������Чֵ,�Ƚ���������С���˷����
        WAVE_all_ave_Current1[xi][2] = max;



        //ת�����ֵ
        //ƽ��ֵ
        WAVE_all_ave_Current2[xi][0] = sum * MY_VDD / 4096 / 80 * (my_i_ratio_value * my_I_100A_Radio);

        //��Чֵ
        temp_double = sqrt(sum_pwr * 1.0 / 80) * MY_VDD / 4096 * (my_i_ratio_value * my_I_100A_Radio);


#if USE_Adjust_suanfa==1

//				if(temp_double > 100) //��С���˷����
//            WAVE_all_ave_Current2[xi][1] = temp_double * my_all_a_adjust * my_adjust_300_a + my_adjust_300_b; //���þ�������������Чֵ,�Ƚ���������С���˷����
//        else
//            WAVE_all_ave_Current2[xi][1] = temp_double * my_all_a_adjust * my_adjust_100_a + my_adjust_100_b;
//        //WAVE_all_ave_Current2[xi][1] = temp_double * my_all_a_adjust * my_adjust_300_a + my_adjust_300_b;
//        //��Чֵ��ֵУ��
//        if(WAVE_all_ave_Current2[xi][1] <50)
//            WAVE_all_ave_Current2[xi][1] =WAVE_all_ave_Current2[xi][1] +my_value_daya; //����У��ϵͳ

#elif USE_Adjust_suanfa==2
        if(temp_double > my_100A_gatedata) //ϵ��У����
            if(temp_double > 138 && temp_double < 230)
                WAVE_all_ave_Current2[xi][1] = temp_double * my_i_300a_radio ;
            else
                WAVE_all_ave_Current2[xi][1] = temp_double * my_i_300a_radio * my_adjust_300_a + my_adjust_300_b;
        else if(temp_double > my_10A_gatedata && temp_double <= my_100A_gatedata )
            WAVE_all_ave_Current2[xi][1] = temp_double * my_i_50a_radio * my_adjust_50_a + my_adjust_50_b;
        else
            WAVE_all_ave_Current2[xi][1] = temp_double * my_i_5a_radio * my_adjust_5_a + my_adjust_5_b;

#else

        WAVE_all_ave_Current2[xi][1] = temp_double;
#endif


        //���ֵ
        WAVE_all_ave_Current2[xi][2] = max * MY_VDD / 4096 * (my_i_ratio_value * my_I_100A_Radio);
    }

    my_all_i_up_value = sum_avr_up / 12; //ADC������ֵ��δ����ת��


}
//���ö�������ȫ��������12�����ڣ�ÿ�����ڵ����ۣ���Чֵ�����ֵ

void fun_real_all_dianchang(void)
{
    uint32_t sum = 0, sum_pwr = 0, count = 0;
    int xi = 0, xj = 0, max = 0;
    for(xi = 0; xi < 12; xi++)
    {   sum = 0;
        max = 0;
        count = 0;
        sum_pwr = 0;
        for(xj = 0; xj < 80; xj++)
        {
            if(my_wave_record_sec2[1][xi * 80 + xj] > 0)
            {
                count++;
                sum = sum + my_wave_record_sec2[1][xi * 80 + xj];
                sum_pwr = sum_pwr + my_wave_record_sec2[1][xi * 80 + xj] * my_wave_record_sec2[1][xi * 80 + xj];
                if(my_wave_record_sec2[1][xi * 80 + xj] > max)
                    max = my_wave_record_sec2[1][xi * 80 + xj];

            }

        }
        //����ֵ
				count=80;
        WAVE_all_ave_E_Field1[xi][0] = sum*1.0 / count;
        WAVE_all_ave_E_Field1[xi][1] = sum *1.0/ count;//sqrt(sum_pwr * 1.0 / 80); //��������
        WAVE_all_ave_E_Field1[xi][2] = max;

        //ת��ֵ
        WAVE_all_ave_E_Field2[xi][0] = sum * MY_VDD / 4096 / count * my_E_ratio_value;

        //WAVE_all_ave_E_Field2[xi][1] = sqrt(sum_pwr * 1.0 / count) * MY_VDD / 4096 * my_E_ratio_value; //�糡����������
        WAVE_all_ave_E_Field2[xi][1] = WAVE_all_ave_E_Field2[xi][0] ; //�糡��ƽ��ֵ����
        WAVE_all_ave_E_Field2[xi][2] = max * MY_VDD / 4096 * my_E_ratio_value;
				//��Ʈ����
				if( WAVE_all_ave_E_Field1[xi][0]==WAVE_all_ave_E_Field1[xi][2] && WAVE_all_ave_E_Field1[xi][0]!=0)
				{
					//����ֵ

        WAVE_all_ave_E_Field1[xi][0] = 0;
        WAVE_all_ave_E_Field1[xi][1] = 0;
        WAVE_all_ave_E_Field1[xi][2] = 0;

        //ת��ֵ
        WAVE_all_ave_E_Field2[xi][0] = 0;     
        WAVE_all_ave_E_Field2[xi][1] = 0 ; //�糡��ƽ��ֵ����
        WAVE_all_ave_E_Field2[xi][2] = 0;
					
				}
    }

}

/*
���ܣ�¼�����ݣ���һ��������ȡ12�����ڵ����ݣ��������������
����ֵ�����ȡ����12�����ڵ�¼�����ݣ��ͷ���1,���û��ȡ�þͷ���0


*/
uint16_t  number_before_pp = 320; //¼�����ݣ��ж�ʱ��ǰ�ĵ�����320���㣬4�����ڣ�����4*80=320

int fun_my_wave1_to_wave2(void)
{
    int xi = 0, xj = 0;
    volatile	int xk = 0;
    //int ii=0,temp=0;
    uint16_t my_wav2_add = 0;
    uint8_t  my_re = 0;
    int my_stop_add1 = 0;
    int my_stop_add2 = 0;


    //��1�������¼�����ݰ�����д��ַǰ��������
    if((my_wave_write_add > my_dianliu_exit_add) && (my_wave_write_add - my_dianliu_exit_add) >= (WAVE_number_sec2 - number_before_pp))
    {   my_wave_re_status = 1; //��������ݵ���������
        my_wav2_add = my_dianliu_exit_add;
        if(my_wav2_add >= number_before_pp) //1.1ǰ320�������жϵ�ǰ
        {
            for(xi = 0; xi < WAVE_number_sec2; xi++)
            {
                my_wave_record_sec2[0][xi] = my_wave_record[0][my_wav2_add - number_before_pp + xi];
                my_wave_record_sec2[1][xi] = my_wave_record[1][my_wav2_add - number_before_pp + xi];
                my_wave_record_sec2[2][xi] = my_wave_record[2][my_wav2_add - number_before_pp + xi];
                //my_wav2_add++;
                xk = 1;
            }
        }
        else //1.2ǰ320�㣬��һ����û���жϵ�ǰ
        {
            for(xi = 0; xi < number_before_pp - my_wav2_add; xi++) //1.2.1ȡ�жϵ���320�������в��֣������ĩβ
            {
                my_wave_record_sec2[0][xi] = my_wave_record[0][WAVE_number - (number_before_pp - my_wav2_add) + xi];
                my_wave_record_sec2[1][xi] = my_wave_record[1][WAVE_number - (number_before_pp - my_wav2_add) + xi];
                my_wave_record_sec2[2][xi] = my_wave_record[2][WAVE_number - (number_before_pp - my_wav2_add) + xi];
                xk = 2;
            }
            for(xj = 0; xi < WAVE_number_sec2; xi++, xj++) //1.2.2ȡ�жϵ�ǰ��320�������еĲ��֣������ͷ
            {
                my_wave_record_sec2[0][xi] = my_wave_record[0][xj];
                my_wave_record_sec2[1][xi] = my_wave_record[1][xj];
                my_wave_record_sec2[2][xi] = my_wave_record[2][xj];
                xk = 3;
            }

        }
        //@@@������ʹ�ã�����¼�����ݵ�1�����ݺ����һ�����ڵĵ�һ�����ݵĲ�ֵ
        //temp=(my_wave_record_sec2[0][0]-my_wave_record_sec2[0][881]);



#if Debug_Usart_out_chazhidata==1
        if(xk == 1 || xk == 2 || xk == 3) //XK1��ʾ��¼��������д��ַǰ��3��ʾ¼��������д��ַǰ������320�����ݲ���������ĩβ
            printf("==XK%d-- error chazhi=%d\n", xk, my_wave_record_sec2[0][0] - my_wave_record_sec2[0][881]);
#endif

        //���ڷ���¼�����ݣ����������е����ݣ�û�о���������
//					if(temp>100 ||temp<=-100)
//				{   printf("====start================\n");
//					  for(ii=0;ii<960;ii++)
//					   	printf("%d\n",my_wave_record_sec2[0][ii]);
//					printf("====end=================\n");
//				}

        my_re = 1;


    }
    //2¼�����ݲ�����д��ַǰ������ĩβ���в�������
    else if((my_wave_write_add < my_dianliu_exit_add) && ((WAVE_number - my_dianliu_exit_add + my_wave_write_add) >= (WAVE_number_sec2 - number_before_pp)))
    {   my_wave_re_status = 1; //��������ݵ���������
        my_wav2_add = my_dianliu_exit_add;

        for(xi = 0; (xi < WAVE_number - my_dianliu_exit_add + number_before_pp) && (xi < WAVE_number_sec2); xi++)
        {
            my_wave_record_sec2[0][xi] = my_wave_record[0][my_dianliu_exit_add - number_before_pp + xi];
            my_wave_record_sec2[1][xi] = my_wave_record[1][my_dianliu_exit_add - number_before_pp + xi];
            my_wave_record_sec2[2][xi] = my_wave_record[2][my_dianliu_exit_add - number_before_pp + xi];
            xk = 4;
        }
        for(xj = 0; xi < WAVE_number_sec2; xi++, xj++)
        {
            my_wave_record_sec2[0][xi] = my_wave_record[0][xj];
            my_wave_record_sec2[1][xi] = my_wave_record[1][xj];
            my_wave_record_sec2[2][xi] = my_wave_record[2][xj];
            xk = 5;
        }
#if Debug_Usart_out_chazhidata==1
        if(xk == 4 || xk == 5) //5����¼�����ݲ���������ĩβ
            printf("==XK%d-- error chazhi=%d\n", xk, my_wave_record_sec2[0][0] - my_wave_record_sec2[0][881]);
#endif
        my_re = 1;

    }
    else
    {
        my_re = 0;
    }
    //����ǰ500ms����ǰ���Ƿ�ͣ��
    if(my_re == 1 && my_IT_status == 0)
    {
        my_befor_500ms_normal_count = 0;
        my_stop_add1 = my_dianliu_exit_add - 2400;
        my_stop_add2 = my_dianliu_exit_add - 2480;
        if(my_stop_add1 >= 0 && my_stop_add2 >= 0)
        {
            for(xi = my_stop_add2; xi <= my_stop_add1; xi++)
            {
                if(my_wave_record[2][xi] >my_HA_Zero_data)
                    my_befor_500ms_normal_count++;
            }
        }
        else if(my_stop_add1 >= 0 && my_stop_add2 < 0)
        {
            my_stop_add2 = WAVE_number + my_stop_add2;
            for(xi = my_stop_add2; xi < WAVE_number; xi++)
            {
                if(my_wave_record[2][xi] >my_HA_Zero_data)
                    my_befor_500ms_normal_count++;
            }
            for(xi = 0; xi <= my_stop_add1; xi++)
            {
                if(my_wave_record[2][xi] >my_HA_Zero_data)
                    my_befor_500ms_normal_count++;

            }

        }
        else if(my_stop_add1 < 0 && my_stop_add2 < 0)
        {
            my_stop_add2 = WAVE_number + my_stop_add2;
            my_stop_add1 = WAVE_number + my_stop_add1;
            for(xi = my_stop_add2; xi <= my_stop_add1; xi++)
            {
                if(my_wave_record[2][xi] >my_HA_Zero_data)
                    my_befor_500ms_normal_count++;
            }

        }
    }

//----����ʹ��
#if				CC1101_SEND_E_Simulation_data_status==1
    int ii = 0, my_temp = 0;
    int16_t xx = 0;
    for(ii = 0; ii < 960; ii++)
    {
        my_temp = ii % 80;
        xx = (int16_t)(sinf((float)((my_temp / 79.0 * 2 * 3.1415926) + 1)) / 2.0 * 4096);
        my_wave_record_sec2[1][ii] = xx;

    }


#endif


    return my_re;
}

/*
����ʹ�ã�
���ܣ��Ѷ��λ����е����ݣ�12�����ڼ����ƽ��ֵ����Чֵ�����ֵ��������ڣ�

*/
void funT_display_wave2(void)
{
    uint8_t xi = 0;
    printf("================\n");
    for(xi = 0; xi < 12; xi++)
        printf("half_A: AVR=%f,RMP=%f,MAX=%f\n", WAVE_half_ave_Current2[xi][0], WAVE_half_ave_Current2[xi][1], WAVE_half_ave_Current2[xi][2]);
    printf("-------------\n");
    for(xi = 0; xi < 12; xi++)
        printf("all__A: AVR=%f,RMP=%f,MAX=%f\n", WAVE_all_ave_Current2[xi][0], WAVE_all_ave_Current2[xi][1], WAVE_all_ave_Current2[xi][2]);
    printf("-------------\n");
    for(xi = 0; xi < 12; xi++)
        printf("all__E: AVR=%f,RMP=%f,MAX=%f\n", WAVE_all_ave_E_Field2[xi][0], WAVE_all_ave_E_Field2[xi][1], WAVE_all_ave_E_Field2[xi][2]);
    printf("================\n");

    //���͸�433ģ��
    for(xi = 0; xi < 12; xi++)
    {
        USART_printf1(&huart2, "half_A: AVR=%f, ", WAVE_half_ave_Current2[xi][0]);
        USART_printf1(&huart2, "RMP=%f, ", WAVE_half_ave_Current2[xi][1]);
        USART_printf1(&huart2, "MAX=%f \n", WAVE_half_ave_Current2[xi][2]);
    }

    for(xi = 0; xi < 12; xi++)
    {
        USART_printf1(&huart2, "all__A: AVR=%f, ", WAVE_all_ave_Current2[xi][0]);
        USART_printf1(&huart2, "RMP=%f, ", WAVE_all_ave_Current2[xi][1]);
        USART_printf1(&huart2, "MAX=%f \n", WAVE_all_ave_Current2[xi][2]);
    }

    for(xi = 0; xi < 12; xi++)
    {
        USART_printf1(&huart2, "all__E: AVR=%f, ", WAVE_all_ave_E_Field2[xi][0]);
        USART_printf1(&huart2, "RMP=%f, ", WAVE_all_ave_E_Field2[xi][1]);
        USART_printf1(&huart2, "MAX=%f \n", WAVE_all_ave_E_Field2[xi][2]);
    }



}



/*
���ܣ�
��ǰʱ�̣�1���������ݷ���12�����ڵ�2��������
���ö������������ݣ�12�����ڣ������ÿ������ƽ��ֵ����Чֵ�����ֵ,���뵽WAVE_all_ave_Current2������
����12��ÿ�����ڵļ�����ֵ������12�����ڵ�����ֵ��ƽ��ֵ����Чֵ�����ֵ�����ŵ�ADC2_Filer_value_buf_2������

�����WAVE_all_ave_Current2���飬12������ÿ�����ڵ�ֵ��3������ƽ��ֵ����Чֵ�����ֵ��
ADC2_Filer_value_buf_2���飬ÿ��ͨ��12�����ڵ�ƽ��ֵ������������������ģ�3������ƽ��ֵ����Чֵ�����ֵ��

����˵����1��ʾ���ڲ�����¼����1�����嵽2�����棬0��ʾ���ã��ⲿ�Ѿ�������2������
*/


void my_adc2_convert2(uint8_t my_status)
{

    double temp1 = 0, temp2 = 0, temp3 = 0,max3=0;
    uint16_t temp11 = 0, temp12 = 0, temp13 = 0,max13=0;
    int xi = 0;

    if(my_status == 1)
    {
        my_fun_wave_rec();
    }

    fun_real_half_Current();  //�������������12�����ڵģ�ÿ�����ڵ�3�����ݣ�����3*12*3=108
    fun_real_all_Current();
    fun_real_all_dianchang();

    //����12�����ڵ��ۼ�ƽ��ֵ
    //===ȫ������
    temp1 = temp2 = temp3 =max3= 0;
    temp11 = temp12 = temp13 =max13= 0;
    for(xi = 0; xi < 12; xi++) //12�����ڣ�ÿ�����ڵ�ֵ
    {
        //����ֵ
        temp11 = temp11 + WAVE_all_ave_Current1[xi][0];
        temp12 = temp12 + WAVE_all_ave_Current1[xi][1];
        temp13 = temp13 + WAVE_all_ave_Current1[xi][2];
				if(WAVE_all_ave_Current1[xi][1]>max13) //������Чֵ�����ֵ
					max13=WAVE_all_ave_Current1[xi][1];

        //ת��ֵ
        temp1 = temp1 + WAVE_all_ave_Current2[xi][0]; //ƽ��ֵ�ۼ����
        temp2 = temp2 + WAVE_all_ave_Current2[xi][1];
        temp3 = temp3 + WAVE_all_ave_Current2[xi][2];
				if( WAVE_all_ave_Current2[xi][1]>max3)  //������Чֵ�����ֵ
					max3= WAVE_all_ave_Current2[xi][1];
    }

    //�õ�12����ƽ����AC��һ�������
    //����ֵ
    ADC2_Filer_value_buf_1[0][0] = temp11 / 12; //ƽ��
    ADC2_Filer_value_buf_1[0][1] = temp12 / 12; //��Чֵ
    //ADC2_Filer_value_buf_1[0][2] = temp13 / 12; //���ֵ
		ADC2_Filer_value_buf_1[0][2] = max13; //���ֵ����Чֵ��

    //ת��ֵ
    ADC2_Filer_value_buf_2[0][0] = temp1 / 12; //ƽ��
    ADC2_Filer_value_buf_2[0][1] = temp2 / 12; //��Чֵ
    //ADC2_Filer_value_buf_2[0][2] = temp3 / 12; //���ֵ
		ADC2_Filer_value_buf_2[0][2] = max3; //���ֵ����Чֵ��


    //====�糡
    temp1 = temp2 = temp3 =max3= 0;
    temp11 = temp12 = temp13 =max13=0;
    for(xi = 0; xi < 12; xi++)
    {
        //����ֵ
        temp11 = temp11 + WAVE_all_ave_E_Field1[xi][0];
        temp12 = temp12 + WAVE_all_ave_E_Field1[xi][1];
        temp13 = temp13 + WAVE_all_ave_E_Field1[xi][2];
				if(WAVE_all_ave_E_Field1[xi][1]>max13)  //��Чֵ�����ֵ
					max13=WAVE_all_ave_E_Field1[xi][1];

        temp1 = temp1 + WAVE_all_ave_E_Field2[xi][0]; //ƽ��ֵ�ۼ����
        temp2 = temp2 + WAVE_all_ave_E_Field2[xi][1];
        temp3 = temp3 + WAVE_all_ave_E_Field2[xi][2];
				if( WAVE_all_ave_E_Field2[xi][1]>max3)  //��Чֵ�����ֵ
					max3= WAVE_all_ave_E_Field2[xi][1];
    }

    //����ֵ
    ADC2_Filer_value_buf_1[1][0] = temp11 / 12; //ƽ��
    ADC2_Filer_value_buf_1[1][1] = temp12 / 12; //��Чֵ
    //ADC2_Filer_value_buf_1[1][2] = temp13 / 12; //���ֵ
		ADC2_Filer_value_buf_1[1][2] = max13; //���ֵ

    //ת��ֵ
    ADC2_Filer_value_buf_2[1][0] = temp1 / 12; //ƽ��
    ADC2_Filer_value_buf_2[1][1] = temp2 / 12; //��Чֵ
    //ADC2_Filer_value_buf_2[1][2] = temp3 / 12; //���ֵ
		ADC2_Filer_value_buf_2[1][2] = max3; //���ֵ
    //====�벨����
    temp1 = temp2 = temp3 =max3= 0;
    temp11 = temp12 = temp13 =max13= 0;
    for(xi = 0; xi < 12; xi++)
    {
        //����ֵ
        temp11 = temp11 + WAVE_half_ave_Current1[xi][0];
        temp12 = temp12 + WAVE_half_ave_Current1[xi][1];
        temp13 = temp13 + WAVE_half_ave_Current1[xi][2];
				if(WAVE_half_ave_Current1[xi][2]>max13)
					max13=WAVE_half_ave_Current1[xi][2];

        temp1 = temp1 + WAVE_half_ave_Current2[xi][0]; //ƽ��ֵ�ۼ����
        temp2 = temp2 + WAVE_half_ave_Current2[xi][1];
        temp3 = temp3 + WAVE_half_ave_Current2[xi][2];
				if( WAVE_half_ave_Current2[xi][2]>max3)
					max3= WAVE_half_ave_Current2[xi][2];
    }
    //����ֵ
    ADC2_Filer_value_buf_1[2][0] = temp11 / 12; //ƽ��
    ADC2_Filer_value_buf_1[2][1] = temp12 / 12; //��Чֵ
    //ADC2_Filer_value_buf_1[2][2] = temp13 / 12; //���ֵ
		ADC2_Filer_value_buf_1[2][2] = max13; //���ֵ


    ADC2_Filer_value_buf_2[2][0] = temp1 / 12; //ƽ��
    ADC2_Filer_value_buf_2[2][1] = temp2 / 12; //��Чֵ
    //ADC2_Filer_value_buf_2[2][2] = temp3 / 12; //���ֵ
		ADC2_Filer_value_buf_2[2][2] = max3; //���ֵ


//===
#if Debug_Usart_OUT_WAVE_12T_CYC==1
    //if(my_Current_exit_Status==1 || my_E_Field_exit_Status==1 || my_Time_Cyc_exit_Status==1 )
    if(my_Current_exit_Status == 1 || my_E_Field_exit_Status == 1  )
        return;



    printf("***cache2 WAVE0 12T DATA --ALL_Current2****\r\n");
    int ii = 0;
    for(ii = 0; ii < 12; ii++)
        printf("%.2f MAX=%.2f\r\n", WAVE_all_ave_Current2[ii][1], WAVE_all_ave_Current2[ii][2]); //12T,ȫ������

    printf("**cache2 *WAVE0 12T DATA --Half_Current2-MAX****\r\n");
    for(ii = 0; ii < 12; ii++)
    {
				 printf("%.2f MAX=%.2f\r\n", WAVE_half_ave_Current2[ii][1], WAVE_half_ave_Current2[ii][2]); //12T,ȫ������
        //printf("%d\r\n", WAVE_half_ave_Current1[ii][2]); //12T,ȫ������
    }
    printf("**cache2 *WAVE0 12T DATA --E_fild2-MAX****\r\n");
    for(ii = 0; ii < 12; ii++)
    {
				 printf("%.2f MAX=%.2f\r\n", WAVE_all_ave_E_Field2[ii][1], WAVE_all_ave_E_Field2[ii][2]); //12T,ȫ������
        //printf("%d\r\n", WAVE_half_ave_Current1[ii][1]); //12T,���
    }

    printf("**cache2*WAVE0 12T DATA --END****\r\n");
#endif
#if Debug_Usart_OUT_WAVE_VALUE==1
    printf("\n ==my_cc1101_all_step=[%x]\n", my_CC1101_all_step);
    my_adc2_convert_dis(0);
#endif


}





/*
���ܣ���2�������е�¼�����ݷ��뵽3�������У�ͬʱ��2���������ƽ��ֵ���뵽3����
*/
void fun_wave2_to_wave3(void)
{
    uint16_t ii = 0, jj = 0;

    //12�����ڵ�960����
    for(ii = 0; ii < WAVE_number_sec2; ii++)
    {
        for(jj = 0; jj < 3; jj++)
            my_wave_record_sec3[jj][ii] = my_wave_record_sec2[jj][ii];

    }
    //12����ÿ���ܵ� ƽ������Ч�����
    for(ii = 0; ii < 12; ii++)
    {
        for(jj = 0; jj < 3; jj++)
        {
            WAVE_half_ave_Current3[ii][jj] = WAVE_half_ave_Current2[ii][jj];
            WAVE_all_ave_Current3[ii][jj] = WAVE_all_ave_Current2[ii][jj];
            WAVE_all_ave_E_Field3[ii][jj] = WAVE_all_ave_E_Field2[ii][jj];

        }
    }
    //12������3��ͨ���������ۼ�ƽ��ֵ
    for(ii = 0; ii < 3; ii++)
    {
        for(jj = 0; jj < 3; jj++)
        {
            ADC2_Filer_value_buf_3[jj][ii] = ADC2_Filer_value_buf_2[jj][ii];  //


        }

    }


}



//================
/*

���ܣ������жϺ������ж϶�·���ӵ�
����ֵ��0Ϊ������0X01Ϊ��·��0x10Ϊ�ӵ�
����λΪ�ӵ�,���λΪ1��ʾ�ӵع��ϣ���3λ������ͬ�Ľӵ����ͣ�
����λΪ��·�����λΪ1��ʾ��·���ϣ���3λ������ͬ�Ķ�·����

*/


//////////////============================
double my_all_current_aver0 = 0;
uint8_t fun_Judege_It_cache3(void)
{
    //double my_all_current_aver0 = 0;

    double my_all_current_aver1[4] = {0};
    double my_all_current_aver2[8] = {0};

    double my_half_current_aver1[4] = {0}; //����ǰ4���벨
    double my_half_current_aver2[8] = {0}; //�벨ADC������ת����õ��ĵ���ֵ
    double my_E_fild_aver0 = 0;
    double my_E_fild_aver1[4] = {0};
    double my_E_fild_aver2[8] = {0};
    double my_cmpar_value[3][12] = {0}; //0��ȫ��������1�е糡��2�а벨����

    uint16_t ii = 0, jj = 0;
    //ȫ����Ӧ��Ч����ֵ
    //my_all_current_aver0 = WAVE_all_ave_Current3[0][1]; //����ǰ����һ��ȫ����ֵ
		my_all_current_aver0 = 0;
    my_all_current_aver0 = my_DAC_Line_I;

    //�ж�ǰ�糡��ƽ��ֵ
    //my_E_fild_aver0 = (WAVE_all_ave_E_Field3[0][1] + WAVE_all_ave_E_Field3[1][1] + WAVE_all_ave_E_Field3[2][1] + WAVE_all_ave_E_Field3[3][1]) / 4.0 / E_cell;
    my_E_fild_aver0 = my_DAC_Line_Efild;


    for(ii = 0; ii < 4; ii++)
    {
        my_half_current_aver1[ii] = WAVE_half_ave_Current3[ii][1]; //����ǰ4������
        my_all_current_aver1[ii] = WAVE_all_ave_Current3[ii][1];
        my_E_fild_aver1[ii] == WAVE_all_ave_Current3[ii][1];
    }



    for(ii = 0; ii < 8; ii++)
    {

        my_all_current_aver2[ii] = WAVE_all_ave_Current3[4 + ii][1]; //�����8�����ڶ�Ӧ����Ч����ֵ
        my_half_current_aver2[ii] = WAVE_half_ave_Current3[4 + ii][1];
        my_E_fild_aver2[ii] = WAVE_all_ave_E_Field3[4 + ii][1] / E_cell; //�жϺ�8�����ڵ�ÿ�����ڵĵ糡ֵ
    }


    //==========================




    //�жϺ�8�����ڣ����ж�ǰ��ֵ�Ĳ�ֵ����
    for(jj = 0; jj < 8; jj++)
    {
        my_cmpar_value[0][jj] = (my_all_current_aver2[jj] - my_all_current_aver0); //��8�����ڵ�ÿ�����ڵĵ���ֵ����ȥ����ǰ��ƽ��ֵ
        if(my_cmpar_value[0][jj] < 0) my_cmpar_value[0][jj] = 0;

        //my_cmpar_value[1][jj] = (my_E_fild_aver2[jj] - my_E_fild_aver0); //��8�����ڣ���ȥǰ4�����ڵ�ƽ��ֵ���糡ֵ
        my_cmpar_value[1][jj] = fabs(my_E_fild_aver0 - my_E_fild_aver2[jj]);


#if Debug_usart_out_wave_cmpare_data==1
        printf("cmpar=%.2f\r\n", my_cmpar_value[0][jj]); //������ֵ
#endif
    }

    //�����͵糡�仯������¼
    my_short_circuit_count = 0; //��·�����������ޣ�����
		my_short_circuit_count2=0;
    my_after_short_stop_count1 = 0;
    my_after_stop1_normal_count = 0;
    my_after_short_stop_count2 = 0;
    my_E_fild_change_count = 0;
    my_E_fild_min_count = 0;

    //��ʼ�ж�
    for(ii = 0; ii < 4; ii++)
    {
        if((my_all_current_aver1[ii] - my_all_current_aver0) >= Current_D_value)
            my_short_circuit_count++;

        if(fabs(my_E_fild_aver0 - my_E_fild_aver1[ii]) >= E_fild_D_value)
            my_E_fild_change_count++;
    }

    //�жϺ���ж�

    for(ii = 0; ii < 8; ii++)
    {
        if(my_cmpar_value[0][ii] >= Current_D_value)
            my_short_circuit_count++;  //�ж�ʱ�̺󣬶�·���ڼ��������ô��ڽ�Ծֵ�����ж� 1

        if(my_half_current_aver2[ii] <= my_HA_Zero_data && my_after_stop1_normal_count == 0)
            my_after_short_stop_count1++;  //�ж�ʱ�̺󣬵�һ��ͣ�����ڼ�����������ͣ�磬����û������ 2

        if(my_half_current_aver2[ii] > my_HA_Zero_data && my_after_short_stop_count1 > 0)
            my_after_stop1_normal_count++; //��һ��ͣ�������״̬ͳ��,�Ƿ���Ҫ���ǰ벨�����Ĳ���������� 3

        if(my_half_current_aver2[ii] <= my_HA_Zero_data && my_after_stop1_normal_count > 0)
            my_after_short_stop_count2++;  //����ͣ���ͳ�� 4

        if(fabs(my_cmpar_value[1][ii]) >= E_fild_D_value) //
            my_E_fild_change_count++;//�ж�ʱ��󣬣��糡���䳬����ֵ���ڼ��� 5


        if(fabs(my_E_fild_aver2[ii]) <= E_fild_threshold_value)
            my_E_fild_min_count++;  //�жϺ󣬼�¼�糡��ֵ̬С�ڷ�ֵ�Ĵ��� 6


    }

    //�ж�ʱ��ǰ��ͣ�����ڵĴ��� 7
    my_befor_short_stop_count = 0;
    for(ii = 0; ii < 4; ii++)
    {
        if(my_half_current_aver1[ii] <= my_HA_Zero_data)
            my_befor_short_stop_count++;

    }

    printf("my_short_circuit_count=%d\r\n", my_short_circuit_count);
    //======12T======
#if Debug_Usart_OUT_WAVE_12T_Interupt==1
    printf("***WAVE0 12T DATA --ALL_Current3****\r\n");
    for(ii = 0; ii < 12; ii++)
        printf("%.2f MAX=%.2f\r\n", WAVE_all_ave_Current3[ii][1], WAVE_all_ave_Current3[ii][2]); //12T,ȫ������

    printf("***WAVE0 12T DATA --Half_Current3****\r\n");
    for(ii = 0; ii < 12; ii++)
        printf("%.2f\r\n", WAVE_half_ave_Current3[ii][1]); //12T,ȫ������

    printf("***WAVE0 12T DATA --END****\r\n");
#endif
    //=======960DATA======
#if Debug_Usart_OUT_WAVE_960Data_Interupt==1
    for(ii = 0; ii < 960; ii++)
        printf("%.2f\n", my_wave_record_sec3[0][ii]*MY_VDD / 4096.0 * (my_i_ratio_value * my_I_100A_Radio)); //960data,ȫ������
#elif Debug_Usart_OUT_WAVE_960Data_Interupt==2
    for(ii = 0; ii < 960; ii++)
        printf("%.2f\r\n", my_wave_record_sec3[1][ii]*MY_VDD / 4096.0 * (my_i_ratio_value * my_I_100A_Radio)); //960data,ȫ���糡
#elif Debug_Usart_OUT_WAVE_960Data_Interupt==3
    for(ii = 0; ii < 960; ii++)
        printf("%.2f\r\n", my_wave_record_sec3[2][ii]*MY_VDD / 4096.0 * (my_i_ratio_value * my_I_100A_Radio)); //960data,ȫ���糡
#endif

    return 0;
}




//////////////////////////=====================
/*
���ܣ���2��������м��㣬ͳ�Ƴ���
��1���жϺ� ��·����������
��2���жϺ� ͣ������������
��3���ж�ǰ��ͣ��������
��4���жϺ�糡���䳬����ֵ����
��5���жϺ�糡��ֵ̬С������ֵ������

*/

uint8_t fun_Judege_It_cache2(void)
{
    //double  my_all_current_aver0 = 0;
    double	my_all_current_aver2[12] = {0};
    double  my_half_current_aver2[12] = {0}; //�벨ADC������ת����õ��ĵ���ֵ
    double  my_E_fild_aver0 = 0;
    double  my_E_fild_aver2[12] = {0};
    double  my_cmpar_value[3][12] = {0}; //0��ȫ��������1�е糡��2�а벨����

    uint8_t ii = 0, jj = 0;
    //ȫ����Ӧ��Ч����ֵ
    // my_all_current_aver0=(WAVE_all_ave_Current2[0][1]+WAVE_all_ave_Current2[1][1]+WAVE_all_ave_Current2[2][1]+WAVE_all_ave_Current2[3][1])/4.0;
    //my_all_current_aver0 = WAVE_all_ave_Current3[0][1]; //����ʱ��ǰ��4�����ڵ�ƽ����Ч����ֵ


    for(ii = 0; ii < 12; ii++)
    {
        //my_all_current_aver1[ii]=(WAVE_all_ave_Current2[4+ii][1]-1.2)/V_cell;  //�����8�����ڶ�Ӧ����Ч����ֵ
        my_all_current_aver2[ii] = WAVE_all_ave_Current2[ii][1];
        my_half_current_aver2[ii] = WAVE_half_ave_Current2[ii][1]; //��Чֵ��ת��
        my_E_fild_aver2[ii] = WAVE_all_ave_E_Field2[ii][1] / E_cell;
    }

    //�糡У���������
    my_E_fild_aver0 = WAVE_all_ave_E_Field3[0][1] / E_cell;

    //����ȫ�������͵糡�ı仯��ֵ
    for(jj = 0; jj < 12; jj++)
    {
        my_cmpar_value[0][jj] = my_all_current_aver2[jj] - my_all_current_aver0;
        if(my_cmpar_value[0][jj] < 0) my_cmpar_value[0][jj] = 0;
        my_cmpar_value[1][jj] = my_E_fild_aver2[jj] - my_E_fild_aver0;
    }

    //�����͵糡�仯������¼
    for(ii = 0; ii < 12; ii++)
    {


        if(my_cmpar_value[0][ii] >= Current_D_value)
            my_short_circuit_count2++;  //�ж�ʱ�̺󣬶�·���ڼ���
        if(my_half_current_aver2[ii] <= my_HA_Zero_data && my_after_stop1_normal_count == 0)
            my_after_short_stop_count1++;  //�ж�ʱ�̺󣬵�һ��ͣ�����ڼ�����������ͣ�磬����δ����
        if(my_half_current_aver2[ii] > my_HA_Zero_data && my_after_short_stop_count1 > 0)
            my_after_stop1_normal_count++; //��һ��ͣ�������״̬ͳ��,�Ƿ���Ҫ���ǰ벨�����Ĳ����������
        if(my_half_current_aver2[ii] <=my_HA_Zero_data && my_after_stop1_normal_count > 0)
            my_after_short_stop_count2++;  //����ͣ���ͳ��



        if((my_cmpar_value[1][ii]) >= E_fild_D_value) //
            my_E_fild_change_count++;//�ж�ʱ��󣬣��糡���䳬����ֵ���ڼ���
        if((my_E_fild_aver2[ii]) <= E_fild_threshold_value)
            my_E_fild_min_count++;  //�жϺ󣬼�¼�糡��ֵ̬С�ڷ�ֵ�Ĵ���


    }
    //======12T======
#if Debug_Usart_OUT_WAVE_Last_12T_Interupt==1
    printf("***interrupt WAVE-%d 12T DATA --ALL_Current****\r\n", my_IT_Count);
    for(ii = 0; ii < 12; ii++)
        printf("%.2f\r\n", WAVE_all_ave_Current2[ii][1]); //12T,ȫ������

    printf("***interrupt WAVE-%d 12T DATA --Half_Current****\r\n", my_IT_Count);
    for(ii = 0; ii < 12; ii++)
        printf("%.2f\r\n", WAVE_half_ave_Current2[ii][1]); //12T,ȫ������

    printf("***interrupt WAVE-%d 12T DATA --END****\r\n", my_IT_Count);
#endif



    return 0;
}
/*

���ܣ������жϺ�3���¼�����ݵķ�������3�������2������ķ��������״̬�ļ���ֵ�ķ���

	my_Fault_End_Status=0x00;  //��4λΪ�ӵأ���4λΪ��·
  0x01,��·��0X03�غ�բ��0X0

	my_befor_short_stop_count=0;
	my_short_circuit_count=0;
	my_after_short_stop_count=0;

	my_E_fild_change_count=0;
	my_E_fild_min_count=0;

*/



uint8_t fun_Judege_It_end(void)
{
    //��·
    my_Fault_Current_End_Status = 0x00;

    if(	my_befor_short_stop_count == 0 && my_short_circuit_count > 0 && my_short_circuit_count <= 12 && my_short_circuit_count2 <20&& my_after_short_stop_count1 > 0  && my_after_stop1_normal_count == 0 && my_after_short_stop_count2 == 0 )
        my_Fault_Current_End_Status = (0X01 | my_Fault_Current_End_Status); //��1��--��·1

    if(my_befor_short_stop_count > 0 && my_short_circuit_count > 0 && my_after_short_stop_count1 < 4  && my_after_stop1_normal_count == 0  && my_after_short_stop_count2 == 0)
        my_Fault_Current_End_Status = 0X02 | my_Fault_Current_End_Status; //��2��--ͻ�ϸ���ӿ��,������

    if(my_befor_500ms_normal_count > 0 && my_befor_short_stop_count > 0 && my_short_circuit_count > 0 && my_after_short_stop_count1 > 0  && my_after_stop1_normal_count == 0  && my_after_short_stop_count2 == 0)
        my_Fault_Current_End_Status = 0X04 | my_Fault_Current_End_Status; //��3���ǹ������غ�բӿ�������ɹ���,������--��·�ж�
		if(my_befor_500ms_normal_count > 0 && my_befor_short_stop_count == 0 && my_short_circuit_count ==0 && my_after_short_stop_count1 > 0  && my_after_stop1_normal_count > 0  && my_after_short_stop_count2 > 0)
        my_Fault_Current_End_Status = 0X04 | my_Fault_Current_End_Status; //��3���ǹ������غ�բӿ�������ɹ���,������--�糡�ж�
		
		if(my_befor_500ms_normal_count > 0 && my_befor_short_stop_count > 0 && my_short_circuit_count > 0 && my_after_short_stop_count1 == 0  && my_after_stop1_normal_count == 0  && my_after_short_stop_count2 == 0)
        my_Fault_Current_End_Status = 0X0E; //�ǹ������բ�ɹ�,������--��·�ж�
		if(my_befor_500ms_normal_count > 0 && my_befor_short_stop_count ==0 && my_short_circuit_count == 0 && my_after_short_stop_count1 > 0  && my_after_stop1_normal_count > 0  && my_after_short_stop_count2 > 0)
        my_Fault_Current_End_Status = 0X0E; //�ǹ������բ�ɹ�,������--�糡�ж�

    if(my_befor_short_stop_count == 0 && my_short_circuit_count > 0 && my_after_short_stop_count1 == 0  && my_after_stop1_normal_count == 0  && my_after_short_stop_count2 == 0)
        my_Fault_Current_End_Status = 0X08 | my_Fault_Current_End_Status; //��4��--����˲��ͻ�䣬������

    if(my_befor_short_stop_count == 0 && my_short_circuit_count > 0 && my_short_circuit_count2 >=20 && my_after_short_stop_count1 > 0 && my_after_stop1_normal_count == 0  && my_after_short_stop_count2 == 0)
        my_Fault_Current_End_Status = 0X10 | my_Fault_Current_End_Status; //��5��--�˹�Ͷ�д󸺺�,������

    if(my_befor_500ms_normal_count == 0 && my_befor_short_stop_count > 0 && my_short_circuit_count > 0 && my_after_short_stop_count1 > 0  && my_after_stop1_normal_count == 0  && my_after_short_stop_count2 == 0)
        my_Fault_Current_End_Status = 0X20 | my_Fault_Current_End_Status; //��6��---���غ�բ,������


    if(my_befor_short_stop_count == 0 && my_short_circuit_count > 0 && my_after_short_stop_count1 > 0  && my_after_stop1_normal_count > 0  && my_after_short_stop_count2 == 0)
        my_Fault_Current_End_Status = 0X41 | my_Fault_Current_End_Status; //��·2���غ�բ�ɹ�--��·�ж�
    if(my_befor_short_stop_count == 0 && my_short_circuit_count > 0 && my_after_short_stop_count1 > 0  && my_after_stop1_normal_count > 0  && my_after_short_stop_count2 > 0)
        my_Fault_Current_End_Status = 0X81 | my_Fault_Current_End_Status; //��·3���غ�բ���ɹ�--��·�ж�




    




    //else
    //my_Fault_Current_End_Status=0X0E; //δ֪״̬
    //�糡�ж�
    if(my_E_fild_change_count > 0 && my_E_fild_min_count > 0 && my_after_short_stop_count1 == 0) 
        my_Fault_E_Fild_End_Status = 0x01; //�糡���䣬�糡С���ض�ֵ���������������ӵع���
    else if(my_E_fild_change_count > 0 && my_E_fild_min_count > 0 && my_after_short_stop_count1 > 0)
        my_Fault_E_Fild_End_Status = 0x02; //�糡���䣬����û�е����ˣ�ͣ��


#if Debug_Usart_OUT_WAVE_End_Just_Interupt==1
    printf("  I=[%XH],  E=[%XH]\r\n", my_Fault_Current_End_Status, my_Fault_E_Fild_End_Status);
    //printf("500ms  ��·ǰ�е�=%d, ǰͣ��=%d, ��·=%d, ��·��ͣ��=%d, ͣ�������=%d, ������ͣ��=%d\r\n",
    printf("500ms  frontX_A_A=%d, frontX_NOA_B=%d, short_circurt_C=%d, afterX_NOA_D=%d, afterNOA_normal_E=%d, afternorm_NOA_F=%d,short_circurt2-G=%d\r\n",
           my_befor_500ms_normal_count,
           my_befor_short_stop_count,
           my_short_circuit_count,
           my_after_short_stop_count1,
           my_after_stop1_normal_count,
           my_after_short_stop_count2,
						my_short_circuit_count2);
    printf("��·ǰ�е�-A, ��·ǰͣ��-B, ��·-C, ��·��ͣ��-D, ͣ�������-E, ������ͣ��-F\n");

#endif

    return 0;
}

/*
���ܣ�¼�����ݴ洢

*/
void my_fun_wave_rec(void)
{
    uint32_t ii = 0;
    ii++;
    while(fun_my_wave1_to_wave2() != 1)
    {
        ii++;
        if(ii > 0xFFFFF)
            break;

    }

}

//==========��·�ж�=============================
/*
���ܣ�
��1���Ƚ����״�¼����1����2����2����3���������г����ж�
��2��200ms���ٴν��У����ж���¼����1����2�����ظ�10�Σ��ܹ�2�룬ÿ�ζ���2����������ж�
��3����10��¼���������ۺ��жϣ��ó�����
  ˼·������¼����ÿ���200ms¼��һ�Σ��ܹ�2s��������Σ��ó����ۡ�

*/


uint8_t my_fun_current_exit_just(void)
{
    my_IT_Count++; //�����жϹ����жϳ������ͳ��
    //USART_printf(&huart2, "---just interrupt count=%d-----\n", my_IT_Count);
    uint8_t temp8 = 0;
    //1.�ж��¼�����
    if(my_IT_status == 0 && (my_Current_exit_Status == 1 || my_E_Field_exit_Status == 1))
    {
        if(my_Current_exit_Status == 1)
        {
            //my_dianliu_exit_add = my_wave_write_add;
            //my_Current_Exit_add = my_dianliu_exit_add;
            my_Wave_It_add = my_dianliu_exit_add;

        }
        else if(my_E_Field_exit_Status == 1)
        {
            //my_dianliu_exit_add = my_wave_write_add;
            //my_E_Field_exit_add = my_dianliu_exit_add;
            my_Wave_It_add = my_E_Field_exit_add;

        }

        my_adc_1_convert();
        my_fun_wave_rec();
        my_adc2_convert2(0); //����12�����ڵ�ÿ�����ڵ�3����������12�����ڵ��ۼ�ƽ��ֵ,��2�������е�����ת�����ˡ�
        my_IT_status = 1; //Ϊ1��ʾ���ж��¼��Ѿ���Ӧ�ˣ���Ҫ������һ�����ȴ�3����ڽ��в������ж�ͣ��״̬
        my_IT_Count = 0; //�жϳ���ͳ�����㣬�´ν���Ϊ��һ��


        fun_wave2_to_wave3(); //���������е��������ݷ��뵽���������У�������ֵ
        //��3��������������ж�
        fun_Judege_It_cache3();//���״�¼��������3��������У���ȫ�ֹ���״̬��¼������ֵ

    }

    //============
    if(my_IT_Count >= 1) //���ж�ζ����������㣬Ȼ����и���״̬���ж�ǰͣ��/��·,�жϺ�ͣ��/��·��ͳ��
    {
        my_dianliu_exit_add = my_wave_write_add;
        my_fun_wave_rec();
        my_adc2_convert2(0); //����12�����ڵ�ÿ�����ڵ�3����������12�����ڵ��ۼ�ƽ��ֵ,���ݷ��ڶ���������
        fun_Judege_It_cache2();//����2�����壬��ȫ�ֹ���״̬��¼������ֵ,���Խ��к�8������160ms�������Σ�10�ξ���1.6s
    }


    if(my_IT_Count >= (5 * 2)) //2�����Ժ󣬽�������״̬���ж�
    {
        //�����жϺ�2���ӵ�¼�����ݷ����󣬽��й��������ж�
        fun_Judege_It_end();

        //���������������ж��¼�
        my_IT_Count = 0;  //�ж��¼����������������жϽ���
        my_IT_status = 0;
        my_Current_exit_Status = 0;
        my_E_Field_exit_Status = 0;

        temp8 = 1;

    }
    //1.��������
    return temp8; //1Ϊ��ʾ�����ˣ�0Ϊû�н���




}



/*
���ܣ��ж���·ͣ��״̬���Եص糡״̬
˼·���˺���������¼������1����2���Ĵ�����Ҳ������ת����ֱ��ȡ12T������2������ֵ��ת�����ֵ�������ð벨�����ж��Ƿ�ͣ�硣
�÷����˺�������Ҫ����DAC���ú������������ڲ���������ߣ�������������һ��������¼������Ϳ����ж�ͣ�磬�ͶԵص糡��ֵ��
*/

double my_E_Fild_old=0;
void my_fun_get_Line_stop_Efild(void)
{

    uint16_t  my_step = 00;

    //return ;
//ȡ��ǰ����
    my_fun_wave1_to_wave2_old_data();
    my_adc2_convert2(0);


//��̬�����ж�
    if( ADC2_Filer_value_buf_2[1][0] <= MY_Efile_Zero_data &&  ADC2_Filer_value_buf_2[2][1] <= my_HA_Zero_data) //�糡Ϊ0���벨Ϊ0
        my_Line_Current_stop_status = 1; //��ʾͣ��,  û�е糡��û�е���

    else if(ADC2_Filer_value_buf_2[0][1] >= my_A_Zero_data &&  ADC2_Filer_value_buf_2[1][1] > MY_Efile_floor_data   && ADC2_Filer_value_buf_2[2][1] > my_HA_Zero_data)
        my_Line_Current_stop_status = 3; //��ʾ������  ��· �е������е糡��

    else if(ADC2_Filer_value_buf_2[0][1] >= my_A_Zero_data &&  ADC2_Filer_value_buf_2[1][1] <= MY_Efile_floor_data 	&& ADC2_Filer_value_buf_2[2][1] > my_HA_Zero_data)
        my_Line_Current_stop_status = 2; //��ʾ�ӵأ�  ��·�糡��С���е���
		
//�糡����ٷֱ�
		if((my_E_Fild_old-ADC2_Filer_value_buf_2[1][1])*1.0/my_E_Fild_old>0.45 && my_E_Fild_old>ADC2_Filer_value_buf_2[1][1] && my_E_Fild_old>MY_Efile_floor_data  && ADC2_Filer_value_buf_2[2][1] > my_HA_Zero_data && ADC2_Filer_value_buf_2[0][1] >= my_A_Zero_data)
		{
			my_Line_Current_stop_status = 2;  //���ðٷֱȷ����жϣ��ӵ�
		}
		my_E_Fild_old=ADC2_Filer_value_buf_2[1][1];
//��̬��������400A�о�
		if(ADC2_Filer_value_buf_2[0][1]>400 && ADC2_Filer_value_buf_2[1][1]>MY_Efile_Zero_data)
		{
				fun_wave2_to_wave3();
        my_Fault_Current_End_Status = 0XF3;
        my_Fault_E_Fild_End_Status = 00;
        printf("==return Short over 400--1 A_S=%d  E_S=%d\n",my_Fault_Current_End_Status,my_Fault_E_Fild_End_Status);
        my_zsq_ALarm_send_status = 1;
        my_step = 0x0002; //���ͱ�������Ϣ����
        xQueueSend(myQueue01Handle, &my_step, 100);
			
		}
		
		
		

//����״̬�ָ�
    if(	my_Line_Current_stop_status == 3 && (my_Fault_Current_End_Status != 0 || my_Fault_E_Fild_End_Status != 0))
    {
        fun_wave2_to_wave3();
				printf("==all return normal--1 A=%d  E=%d\n",my_Fault_Current_End_Status,my_Fault_E_Fild_End_Status);
        my_Fault_Current_End_Status = 00;
        my_Fault_E_Fild_End_Status = 00;
       
        my_zsq_ALarm_send_status = 1;
        my_step = 0x0002; //���ͱ�������Ϣ����
        xQueueSend(myQueue01Handle, &my_step, 100);
    }
//��·�����ָ�,ֻ���ǵ����������ǵ糡
		else if(my_Line_Current_stop_status==2 && my_Fault_Current_End_Status != 0)
		{
				fun_wave2_to_wave3();
				printf("==current return normal--1 A=%d  E=%d\n",my_Fault_Current_End_Status,my_Fault_E_Fild_End_Status);
        my_Fault_Current_End_Status = 00;
        //my_Fault_E_Fild_End_Status = 00;
        
        my_zsq_ALarm_send_status = 1;
        my_step = 0x0002; //���ͱ�������Ϣ����
        xQueueSend(myQueue01Handle, &my_step, 100);
			
		}

//ͣ��״̬�ϴ�
    if(my_Line_Current_stop_status == 1 && my_Line_Current_stop_last_status == 3 ) //�ϴ�����������ͣ�磬�ϴ�
    {
        fun_wave2_to_wave3();
        my_Fault_Current_End_Status = 0xFE;
        my_Fault_E_Fild_End_Status = 0xFE;
        my_Line_Current_stop_last_status = 1; //ͣ��
        printf("==return normal--2---stop A=%d  E=%d\n",my_Fault_Current_End_Status,my_Fault_E_Fild_End_Status);
        my_zsq_ALarm_send_status = 1;
        my_step = 0x0002; //���ͱ�������Ϣ����
        xQueueSend(myQueue01Handle, &my_step, 100);
    }
//�ӵ�״̬�ϴ�
    else if(my_Line_Current_stop_status == 2 && my_Line_Current_stop_last_status == 3)
    {
        my_E_Field_exit_add = my_E_fild_time_add;
        fun_wave2_to_wave3();

        my_Fault_E_Fild_End_Status = 0x01;
        my_Line_Current_stop_last_status = my_Line_Current_stop_status;
        printf("==return normal--3---jiedi= A=%d  E=%d\n",my_Fault_Current_End_Status,my_Fault_E_Fild_End_Status);
        my_zsq_ALarm_send_status = 1;
        my_step = 0x0002; //���ͱ�������Ϣ����
        xQueueSend(myQueue01Handle, &my_step, 100);

    }
//��·��������ʷ���ݱ仯
    else if(my_Line_Current_stop_status == 3 && my_Line_Current_stop_last_status != 3)
    {

        my_Line_Current_stop_last_status = 3; //����ʷ״̬���ָ�Ϊ����״̬
    }


#if Debug_Usart_OUT_LINE_STOP_STATUS==1
    printf("--Line stop staus=[%d]--A=%d,E=%d \n", my_Line_Current_stop_status,my_Fault_Current_End_Status,my_Fault_E_Fild_End_Status);
#endif

//�糡ֵ���

    if(my_Line_Current_stop_status == 1)
        my_line_Current_value = 0; //��ȫ������Ʈֵ��Ϊ0
    else
        ;  //����䣬Ŀ������ʾͣ���ʱ�����Ʈֵ�������������Ʈ�������δ˿����
    my_line_Current_value = ADC2_Filer_value_buf_2[0][1] * 10; //��������Чֵ
    my_Line_Efild_valu = ADC2_Filer_value_buf_2[1][1]; //��õĵ糡����Чֵ
#if Debug_Usart_OUT_LINE_Efield_STATUS==1
    printf("--Line current=%.2f  Efild=%d--\r\n", my_line_Current_value / 10.0, my_Line_Efild_valu);

#endif
}

//=============
//���ݴ洢����2,12�������þ�����

void my_fun_wave1_to_wave2_old_data(void)
{
    int ii = 0;
    uint16_t my_end_add = my_wave_write_add;
    uint16_t my_start_add = 0;
    uint16_t temp_add = 0;
    uint16_t count_number = rec_T_Count * 80;
    my_E_fild_time_add = my_wave_write_add;
    if(my_end_add >= count_number)
    {
        my_start_add = my_end_add - count_number;
    }
    else
    {
        my_start_add = WAVE_number - (count_number - my_end_add);
    }

    //��������
    temp_add = my_start_add;
    for(ii = 0; ii < count_number; ii++)
    {
        my_wave_record_sec2[0][ii] = my_wave_record[0][temp_add];
        my_wave_record_sec2[1][ii] = my_wave_record[1][temp_add];
        my_wave_record_sec2[2][ii] = my_wave_record[2][temp_add];

        temp_add++;
        if(temp_add >= WAVE_number)
            temp_add = 0;
    }
    if(count_number < WAVE_number_sec2)
    {
        for(; ii < WAVE_number_sec2; ii++)
        {
            my_wave_record_sec2[0][ii] = 0;
            my_wave_record_sec2[1][ii] = 0;
            my_wave_record_sec2[2][ii] = 0;
        }

    }
}

/*
���ܣ���ѯ����ã��糡״̬
*/
//void my_fun_query_Efild(void)
//{

//    uint16_t my_step = 0;
//    my_fun_wave1_to_wave2_old_data();
//    my_adc2_convert2(0);

//    //ͣ���ж�
//    if( ADC2_Filer_value_buf_2[1][2] <= MY_Efile_Zero_data &&  ADC2_Filer_value_buf_2[2][2] <= my_HA_Zero_data) //�糡Ϊ0���벨Ϊ0
//        my_Line_Current_stop_status = 1; //��ʾͣ��,û�е糡��û�е���
//    else if(ADC2_Filer_value_buf_2[0][1] >= my_A_Zero_data &&  ADC2_Filer_value_buf_2[1][1] > MY_Efile_Zero_data)
//        my_Line_Current_stop_status = 0; //��ʾ��������· �е������е糡��
//    else if(ADC2_Filer_value_buf_2[0][1] >= my_A_Zero_data &&  ADC2_Filer_value_buf_2[1][1] <= MY_Efile_floor_data)
//        my_Line_Current_stop_status = 2; //��ʾ�ӵأ���·�糡��С���е���

//    //�ӵز���
//    if(my_Line_Current_stop_status == 2  && my_Line_Current_stop_last_status == 0)
//    {
//        my_E_Field_exit_add = my_E_fild_time_add;
//        fun_wave2_to_wave3();

//        my_Fault_E_Fild_End_Status = 0x01;
//        my_Line_Current_stop_last_status = my_Line_Current_stop_status;
//        my_zsq_ALarm_send_status = 1;
//        my_step = 0x0002; //���ͱ�������Ϣ����
//        xQueueSend(myQueue01Handle, &my_step, 100);

//    }

//}
