
#include "tim.h"
#include "stm32f1xx_hal.h"

//void my_start_systick_time(uint16_t delay_time);
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

//TIM_HandleTypeDef htim5;
void HAL_TIM_PeriodElapsedCallback2(TIM_HandleTypeDef *htim);

