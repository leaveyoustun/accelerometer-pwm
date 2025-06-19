#include "main.h"

extern uint32_t g_counter;
extern uint32_t g_channel_1_state;
extern uint32_t g_channel_2_state;
extern FMPI2C_HandleTypeDef hfmpi2c1;

#if 0
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  g_counter = 0;
  g_channel_1_state = 16;
  g_channel_2_state = 8;

}
#endif
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);
  g_channel_1_state = 0;
  g_channel_2_state = 0;

}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == MPU6050_INT_Pin){
    mpu6050_interrupt_handle(&hfmpi2c1);
  }

}
