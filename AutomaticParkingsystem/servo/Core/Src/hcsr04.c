#include "stm32f1xx_hal.h"
#include "main.h"
#include "hcsr04.h"
	uint32_t pMillis;
	uint32_t val1 = 0;
	uint32_t val2 = 0;

extern TIM_HandleTypeDef htim1;
int check_distance_in(int distance)
{
				HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
	  __HAL_TIM_SET_COUNTER(&htim1, 0);
	   while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
	   HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

	   pMillis = HAL_GetTick();
	   while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
	   val1 = __HAL_TIM_GET_COUNTER (&htim1);

	   pMillis = HAL_GetTick();
	   while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
	   val2 = __HAL_TIM_GET_COUNTER (&htim1);
	   distance= (val2-val1)* 0.034/2;
			return distance;
}
