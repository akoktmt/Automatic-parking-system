#include "stm32f1xx_hal.h"
#include "main.h"
#include "hcsr04.h"
	uint32_t pMillis;
	uint32_t val1 = 0;
	uint32_t val2 = 0;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
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
int check_distance_out(int distance)
{
				HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_SET);
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
	   while (__HAL_TIM_GET_COUNTER (&htim4) < 10);  // wait for 10 us
	   HAL_GPIO_WritePin(TRIG_PORT1, TRIG_PIN1, GPIO_PIN_RESET);

	   pMillis = HAL_GetTick();
	   while (!(HAL_GPIO_ReadPin (ECHO_PORT1, ECHO_PIN1)) && pMillis + 10 >  HAL_GetTick());
	   val1 = __HAL_TIM_GET_COUNTER (&htim4);

	   pMillis = HAL_GetTick();
	   while ((HAL_GPIO_ReadPin (ECHO_PORT1, ECHO_PIN1)) && pMillis + 50 > HAL_GetTick());
	   val2 = __HAL_TIM_GET_COUNTER (&htim4);

	   distance = (val2-val1)* 0.034/2;
				return distance;
}
