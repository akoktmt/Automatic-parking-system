#ifndef HCSR04
#define HCSR04
#define TRIG_PIN GPIO_PIN_0
#define TRIG_PORT GPIOB
#define ECHO_PIN GPIO_PIN_10
#define ECHO_PORT GPIOB

#define TRIG_PIN1 GPIO_PIN_1
#define TRIG_PORT1 GPIOB
#define ECHO_PIN1 GPIO_PIN_11
#define ECHO_PORT1 GPIOB
#include "stm32f1xx_hal.h"
int check_distance_in(int distance_in);
int check_distance_out(int distance_out);
#endif
