#include "maxtric.h"
#include "stm32f1xx_hal.h"
#include "main.h"
extern SPI_HandleTypeDef hspi2;

void MAX7219_write(uint8_t address, uint8_t data)
{
	uint8_t buf[2]={0};
	buf[0]=address;
	buf[1]=data;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,buf, 2,1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(1);
}
uint8_t booltoByte(int (*ShowArray_ptr)[8])
{
	uint8_t data = 0x00;
  for (int i = 0; i < 8; i++)
    data = data << 1 | ShowArray_ptr[0][i];
  return data; 
}
void draw(int (*ShowArray_ptr)[8])
{
  MAX7219_write(Digit0, booltoByte(&ShowArray_ptr[0]));
  MAX7219_write(Digit1, booltoByte(&ShowArray_ptr[1]));
  MAX7219_write(Digit2, booltoByte(&ShowArray_ptr[2]));
  MAX7219_write(Digit3, booltoByte(&ShowArray_ptr[3]));
  MAX7219_write(Digit4, booltoByte(&ShowArray_ptr[4]));
  MAX7219_write(Digit5, booltoByte(&ShowArray_ptr[5]));
  MAX7219_write(Digit6, booltoByte(&ShowArray_ptr[6]));
  MAX7219_write(Digit7, booltoByte(&ShowArray_ptr[7]));
}
void MAX7219clear()
{
  MAX7219_write(Digit0, 0x00);
  MAX7219_write(Digit1, 0x00);
  MAX7219_write(Digit2, 0x00);
  MAX7219_write(Digit3, 0x00);
  MAX7219_write(Digit4, 0x00);
  MAX7219_write(Digit5, 0x00);
  MAX7219_write(Digit6, 0x00);
  MAX7219_write(Digit7, 0x00);
}
