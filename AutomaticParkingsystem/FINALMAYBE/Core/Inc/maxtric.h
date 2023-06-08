#ifndef MAXTRIC
#define MAXTRIC
#include "stm32f1xx_hal.h"
#include "main.h"
#define No_Op  0x00
#define Digit0 0x01
#define Digit1 0x02
#define Digit2 0x03
#define Digit3 0x04
#define Digit4 0x05
#define Digit5 0x06
#define Digit6 0x07
#define Digit7 0x08

#define DecodeMode  0x09
#define Intensity   0x0A
#define ScanLimit   0x0B
#define Shutdown    0x0C
#define DisplayTest 0x0F
void MAX7219clear(void);
void MAX7219_write(uint8_t address, uint8_t data);
void draw(int (*ShowArray_ptr)[8]);
uint8_t booltoByte(int (*ShowArray_ptr)[8]);
#endif
