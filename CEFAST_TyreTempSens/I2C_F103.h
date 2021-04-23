#include <stm32f10x.h>
#include <stdint.h>

void I2C_Start(void);
void I2C_Address(uint8_t address);
void I2C_Config(void);
void I2C_Write(uint8_t data);
void I2C_WriteBuf(uint8_t *data, int size);
void I2C_Stop(void);
void I2C_Read(uint8_t addr, uint8_t *data, int size);