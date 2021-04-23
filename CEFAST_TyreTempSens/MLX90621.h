#include <stm32f10x.h>
#include <math.h>
#include <stdint.h>

#define SCALEALPHA 0.000001
    
typedef struct{
	int16_t vTh25;
  float kT1;
  float kT2;        
  float tgc;
  float KsTa;
  float ksTo;
  float alpha[64];    
  float ai[64];
  float bi[64];    
  float cpAlpha;
  float cpA;
  float cpB;
  uint16_t brokenPixels[5];
  uint16_t outlierPixels[5];  
} paramsMLX90621;
    
int MLX90621_DumpEE(uint8_t *eeData);
int MLX90621_GetFrameData(uint16_t *frameData);
int MLX90621_Configure(uint8_t *eeData);
int MLX90621_GetOscillatorTrim(uint16_t *oscTrim);
int MLX90621_GetConfiguration(uint16_t *cfgReg);
int MLX90621_ExtractParameters(uint8_t *eeData, paramsMLX90621 *mlx90621);
float MLX90621_GetTa(uint16_t *frameData, const paramsMLX90621 *params);
void MLX90621_GetImage(uint16_t *frameData, const paramsMLX90621 *params, float *result);
void MLX90621_CalculateTo(uint16_t *frameData, const paramsMLX90621 *params, float emissivity, float tr, float *result);
int MLX90621_SetResolution(uint8_t resolution);
int MLX90621_GetCurResolution();
int MLX90621_SetRefreshRate(uint8_t refreshRate);   
int MLX90621_GetRefreshRate();  
void MLX90621_I2CInit(void);
int MLX90621_I2CReadEEPROM(uint8_t slaveAddr, uint8_t startAddress, int nMemAddressRead, uint8_t *data);
int MLX90621_I2CRead(uint8_t slaveAddr, uint8_t command, uint8_t startAddress, uint8_t addressStep, int nMemAddressRead, uint16_t *data);
int MLX90621_I2CWrite(uint8_t slaveAddr, uint8_t command, uint8_t checkValue, uint16_t data);