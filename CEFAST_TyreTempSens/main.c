

#include <stm32f10x.h>                 // Device header
#include <Delay_F103.h>
#include <I2C_F103.h>
#include <MLX90621.h>


static uint8_t eeData[256] = {0};
static uint16_t frame[66] = {0};
static paramsMLX90621 sensor;
int status = 0;
float Ta = 0.0;
float Tr = 36.0;
float To[64] = {0};
	


void GPIO_Config (void){
		RCC->APB2ENR = (1<<4); //Habilita o clock para a porta C
		GPIOC->CRH |= (1<<20); //PC13 Output Mode 10MHz
}


int main(void){

	
	SystemInit();
	GPIO_Config();
	TIM2_Config();
	MLX90621_I2CInit();
	Delay_ms(5);
	status = MLX90621_DumpEE(eeData);
	status = MLX90621_Configure(eeData);
	status = MLX90621_SetRefreshRate(0x0A);
	status = MLX90621_ExtractParameters(eeData,&sensor);
	status = MLX90621_DumpEE(eeData);
	while(1){
		status = MLX90621_GetFrameData(frame);
		Ta = MLX90621_GetTa(frame,&sensor);
		MLX90621_CalculateTo(frame,&sensor,0.98,Ta,To);
		GPIOC->BSRR |= (1<<13); //Seta PC13
		Delay_ms(100);
		GPIOC->BSRR |= (1<<29); //Reseta PC13
		Delay_ms(100);
	}
	
}