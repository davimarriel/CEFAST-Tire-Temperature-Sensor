#include <I2C_F103.h>


void I2C_Config(void){
	
	
	//Enable I2C Module and GPIO Clocks
	RCC->APB2ENR |= (1<<3); //Enable GPIOB Clock (I2C2: PB10||PB11)
	RCC->APB2ENR |= (1<<0); //Enable AFIO Clock
	//Configure I2C Pins 
	GPIOB->CRH |= (3<<12) | (3<<8); //Set PB10 and PB11 to Output Mode with 10MHz max speed
	
	RCC->APB1ENR |= (1<<22); //Enable I2C2 Clock 
	
	//Reset the I2C
	//I2C2->CR1 |= (1<<15); //Put the I2C under reset state
	//I2C2->CR1 &= ~(1<<15);
	//RCC->APB1RSTR &= (1<<22);
	
	
	
	////Enable the I2C
	//I2C2->CR1 |= (1<<0);
	
	//Errata config
	I2C2->CR1 &= ~(1<<0); //Disable I2C
	GPIOB->ODR |= (1<<10) | (1<<11); // Set Outputs to High
	while(!(GPIOB->IDR & (3<<10))); // Verify High Level
	GPIOB->ODR &= ~(1<<11); // Set SDA to Low
	while((GPIOB->IDR & (1<<11))); // Verify Low Level
	GPIOB->ODR &= ~(1<<10); // Set SCL to Low
	while((GPIOB->IDR & (1<<10))); // Verify Low Level
	GPIOB->ODR |= (1<<10); // Set SCL to High
	while(!(GPIOB->IDR & (1<<10))); // Verify High Level
	GPIOB->ODR |= (1<<11); // Set SDA to Low
	while(!(GPIOB->IDR & (1<<11))); // Verify High Level
	GPIOB->CRH |= (3<<10);
	GPIOB->CRH |= (3<<14); //Set Alternate function output Open-Drain
	
	//Reset the I2C
	I2C2->CR1 |= (1<<15); //Put the I2C under reset state
	I2C2->CR1 &= ~(1<<15);
	
	I2C2->CR1 |= (1<<10); //Set the ACK bit
	
	//Peripheral Clock Frequency
	I2C2->CR2 = 8; 
	
	//Clock control Register
	I2C2->CCR = 40;
	
	//Configure TRISE
	I2C2->TRISE = 37;
	
	//Enable the I2C
	I2C2->CR1 |= (1<<0);
}

void I2C_Start(void){
	I2C2->CR1 &= ~(1<<9); //Clear the stop condition
	I2C2->CR1 |= (1<<10); //Set the ACK bit
	I2C2->CR1 |= (1<<8); //Start/Repeated Start Generation
	while(!(I2C2->SR1 & (1<<0))); //Wait until SB is set
}


void I2C_Address(uint8_t address){
	I2C2->DR = address; //Set the address of a transmission
	while(!(I2C2->SR1 & (1<<1))); //Wait until ADDR is set
	uint8_t temp = I2C2->SR1 | I2C2->SR2; //Clear ADDR bit
}


void I2C_Write(uint8_t data){
	while(!(I2C2->SR1 &(1<<7))); //Wait until TxE is sett
	I2C2->DR = (volatile uint32_t) data; //Send data
	while(!(I2C2->SR1 &(1<<2))); //Wait until BTF is set
}

void I2C_WriteBuf(uint8_t *data,int size){
	while(!(I2C2->SR1 &(1<<7))); //Wait until TxE is set
	while(size){
		while(!(I2C2->SR1 &(1<<7))); //Wait until TxE is set
		I2C2->DR = (volatile uint32_t) *data++; //Send data
		size--;
	}
	while(!(I2C2->SR1 &(1<<2))); //Wait until BTF is set
}

void I2C_Read(uint8_t addr, uint8_t *data, int size){
	int remaining = size;
	if(size == 1){
		I2C2->DR = addr; //Set the address of a transmission
		while(!(I2C2->SR1 & (1<<1))); //Wait until ADDR is set
		I2C2->CR1 &= ~(1<<10); //Clear ACK bit;
		uint8_t temp = I2C2->SR1 | I2C2->SR2; //Clear ADDR bit
		I2C2->CR1 |= (1<<9); //STOP bit
		
		while(!(I2C2->SR1 & (1<<6))); //Wait until RxNE is set
		
		data[size-remaining] = I2C2->DR; //Read DR
	}
	else{
		I2C2->DR = addr; //Set the address of a transmission
		while(!(I2C2->SR1 & (1<<1))); //Wait until ADDR is set
		uint8_t temp = I2C2->SR1 | I2C2->SR2; //Clear ADDR bit
		while(remaining>2){
			while(!(I2C2->SR1 & (1<<6))); //Wait until RxNE is set
			data[size-remaining] = I2C2->DR; //Copy the data into buffer
			I2C2->CR1 |= (1<<10); //Set the ACK bit
			remaining--;
		}
		while(!(I2C2->SR1 & (1<<6))); //Wait until RxNE is set
		data[size-remaining] =  I2C2->DR;
		I2C2->CR1 &= ~(1<<10); //Clear ACK bit;
		I2C2->CR1 |= (1<<9); //STOP bit
		remaining--;
		while(!(I2C2->SR1 & (1<<6))); //Wait until RxNE is set
		data[size-remaining] = I2C2->DR;
	}
}

void I2C_Stop(void){
	I2C2->CR1 |= (1<<9); //STOP Generation
	while((I2C2->SR2 & (1<<2))); //Wait until BUSY is reset
}