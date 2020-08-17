/*
 * I2C.cpp
 *
 *  Created on: Aug 3, 2020
 *      Author: MCLEANS
 */

#include "I2C.h"

namespace custom_libraries {

I2C_::I2C_(I2C_TypeDef *_I2C,
			GPIO_TypeDef *GPIO,
			uint8_t SDA,
			uint8_t SCL,
			Mode mode): _I2C(_I2C),
						GPIO(GPIO),
						SDA(SDA),
						SCL(SCL),
						mode(mode){
	//INITIALIZE I2C

	//INITIALIZE I2C RCC ENABLE BITS TO OFF
	if(_I2C == I2C1) RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
	if(_I2C == I2C2) RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
	if(_I2C == I2C3) RCC->APB1ENR &= ~RCC_APB1ENR_I2C3EN;

	//INITIALIZE I2C CONTROL REGISTERS TO 0X00
	_I2C->CR2 = 0x00;
	_I2C->CR1 = 0x00;

	//ENABLE I2C RCC
	if(_I2C == I2C1) RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	if(_I2C == I2C2) RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	if(_I2C == I2C3) RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

	//ENABLE GPIO RCC
	if(GPIO == GPIOA) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	if(GPIO == GPIOB) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	if(GPIO == GPIOC) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
	if(GPIO == GPIOD) RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	if(GPIO == GPIOE) RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;



	///SET I2C PINS TO ALTERNATE OPEN DRAIN AND ENABLE PULL_UP
	GPIO->MODER &= ~(1<<(SCL*2));
	GPIO->MODER |= (1<<((SCL*2)+1));

	GPIO->MODER &= ~(1<<(SDA*2));
	GPIO->MODER |= (1<<((SDA*2)+1));

	//SET TO OUTPUT OPEN DRAIN
	GPIO->OTYPER |= (1<<SCL);
	GPIO->OTYPER |= (1<<SDA);

	//ENABLE PULLUP
	GPIO->PUPDR |= (1<<(SCL*2));
	GPIO->PUPDR &= (1<<((SCL*2)+1));

	GPIO->PUPDR |= (1<<(SDA*2));
	GPIO->PUPDR &= (1<<((SDA*2)+1));

	//ENABLE ACTUAL ALTERNATE FUNCTION
	if(SDA < 8){
		GPIO->AFR[0] |= (4 << (SDA*4));
	}
	else{
		GPIO->AFR[1] |= (4 << (SDA*4));
	}

	if(SCL < 8){
		GPIO->AFR[0] |= (4 << (SDA*4));
	}
	else{
		GPIO->AFR[1] |= (4 << (SDA*4));
	}

	//SET PERIPHERAL CLOCK REQUENCY FOR I2C (I2C IS ON APB1 AT 36MHz)
	//CHANGE THIS TO THE CLOCK FREQUENCY OF YOUR APB1 BUS
	_I2C->CR2 |= APB1_FREQ;

	//SET CLOCK CONTROL REGISTER IN Fm/Sm
	int I2C_FREQUENCY = 0;
	if(mode == standard) I2C_FREQUENCY = 100;
	if(mode == fast) I2C_FREQUENCY = 400;

	float I2C_PERIOD = (1/I2C_FREQUENCY);
	double APB_BUS_PERIOD = (1/APB1_FREQ);

	int CCR_VALUE = ((I2C_PERIOD/2)/APB_BUS_PERIOD);
	_I2C->CCR |= CCR_VALUE;

	//SET I2C RISE TIME
	signed long  RISE_TIME =(((1/1000000)/APB_BUS_PERIOD)+1);
	_I2C->TRISE = RISE_TIME;

	//ENABLE THE PERIPHERAL, MUST BE DONE LAST
	_I2C->CR1 |= I2C_CR1_PE;

}

void I2C_::read_bytes(uint8_t address,uint8_t *buffer,uint8_t len){
		uint32_t temp = 0;
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		//ENABLE I2C DMA
		_I2C->CR2 |= I2C_CR2_DMAEN;
		//ENABLE ACKS
		_I2C->CR1 |= I2C_CR1_ACK;
		_I2C->CR2 |= I2C_CR2_LAST;
		//CONFIGURE DMA
		if(_I2C == I2C1){
			DMA1_Channel7 ->CMAR = (uint32_t) buffer;
			DMA1_Channel7 -> CPAR = (uint32_t) &I2C1->DR;
			DMA1_Channel7->CNDTR = len;
			DMA1_Channel7 ->CCR |= DMA_CCR7_TCIE | DMA_CCR7_MINC | DMA_CCR7_EN | DMA_CCR7_CIRC;
		}

		if(_I2C == I2C2){
			DMA1_Channel5 ->CMAR = (uint32_t) buffer;
			DMA1_Channel5 -> CPAR = (uint32_t) &I2C1->DR;
			DMA1_Channel5->CNDTR = len;
			DMA1_Channel5 ->CCR |= DMA_CCR5_TCIE | DMA_CCR5_MINC | DMA_CCR5_EN | DMA_CCR5_CIRC;
		}

		//SEND START BIT
		_I2C->CR1 |= I2C_CR1_START;
		//WAIT FOR START BIT TO BE SENT
		while(!(_I2C->SR1 & I2C_SR1_SB)){}
		//SEND DEVICE ADDRESS
		_I2C->DR = address+1;
		//WAIT FOR ADDRESS TO BE SENT
		while(!(_I2C->SR1 & I2C_SR1_ADDR)){}
		temp = I2C1->SR2;

		if(_I2C == I2C1){
			//WAIT UNTIL DMA TRANFER IS COMPLETE
			while((DMA1->ISR & DMA_ISR_TCIF7) == 0){}
			//CLEAR TRANSFER COMPLETE FLAG
			DMA1->IFCR |= DMA_IFCR_CTCIF7;
		}
		if(_I2C == I2C2){
			//WAIT UNTIL DMA TRANFER IS COMPLETE
			while((DMA1->ISR & DMA_ISR_TCIF5) == 0){}
			//CLEAR TRANSFER COMPLETE FLAG
			DMA1->IFCR |= DMA_IFCR_CTCIF5;
		}

		//SEND STOP BIT
		_I2C->CR1 |= I2C_CR1_STOP;
}

void I2C_::write_byte(uint8_t address,uint8_t mem,uint8_t data){
	uint32_t temp;
	//GENERATE START CONDITION
	_I2C->CR1 |= I2C_CR1_START;
	//WAIT UNTIL START BIT IS SENT
	while(!(_I2C->SR1 & I2C_SR1_SB)){};
	//SEND ADDRESS
	_I2C->DR = address;
	//WAIT UNTIL ADDRESS IS SENT
	while(!(_I2C->SR1 & I2C_SR1_ADDR)){}
	//READ SR1 AND SR2 TO CLEAR THE BIT
	temp = _I2C->SR2;
	//ADDRESS TO WRITE TO
	_I2C->DR = mem;
	//WAIT FOR TRANSFER TO COMPLETE
	while(!(_I2C->SR1 & I2C_SR1_TXE)){}
	//DATA TO WRITE IN ADDRESS
	_I2C->DR = data;
	//WAIT FOR TRANSFER TO COMPLETE
	while(!(_I2C->SR1 & I2C_SR1_TXE)){}
	//SET STOP BIT
	_I2C->CR1 |= I2C_CR1_STOP;
}

void I2C_::write_bytes(uint8_t address,uint8_t mem,uint8_t *data,uint16_t len){
	uint32_t temp;
	//GENERATE START CONDITION
	_I2C->CR1 |= I2C_CR1_START;
	//WAIT UNTIL START BIT IS SENT
	while(!(_I2C->SR1 & I2C_SR1_SB)){};
	//SEND ADDRESS
	_I2C->DR = address;
	//WAIT UNTIL ADDRESS IS SENT
	while(!(_I2C->SR1 & I2C_SR1_ADDR)){}
	//READ SR1 AND SR2 TO CLEAR THE BIT
	temp = _I2C->SR2;
	//ADDRESS TO WRITE TO
	_I2C->DR = mem;
	//WAIT FOR TRANSFER TO COMPLETE
	while(!(_I2C->SR1 & I2C_SR1_TXE)){}
	for(int i = 0; i < len; i++, *data++){
		//DATA TO WRITE IN ADDRESS
		_I2C->DR = *data;
		//WAIT FOR TRANSFER TO COMPLETE
		while(!(_I2C->SR1 & I2C_SR1_TXE)){}
	}

	//SET STOP BIT
	_I2C->CR1 |= I2C_CR1_STOP;
}

void I2C_::write_memp(uint8_t address,uint8_t mem){
	uint32_t temp;
	//GENERATE START CONDITION
	_I2C->CR1 |= I2C_CR1_START;
	//WAIT UNTIL START BIT IS SENT
	while(!(_I2C->SR1 & I2C_SR1_SB)){};
	//SEND ADDRESS
	_I2C->DR = address;
	//WAIT UNTIL ADDRESS IS SENT
	while(!(_I2C->SR1 & I2C_SR1_ADDR)){}
	//READ SR1 AND SR2 TO CLEAR THE BIT
	temp = _I2C->SR2;
	//ADDRESS TO WRITE TO
	_I2C->DR = mem;
	//WAIT FOR TRANSFER TO COMPLETE
	while(!(_I2C->SR1 & I2C_SR1_TXE)){}
	//SET STOP BIT
	_I2C->CR1 |= I2C_CR1_STOP;

}

uint8_t I2C_::BCD_to_decimal(uint8_t val){
	return((val /16 * 10) + (val %16));
}

uint8_t I2C_::decimal_to_BCD(uint8_t val){
	return((val /10 * 16) + (val %10));
}

I2C_::~I2C_() {
	// TODO Auto-generated destructor stub
}

} /* namespace custom_libraries */
