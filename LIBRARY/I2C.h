/*
 * I2C.h
 *
 *  Created on: Aug 3, 2020
 *      Author: MCLEANS
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xx.h"

namespace custom_libraries {

#define APB1_FREQ 42
#define APB2_FREQ 84

enum Mode{
	standard, //100KHz
	fast //400KHz
};

class I2C_ {
private:
	I2C_TypeDef *_I2C;
	GPIO_TypeDef *GPIO;
	uint8_t SDA;
	uint8_t SCL;
	Mode mode;

private:
public:
public:
	I2C_(I2C_TypeDef *_I2C,
		GPIO_TypeDef *GPIO,
		uint8_t SDA,
		uint8_t SCL,
		Mode mode);
	void read_bytes(uint8_t address,uint8_t *buffer,uint8_t len);
	void write_byte(uint8_t address,uint8_t mem,uint8_t data);
	void write_bytes(uint8_t address,uint8_t mem,uint8_t *data,uint16_t len);
	void write_memp(uint8_t address,uint8_t mem);
	uint8_t BCD_to_decimal(uint8_t val);
	uint8_t decimal_to_BCD(uint8_t val);
	~I2C_();
};

} /* namespace custom_libraries */

#endif /* I2C_H_ */
