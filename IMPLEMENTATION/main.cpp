#include "stm32f4xx.h"
#include "clockconfig.h"
#include "I2C.h"

custom_libraries::clock_config system_clock;

int main(void) {
  system_clock.initialize();
  custom_libraries::I2C_ i2c(I2C2,GPIOB,11,10,custom_libraries::standard);
  i2c.write_byte(0xD0,0x00,0x06);
  uint8_t data[2];
  i2c.write_memp(0xD0,0x00);
  i2c.read_bytes(0xD0,data,2);

  while(1){
  
  }
}
