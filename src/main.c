#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "../inc/peripherals.h"
#include "../inc/i2c.h"

RCC_t   * const RCC     = (RCC_t    *)  0x40023800;
GPIOx_t * const GPIOA   = (GPIOx_t  *)  0x40020000;
GPIOx_t * const GPIOB   = (GPIOx_t  *)  (0x40020000 + 0x400);
USART_t * const USART2  = (USART_t  *)  0x40004400;
NVIC_t  * const NVIC    = (NVIC_t   *)  0xE000E100;
I2C_t   * const I2C1    = (I2C_t    *)  0x40005400;

void initialise_monitor_handles(void);
char UART_read(void);


void wait_ms(int time){
    for(int i = 0; i < time; i++){
        for(int j = 0; j < 1600; j++);
    }
}



int main(void){

    initialise_monitor_handles();

    // I2CAGC
    uint8_t x_coord;
    uint8_t y_coord;
    uint8_t data = 0x0;
    int timebase;
    uint8_t config_data = 0x00;

    configure_i2c_pullup();
    I2C1_init();


    //Check if reset is done correctly
    while(!(data & 1)){

        I2C1_byte_read(0x40, 0x0F, &data);
    }

    // Send Test Bits to CR
    I2C1_byte_write(0x40, 0x2E, 0x86);
    // Set Maximum Sensitivity
    I2C1_byte_write(0x40, 0x2A, 0x3F);
    // Set Scaling factor to 90%
    I2C1_byte_write(0x40, 0x2D, 10);

    if(determineTimebase(&timebase) == LOWPOWERMODE){

        config_data &= ~(1 << 8);

    }else{

        timebase = 0;
        config_data |= (1 << 8);
    }

    // Set CR : IDLE, NO INT
    I2C1_byte_write(0x40, 0x0F, (config_data | (timebase << 4)));

    checkINT_init();

    for(;;){

        checkINT(&x_coord, &y_coord);
    }
}
