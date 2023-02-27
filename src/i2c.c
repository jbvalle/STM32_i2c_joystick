#include "../inc/peripherals.h"
#include "../inc/i2c.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void initialise_monitor_handles(void);
char UART_read(void);

void I2C1_init(void){

    /*Indication LED*/

    //Enable Clock to GPIOA Peripheral
    RCC->RCC_AHB1ENR |= 1;
    // Reset MODER Bitfield of PA5
    GPIOA->GPIOx_MODER &= ~(3 << (pin5 * MODER));
    // Write Value 1 to MODER Bitfield PA5 to configure PA5 as OUTPUT
    GPIOA->GPIOx_MODER |=  (1 << (pin5 * MODER));
    RCC->RCC_AHB1ENR |= 2;                  //Enable GPIOB Clock
    RCC->RCC_APB1ENR |= (1 << 21);          //Enable I2C Clock
                                            //
    /**************************************/


    /* Configure PB8, PB9 */
    GPIOB->GPIOx_AFRH &= ~0xFF;             //Set Alternate Function AF04 for PB8, PB9
    GPIOB->GPIOx_AFRH |= 0x44;                                        
    GPIOB->GPIOx_MODER &= ~(3 << (2 * 8));  //Set MODER to use AF
    GPIOB->GPIOx_MODER &= ~(3 << (2 * 9));
    GPIOB->GPIOx_MODER |= (2 << (2 * 8));
    GPIOB->GPIOx_MODER |= (2 << (2 * 9));

    GPIOB->GPIOx_OTYPER |= (1 << 8);        //Set PB8,9 to OpenDrain
    GPIOB->GPIOx_OTYPER |= (1 << 9);        
    GPIOB->GPIOx_PUPDR &= ~(0xF << 16);     //Set PB8,9 to use Pullup resistors
    GPIOB->GPIOx_PUPDR |=  (1 << (2 * 8));     
    GPIOB->GPIOx_PUPDR |=  (1 << (2 * 9));     

    I2C1->I2C_CR1 = 0x8000;                 //Software Reset
    I2C1->I2C_CR1 &= ~0x8000;               //deactivate RESET
    I2C1->I2C_CR2 = 0x0010;                 //Set FREQ to 16MHz
    I2C1->I2C_CCR = 80;                     //Set Standard Mode 100kHz
    I2C1->I2C_TRISE = 17;                   //Set maximum rise time
    I2C1->I2C_CR1 |= 0x0001;                //enable I2C1 Module
}

int I2C1_byte_write(char slave_addr, char mem_addr, uint8_t data){

    volatile int tmp;

    while(I2C1->I2C_SR2 & (1 << SR2_BUSY));               //Wait until bus is not busy

    I2C1->I2C_CR1 |= (1 << CR1_START);                 //Generate Start Signal

    while(!(I2C1->I2C_SR1 & (1 << SR1_SB)));            //Wait until START is set
                                                        //
    I2C1->I2C_DR = slave_addr << 1;                     //Send SLA + WRITE
    while(!(I2C1->I2C_SR1 & (1 << SR1_ADDR)));          //Wait until ADDR matched
    tmp = I2C1->I2C_SR2;                                //Clear ADDR Bit

    while(!(I2C1->I2C_SR1 & (1 << SR1_TxE)));           //Wait until Tx Register Empty
    I2C1->I2C_DR = mem_addr;                            //Send Memory Address
                                                        //
    while(!(I2C1->I2C_SR1 & (1 << SR1_TxE)));           //Wait until Tx Register Empty
    I2C1->I2C_DR = data;

    while(!(I2C1->I2C_SR1 & (1 << SR1_BTF)));           //Wait until all the Transmit Bytes have been sent
    I2C1->I2C_CR1 |= (1 << CR1_STOP);                   //Generate STOP Signal
    return 0;
}

int I2C1_byte_read(char slave_addr, char mem_addr, uint8_t *data){

    volatile int tmp;

    while(I2C1->I2C_SR2 & (1 << SR2_BUSY));               //Wait until bus is not busy

    I2C1->I2C_CR1 |= (1 << CR1_START);                 //Generate Start Signal

    while(!(I2C1->I2C_SR1 & (1 << SR1_SB)));            //Wait until START is set

    I2C1->I2C_DR = slave_addr << 1;         //Transmit SLA+W
    while(!(I2C1->I2C_SR1 & (1 << SR1_ADDR)));            //Wait for ACK
    tmp = I2C1->I2C_SR2;

    while(!(I2C1->I2C_SR1 & (1 << SR1_TxE)));         //Wait for TxNE is set
    I2C1->I2C_DR = mem_addr;                //SEND MEMORY ADDRESS
    while(!(I2C1->I2C_SR1 & (1 << SR1_TxE)));         //Wait for TxNE is set

    I2C1->I2C_CR1 |= (1 << CR1_START);                 //Generate RESTART
    while(!(I2C1->I2C_SR1 & 1));            //Wait until START is set

    I2C1->I2C_DR = slave_addr << 1 | 1;     //Transmit SLA+R
    while(!(I2C1->I2C_SR1 & (1 << SR1_ADDR)));            //Check if ACK recieved
    I2C1->I2C_CR1 &= ~(1 << CR1_ACK);                //Disable ACK from MASTER
    tmp = I2C1->I2C_SR2;                    //RESET ADDR FLAG by reading SR2

    I2C1->I2C_CR1 |= (1 << CR1_STOP);                 //Generate STOP

    while(!(I2C1->I2C_SR1 & (1 << SR1_RxNE)));         //Wait until RxNE flag is set
    *data = I2C1->I2C_DR;                   //Read Data

    return 0;
}


void configure_i2c_pullup(void){

    RCC->RCC_AHB1ENR |= 1;

    GPIOA->GPIOx_MODER &= ~(3 << (8 * 2));      //Set PA8,9 as OUTPUT
    GPIOA->GPIOx_MODER &= ~(3 << (9 * 2));
    GPIOA->GPIOx_MODER |= (1 << (8 * 2));
    GPIOA->GPIOx_MODER |= (1 << (9 * 2));

    GPIOA->GPIOx_PUPDR &= ~(3 << (8 * 2));      //Set PULLUP 
    GPIOA->GPIOx_PUPDR &= ~(3 << (9 * 2));
    GPIOA->GPIOx_PUPDR |=  (1 << (8 * 2));
    GPIOA->GPIOx_PUPDR |=  (1 << (9 * 2));

    GPIOA->GPIOx_OTYPER |= (1 << 8);            //Set PA8,9 as Opendrain 
    GPIOA->GPIOx_OTYPER |= (1 << 9);

    GPIOA->GPIOx_ODR |= (1 << 8);               //Set PA8,9 to HIGH
    GPIOA->GPIOx_ODR |= (1 << 9);
}

void checkINT_init(void){
    RCC->RCC_AHB1ENR |= (1 << 1);

    GPIOB->GPIOx_MODER &= ~(3 << (10 * 2));     //SET PB10 as INPUT
}

void checkINT(uint8_t *x_coord, uint8_t *y_coord){


    if(!(GPIOB->GPIOx_IDR & (1 << 10))){

        GPIOA->GPIOx_ODR ^= (1 << pin5);

        I2C1_byte_read(0x40, 0x10, x_coord);
        I2C1_byte_read(0x40, 0x11, y_coord);

        printf("\n\rX: %d, Y: %d", *x_coord, *y_coord);

    }
}

int determineTimebase(int *timebase_){

    int* timebase = timebase_;

    char input[10], letter;

    puts("\n\rWould you like to enter Low Power Mode? (y/n)\n\r");
    for(int i = 0; i < 10; i++) input[i++] = '\0';
    for(int i = 0; (letter = UART_read()) != '\r'; ) input[i++] = letter;

    if(strcmp(input, "y") == 0){

        puts("\n\rChoose a timebase:\n\r");
        puts("\n\r 20ms | 0");
        puts("\r 40ms | 1");
        puts("\r 80ms | 2");
        puts("\r100ms | 3");
        puts("\r140ms | 4");
        puts("\r200ms | 5");
        puts("\r260ms | 6");
        puts("\r320ms | 7\n\r");

        for(int i = 0; i < 10; i++) input[i++] = '\0';
        for(int i = 0; (letter = UART_read()) != '\r'; ) input[i++] = letter;
        sscanf(input, "%d", timebase);
        printf("\n\rYou chose timebase %d", *timebase);

        return LOWPOWERMODE;
    }

    return IDLEMODE;
}
