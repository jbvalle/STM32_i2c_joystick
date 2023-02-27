#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define LOWPOWERMODE 0
#define IDLEMODE     1

#define TIMESPACE 4
#define MODER 2
#define pin5 5

#define CR1_SWRST       15
#define CR1_ACK         10
#define CR1_STOP         9
#define CR1_START        8
#define CR1_PE           0

#define SR1_TxE         7
#define SR1_RxNE        6
#define SR1_BTF         2
#define SR1_ADDR        1
#define SR1_SB          0

#define SR2_BUSY        1


void I2C1_init(void);
int I2C1_byte_read(char slave_addr, char mem_addr, uint8_t *data);
int I2C1_byte_write(char slave_addr, char mem_addr, uint8_t data);

void configure_i2c_pullup(void);
void checkINT_init(void);
void checkINT(uint8_t *x_coord, uint8_t *y_coord);
int determineTimebase(int *timebase_);
#endif
