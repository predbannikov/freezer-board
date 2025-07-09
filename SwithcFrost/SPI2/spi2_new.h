#ifndef SPI2_NEW_H
#define SPI2_NEW_H

#include "stdint.h"
#define CS_PB2_L spi2_csPB2_low();
#define CS_PB2_H spi2_csPB2_high();

#define CS_PB12_L spi2_cs_low();
#define CS_PB12_H spi2_cs_high();


#define CS_PB9_L spi2_csPB9_low();
#define CS_PB9_H spi2_csPB9_high();

#define CS_PB11_L spi2_csPB11_low();
#define CS_PB11_H spi2_csPB11_high();


void spi1_init(void);
void spi2_init(void);
void spi1_transmit(uint8_t *data,uint32_t size);
void spi1_cs_low();
void spi1_cs_high();

void spi2_transmit(uint8_t *data,uint8_t size);
void spi2_cs_low();
void spi2_cs_high();
void spi2_csPB2_low();
void spi2_csPB2_high();

uint8_t spi_read(uint8_t reg, uint8_t *buf, uint16_t len); 
uint8_t spi_write(uint8_t reg, uint8_t *buf, uint16_t len);    

#endif /* SPI2_NEW_H_ */
