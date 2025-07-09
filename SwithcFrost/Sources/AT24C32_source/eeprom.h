#ifndef i2ceeprom_h
#define i2ceeprom_h

//#define i2ceeprom32_I2C_ADDRESS 0x50 //default address

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif

class

 i2ceeprom {

  public:

    void i2c_eeprom_write_byte( byte deviceaddress, unsigned int eeaddress, byte data );

    void i2c_eeprom_write_block( byte deviceaddress, unsigned int eeaddresspage, void* data, unsigned int length );

    byte i2c_eeprom_read_byte( byte deviceaddress, unsigned int eeaddress );

    void i2c_eeprom_read_buffer( byte deviceaddress, unsigned int eeaddress, void *buffer, byte length );

};

#endif
