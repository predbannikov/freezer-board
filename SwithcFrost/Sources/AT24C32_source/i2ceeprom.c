#include <i2ceeprom.h>
02
#include <Wire.h> //I2C library // A4,A5
03
 
04
// i2ceeprom32 provides 32,768 bits 0x50
05
 
06
  void i2ceeprom::i2c_eeprom_write_byte( byte deviceaddress, unsigned int eeaddress, byte data ) {
07
    byte rdata = data;
08
    Wire.beginTransmission(deviceaddress);
09
    Wire.write((int)(eeaddress >> 8)); // MSB
10
    Wire.write((int)(eeaddress & 0xFF)); // LSB
11
    Wire.write(rdata);
12
    Wire.endTransmission();
13
  }
14
 
15
 
16
  void i2ceeprom::i2c_eeprom_write_block( byte deviceaddress, unsigned int eeaddress, void* data, unsigned int length ) {
17
    const byte* current = reinterpret_cast<const byte*>(data);
18
    unsigned int c;
19
    for ( c = 0; c < length; c++)
20
    {
21
    Wire.beginTransmission(deviceaddress);
22
    Wire.write((int)(eeaddress >> 8)); // MSB
23
    Wire.write((int)(eeaddress & 0xFF)); // LSB
24
    Wire.write(*current++);
25
    Wire.endTransmission();
26
    eeaddress++;delay(20);
27
    }
28
  }
29
 
30
  byte i2ceeprom::i2c_eeprom_read_byte( byte deviceaddress, unsigned int eeaddress ) {
31
    byte rdata = 0xFF;
32
    Wire.beginTransmission(deviceaddress);
33
    Wire.write((int)(eeaddress >> 8)); // MSB
34
    Wire.write((int)(eeaddress & 0xFF)); // LSB
35
    Wire.endTransmission();
36
    Wire.requestFrom(deviceaddress,(byte)1);
37
    if (Wire.available()) rdata = Wire.read();
38
    return rdata;
39
  }
40
 
41
  // maybe let's not read more than 30 or 32 bytes at a time!
42
    void i2ceeprom::i2c_eeprom_read_buffer( byte deviceaddress, unsigned int eeaddress, void *buffer, byte length ) {
43
    byte* current = reinterpret_cast<byte*>(buffer);
44
    Wire.beginTransmission(deviceaddress);
45
    Wire.write((int)(eeaddress >> 8)); // MSB
46
    Wire.write((int)(eeaddress & 0xFF)); // LSB
47
    Wire.endTransmission();
48
    Wire.requestFrom(deviceaddress,(byte)length);
49
    int c = 0;
50
    for ( c = 0; c < length; c++ )
51
    if (Wire.available())  *current++ = Wire.read();
52
 }
//-------------------------------------------------------------------------------------
 
 
 
 
 
 
 
   void setup()
05
  {
06
    char somedata[] = "this is data from the eeprom"; // data to write ABCDEFGHIJKLMNOPQRSTUVWXYZ
07
    Wire.begin(); // initialise the connection
08
    Serial.begin(38400);
09
    i2c_eeprom_write_block(0x50, 0, (byte *)somedata, sizeof(somedata)); // write to EEPROM
10
 
11
    delay(10); //add a small delay
12
 
13
    Serial.println("Memory written");
14
  }
15
 
16
  void loop()
17
  {
18
    int addr=0; //first address
19
    byte b = i2c_eeprom_read_byte(0x50, 0); // access the first address from the memory
20
 
21
    while (b!=0)
22
    {
23
      Serial.print((char)b); //print content to serial port
24
      addr++; //increase address
25
      b = i2c_eeprom_read_byte(0x50, addr); //access an address from the memory
26
    }
27
    Serial.println(" ");
28
    delay(200000);
29
  }

 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 