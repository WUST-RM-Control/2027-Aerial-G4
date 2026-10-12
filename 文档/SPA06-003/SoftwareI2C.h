#ifndef SOFTWARE_I2C_H
#define SOFTWARE_I2C_H

typedef signed   char   int8;     //!< Signed 8 bit integer
typedef unsigned char   uint8;    //!< Unsigned 8 bit integer

typedef signed   short  int16;    //!< Signed 16 bit integer
typedef unsigned short  uint16;   //!< Unsigned 16 bit integer

typedef signed   long   int32;    //!< Signed 32 bit integer
typedef unsigned long   uint32;   //!< Unsigned 32 bit integer

typedef unsigned char   bool;     //!< Boolean data type

void SoftI2C_start();
void SoftI2C_stop();

void SoftI2C_write_byte(uint8 value);
uint8 SoftI2C_read_byte(uint8 ack);


#endif
