#ifndef CommunicationUtils_h
#define CommunicationUtils_h

#include "Arduino.h"


void serialPrintFloatArr(float * arr, int length);
void serialFloatPrint(float f);
void writeArr(void * arr, uint8_t arr_length, uint8_t type_bytes);
void writeVar(void * val, uint8_t type_bytes);
int readRegister(int, byte);
void writeRegister(int, byte, byte);
void readFrom(byte address, int num, byte buff[]);

//byte _buff_comm[6] ;    //6 bytes buffer for saving data read from the device


#endif // CommunitationUtils_h
