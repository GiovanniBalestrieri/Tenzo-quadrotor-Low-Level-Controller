#include "CommunicationUtils.h"

#include <Wire.h>

void writeRegister(int deviceAddress, byte address, byte val) 
{
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address)
{
  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) 
  {
    // waiting
    // Serial.println("No Data");
  }

  v = Wire.read();
  return v;
}


// Reads num bytes starting from address register on device in to _buff array
void readFrom(int _dev_address, byte address, int num, byte buff[]) {
  Wire.beginTransmission(_dev_address); // start transmission to device
  Wire.write(address);             // sends address to read from
  Wire.endTransmission();         // end transmission

  Wire.beginTransmission(_dev_address); // start transmission to device
  Wire.requestFrom(_dev_address, num);    // request 6 bytes from device

  int i = 0;
  while(Wire.available())         // device may send less than requested (abnormal)
  {
    buff[i] = Wire.read();    // receive a byte
    i++;
  }

  Wire.endTransmission();         // end transmission
}


void serialPrintFloatArr(float * arr, int length) {
  for(int i=0; i<length; i++) {
    serialFloatPrint(arr[i]);
    Serial.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for(int i=0; i<4; i++) {
    
    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);
    
    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;
    
    Serial.print(c1);
    Serial.print(c2);
  }
}


void writeArr(void * varr, uint8_t arr_length, uint8_t type_bytes) {
  byte * arr = (byte*) varr;
  for(uint8_t i=0; i<arr_length; i++) {
    writeVar(&arr[i * type_bytes], type_bytes);
  }
}


// thanks to Francesco Ferrara and the Simplo project for the following code!
void writeVar(void * val, uint8_t type_bytes) {
  byte * addr=(byte *)(val);
  for(uint8_t i=0; i<type_bytes; i++) { 
    Serial.write(addr[i]);
  }
}


