/*
  Inertial.h - Library to handle inertial sensors and estimates
  to configure Accelerometers, Gyroscopes, Magnetometers, Barometers, Gps
  Created by Giovanni Balestrieri - UserK, March 3, 2017.
  
  Reviewved: 
  - 29 June 2017

  Detailed explanation at
  www.userk.co.uk
*/
#ifndef Inertial_h
#define Inertial_h

#include "Arduino.h"
#include "MedianFilter.h"


//#define ADXL345 1
//#define L3G4200D 1

#define MPU6050 1

#include "FIMU_ADXL345.h"
#define FIMU_ACC_ADDR ADXL345_ADDR_ALT_LOW // SDO connected to GND
#include "FIMU_ITG3200.h"


#define FIMU_BMA180_DEF_ADDR BMA180_ADDRESS_SDO_LOW
#define FIMU_ITG3200_DEF_ADDR ITG3200_ADDR_AD0_LOW // AD0 connected to GND


extern int ACC_X_AXIS_PIN;
extern int ACC_Y_AXIS_PIN;
extern int ACC_Z_AXIS_PIN;
extern int gyroSensibility;
extern int gyroBiasSamples;
extern volatile boolean filterGyro;
extern volatile boolean filterAcc;
extern volatile float alphaA;
extern volatile float alphaW;
extern const float pi;
//extern const float k_compl_filter_acc;
//extern const float k_compl_filter_gyro;
extern unsigned long timerInertial1;
extern unsigned long timerInertial2;

#define XAXIS 0
#define YAXIS 1
#define ZAXIS 2


#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

// Gyro
#define L3G4200D_Address 105
#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

#define scale2000 70

#define STATUS_REG 0x27
#define ZOR_REG 0b01000000
#define ZDA_REG 0b00000100
#define YOR_REG 0b00100000
#define YDA_REG 0b00000010
#define XOR_REG 0b00010000
#define XDA_REG 0b00000001



class Inertial
{
  public:
    Inertial();
    void init(bool);

    void setupGyroscope();
    void setupAccelerometer();
    void setupMagnetometer();

    void calcGyroBias();
    void getMagnetometerValues();
    void requestMagnetometerData();
    
    void getValues(float * values);
    void getValues(float * values, boolean a, boolean g);
    void getRawValues(int * raw_values);

    void filterValuesFirstOrder(float alpha);
    void filterValuesSecondOrder(float alpha);

    float getRollEst();
    float getPitchEst();
    float getRollEstAcc();
    float getPitchEstAcc();
    float getYawEst();
    float getRollRaw();
    float getPitchRaw();
    float getYawRaw();
    void getYawPitchRoll(float *);

    // Accelerometer
    void getAcc(boolean);
    float getAccX();
    float getAccY();
    float getAccZ();
    float getAccXFilt();
    float getAccYFilt();
    float getAccZFilt();
    void estAngleFromAcc();

    // Gyroscope
    float getAngularVel(int axis);    
    void getGyroValues(int * values);
    void getAngularVel();    
    void setupL3G4200D(int);
    boolean getGyroCalibrationStatus();
    void setGyroCalibrationStatus(boolean statusGyroCalib);
    void removeBiasAndScaleGyroData();
    void removeBiasAndScale();

    
    MedianFilter medianGyroX;
    MedianFilter medianGyroY;
    MedianFilter medianGyroZ;

    void wFilter(float val[]);
    void accFilter(float val[]);

    unsigned long dt;
    
    byte xMSB;
    byte xLSB;
    int xC;
    byte yMSB;
    byte yLSB;
    int yC;
    byte zMSB;
    byte zLSB;
    int zC;
    
    ADXL345 acc;
    ITG3200 gyro;
  
  
   
  private:
    float _phi;
    float _phiEst;
    float _phiAcc;
    float _psi;
    float _psiEst;
    float _psiAcc;
    float _theta;
    float _thetaEst;
    float _thetaAcc;
    float _wx;
    float _wy;
    float _wz;


    // gyro bias
    int _gyroBiasTempX;
    int _gyroBiasTempY;
    int _gyroBiasTempZ;
    int _gyroBiasX;
    int _gyroBiasY;
    int _gyroBiasZ;
    boolean _initializedGyroCalib;
    
    float _ax;
    float _ay;
    float _az;
    float _axRaw;
    float _ayRaw;
    float _azRaw;
    float _xPose;
    float _yPose;
    float _zPose;
    float _wF[3] = {0,0,0};
    float _aF[3] = {0,0,0};
    float _wFm1[3] = {0,0,0};
    float _aFm1[3] = {0,0,0};
    
    int _axPin;
    int _ayPin;
    int _azPin;
};
#endif
