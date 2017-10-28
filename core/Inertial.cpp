/*
  Inertial.cpp - Class to handle inertial sensors and estimates
  to configure Accelerometers, Gyroscopes, Magnetometers, Barometers
  Created by Giovanni Balestrieri - UserK, March 3, 2017.
  
  Reviewved: 
  - 29 June 2017
  - 28 Oct 2017

  Detailed explanation at
  www.userk.co.uk
*/


#include "Inertial.h"
//#include "Ctx.h"
#include "CommunicationUtils.h"
#include "MedianFilter.h"

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



// 3.3 Fully loaded Tenzo V2.2
int xRawMin = 409;
int xRawMax = 278;
// 
int yRawMin = 404;
int yRawMax = 272;
// 
int zRawMin = 419;
int zRawMax = 288;


// Complementary Filter gains
const float k_compl_filter_acc = 0.025;
const float k_compl_filter_gyro = 0.975;

/*
 * Constructor: 
 *              Loads configuration values from ctx
 */
Inertial::Inertial(){
  // Load accelerometer pins
  #ifdef  ADXL345_ACC
    _axPin = ACC_X_AXIS_PIN;
    _ayPin = ACC_Y_AXIS_PIN;
    _azPin = ACC_Z_AXIS_PIN;
  #endif
  
  _gyroBiasTempX = 0;
  _gyroBiasTempY = 0;
  _gyroBiasTempZ = 0;
  _gyroBiasX = 0;
  _gyroBiasY = 0;
  _gyroBiasZ = 0;

  dt = 0;

  // Init MedianFilters
   MedianFilter medianGyroX(3,0);
   MedianFilter medianGyroY(3,0);
   MedianFilter medianGyroZ(3,0);
}

/*
 * Initialize sensors
 */
void Inertial::init( bool fastmode){
  delay(5);
  
  // disable internal pullups of the ATMEGA which Wire enable by default
  #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) || defined(__AVR_ATmega328P__)
    // deactivate internal pull-ups for twi
    // as per note from atmega8 manual pg167
    cbi(PORTC, 4);
    cbi(PORTC, 5);
  #else
    // deactivate internal pull-ups for twi
    // as per note from atmega128 manual pg204
    cbi(PORTD, 0);
    cbi(PORTD, 1);
  #endif
  
  if(fastmode) { // switch to 400KHz I2C
    TWBR = ((16000000L / 400000L) - 16) / 2; // see twi_init in Wire/utility/twi.c
  }
  
  // Init Accelerometer
  this->setupAccelerometer();

  // Init Gyroscope
  this->setupGyroscope();

  // Init Magnetometer
  this->setupMagnetometer();
}

/**
 * Initialize Magnetometer
 */
void Inertial::setupMagnetometer() {
  // CMPS10
  // No preconfig required  
}

/**
 * Initialize Gyroscope
 */
void Inertial::setupGyroscope() {

  #ifdef L3G4200D
    // Configure L3G4200 
    this->setupL3G4200D(gyroSensibility); 
    //wait for the sensor to be ready   
    delay(500); 
    
    // Compute gyro bias  
    this->calcGyroBias();
    Serial.println("[ Ok ] Gyro");
  #endif 

  
  #ifdef MPU6050
    gyro = ITG3200();  
    
    // init ITG3200
    gyro.init(FIMU_ITG3200_DEF_ADDR);  
 //   gyro.begin(0x69);
    
    delay(1000);
    // calibrate the ITG3200
    gyro.zeroCalibrate(128,5);
    Serial.println("[ Ok ] Gyro itg3200");
  #endif
}


/**
 * Initialize Accelerometer
 */
void Inertial::setupAccelerometer() {
  #ifdef ADXL345_ACC
    pinMode(ACC_X_AXIS_PIN,INPUT);
    pinMode(ACC_Y_AXIS_PIN,INPUT);
    pinMode(ACC_Z_AXIS_PIN,INPUT);
  #endif

  #ifdef MPU6050
    acc = ADXL345();
    // init ADXL345
    acc.init(FIMU_ACC_ADDR);
  #endif
    
  Serial.println("[ Ok ] Acc");
}

/**
 * Computes Gyro bias
 */
void Inertial::setupL3G4200D(int scale) {
  //From  Jim Lindblom of Sparkfun's code

    // Enable x, y, z and turn off power down:
  //writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);
  // 400Hz and 1Hz cutoff frq of the HPF
  //writeRegister(L3G4200D_Address, CTRL_REG1, 0b01011111);
  // 200Hz 
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b01001111);

  // If you'd like to adjust/use the HPF:
  // High pass filter cut off frecuency configuration
  // ODR 200Hz, cutoff 0.02Hz 1001
  //writeRegister(L3G4200D_Address, CTRL_REG2, 0b00001001);
  // ODR 200Hz, cutoff 1 Hz 0100
  //writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000100);
  // ODR 200Hz, cutoff 0.02Hz 1001
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00001001);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  //writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range:
  if(scale == 250)
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }
  else if(scale == 500)
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }
  else
  {
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00010000);
  //writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
  
}


/**
 * Returns phi, theta, psi (roll, pitch,yaw) estimates
 */
 void Inertial::getYawPitchRoll(float *angles) {

    dt = micros() - dt;
    // update phiAcc and thetaAcc
    this->estAngleFromAcc();

    // Estimate Angles 
    angles[0] = (angles[0]  + _wx*(float)dt/1000000.0)*k_compl_filter_gyro + _phiAcc*k_compl_filter_acc;
    angles[1] = (angles[1]  + _wy*(float)dt/1000000.0)*k_compl_filter_gyro + _thetaAcc*k_compl_filter_acc;  
    dt = micros();
 }

 /**
 * Returns phi, theta, psi (roll, pitch,yaw) estimates
 */
 void Inertial::estAngleFromAcc(){ 
  if (filterAcc)
  {
    _phiAcc = (atan2(-_aF[0],-_aF[2])) * RAD_TO_DEG;
    _thetaAcc = (atan2(-_aF[1],-_aF[2])) * RAD_TO_DEG;
  }
  else 
  {
    _phiAcc = (atan2(-_ax,-_az)) * RAD_TO_DEG;
    _thetaAcc = (atan2(-_ay,-_az)) * RAD_TO_DEG;
  }
 }




/**
 * Computes Gyro bias
 */
void Inertial::calcGyroBias() {
  
  for (int i = 0; i<gyroBiasSamples; i++)
  {
    delay(5);
    this->getAngularVel(); 
    _gyroBiasTempX = _gyroBiasTempX + _wx;
    _gyroBiasTempY = _gyroBiasTempY + _wy;
    _gyroBiasTempZ = _gyroBiasTempZ + _wz;
   
  }
  _gyroBiasX = _gyroBiasTempX / gyroBiasSamples;
  _gyroBiasY = _gyroBiasTempY / gyroBiasSamples;
  _gyroBiasZ = _gyroBiasTempZ / gyroBiasSamples;
  
  _initializedGyroCalib = true;
}

void Inertial::getAngularVel() {
  // Get Data if available
  byte statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  while(!(statusflag & ZDA_REG) && (statusflag & ZOR_REG)&&!(statusflag & YDA_REG) && (statusflag & YOR_REG)&& !(statusflag & XDA_REG) && (statusflag & XOR_REG)) 
  {
    statusflag = readRegister(L3G4200D_Address, STATUS_REG);
  }
  

  xMSB = readRegister(L3G4200D_Address, 0x29);
  xLSB = readRegister(L3G4200D_Address, 0x28);
  xC = ((xMSB << 8) | xLSB);
  //medianGyroX.in(xC);
  //_wx = medianGyroX.out();    
    _wx = xC;

  yMSB = readRegister(L3G4200D_Address, 0x2B);
  yLSB = readRegister(L3G4200D_Address, 0x2A);
  yC = ((yMSB << 8) | yLSB);
  //medianGyroY.in(yC);
  //_wy = medianGyroY.out();     
    _wy = yC;

  zMSB = readRegister(L3G4200D_Address, 0x2D);
  zLSB = readRegister(L3G4200D_Address, 0x2C);
  zC = ((zMSB << 8) | zLSB);
  //medianGyroZ.in(zC);
  //_wz = medianGyroZ.out();  
    _wz = zC;   
  
  if (_initializedGyroCalib)
    this->removeBiasAndScaleGyroData();
    
   if (filterGyro)
   {
      _wF[0] = _wy;
      _wF[1] = _wy;
      _wF[2] = _wz;
      wFilter(_wF);
   }

}

void Inertial::getGyroValues(int * values) { 
  gyro.readGyroRaw(&values[0], &values[1], &values[2]);
}

void Inertial::getRawValues(int * raw_values) {
  acc.readAccel(&raw_values[0], &raw_values[1], &raw_values[2]);
  gyro.readGyroRaw(&raw_values[3], &raw_values[4], &raw_values[5]);
  //magn.getValues(&raw_values[6], &raw_values[7], &raw_values[8]);
  
}

void Inertial::getValues(float * values) {  
  int accval[3];
  int omegaval[3];
  
  acc.readAccel(&accval[0], &accval[1], &accval[2]);
  gyro.readGyroRawCal(&omegaval[0],&omegaval[1],&omegaval[2]);
   
  _axRaw = ((float) accval[0]);
  _ayRaw = ((float) accval[1]);
  _azRaw = ((float) accval[2]);
  
  _wx = ((float) omegaval[0]);
  _wy = ((float) omegaval[1]);
  _wz = ((float) omegaval[2]);

  
  _wx =  _wx / (float) (14.375);
  _wy =  _wy / (float) (14.375);
  _wz =  _wz / (float) (14.375);

  values[0] = ((float) accval[0]);
  values[1] = ((float) accval[1]);
  values[2] = ((float) accval[2]);
  values[3] = ((float) _wx);
  values[4] = ((float) _wy);
  values[5] = ((float) _wz);
  
  //magn.getValues(&values[6]);
}

void Inertial::getValues(float * values, boolean accFilt, boolean gyroFilt) {  
  int accval[3];
  float accVal[3];
  int omegaval[3];
  
  acc.get_Gxyz(accVal);
  
  //acc.readAccel(&accval[0], &accval[1], &accval[2]);
  gyro.readGyroRawCal(&omegaval[0],&omegaval[1],&omegaval[2]);
   
  /*
  _ax = ((float) accval[0]);
  _ay = ((float) accval[1]);
  _az = ((float) accval[2]);
  */
  
  _ax = accVal[0];
  _ay = accVal[1];
  _az = accVal[2];

  if (accFilt) {
    _aF[0] = _ax;
    _aF[1] = _ay;
    _aF[2] = _az;
    this->accFilter(_aF);
  }

  
  Serial.print("\tg1\t");
  Serial.print(omegaval[1]);
  
  
  _wx = ((float) omegaval[0]);
  _wy = ((float) omegaval[1]);
  _wz = ((float) omegaval[2]);

  /*
  Serial.print("\tg2");
  Serial.print(_wy);
  */
  
  _wx =  _wx / (14.375);
  _wy =  _wy / (14.375);
  _wz =  _wz / (14.375);

  /*
  Serial.print("\tg3");
  Serial.println(_wy);
  */

  values[0] = ((float) accval[0]);
  values[1] = ((float) accval[1]);
  values[2] = ((float) accval[2]);
  values[3] = ((float) _wx);
  values[4] = ((float) _wy);
  values[5] = ((float) _wz);
  
}


/**
 * Returns angular velocities in a float array
 */
float Inertial::getAngularVel(int axis){
  float res;
  if (axis == XAXIS)
     res = _wx;
  else if (axis == YAXIS)
     res = _wy;
  else if (axis == ZAXIS)
     res = _wz;
  return res;
}



void Inertial::wFilter(float val[])
{
  val[0] = (1-alphaW)*val[0] + alphaW*_wFm1[0];
  val[1] = (1-alphaW)*val[1] + alphaW*_wFm1[1];
  val[2] = (1-alphaW)*val[2] + alphaW*_wFm1[2];
  
  _wFm1[0] = val[0];
  _wFm1[1] = val[1];
  _wFm1[2] = val[2];
}

void Inertial::accFilter(float val[])
{
  val[0] = (1-alphaA)*val[0] + alphaA*_aFm1[0];
  val[1] = (1-alphaA)*val[1] + alphaA*_aFm1[1];
  val[2] = (1-alphaA)*val[2] + alphaA*_aFm1[2];
  
  _aFm1[0] = val[0];
  _aFm1[1] = val[1];
  _aFm1[2] = val[2];
}

/**
 * Removes Bias and Scale Gyroscope values
 */
void Inertial::removeBiasAndScaleGyroData() {
  #ifdef L3G4200D
    _wx = (_wx - _gyroBiasX)*scale2000/1000;
    _wy = (_wy - _gyroBiasY)*scale2000/1000;
    _wz = (_wz - _gyroBiasZ)*scale2000/1000;
  #endif

  #ifdef MPU6050
    _wx = (_wx - _gyroBiasX)*scale2000/1000;
    _wy = (_wy - _gyroBiasY)*scale2000/1000;
    _wz = (_wz - _gyroBiasZ)*scale2000/1000;
  #endif
}


/**
 * Get Accelerometer's raw values
 */
void Inertial::getAcc(boolean accFilt) {

  #ifdef ADXL345_ACC 
    // TODO test
    // Read raw values
     _axRaw=analogRead(ACC_X_AXIS_PIN);
     _ayRaw=analogRead(ACC_Y_AXIS_PIN);
     _azRaw=analogRead(ACC_Z_AXIS_PIN);
  
    // convert Raw values
    float xAccScaled = map(_axRaw, xRawMin, xRawMax, -1000, 1000);
    float yAccScaled = map(_ayRaw, yRawMin, yRawMax, -1000, 1000);
    float zAccScaled = map(_azRaw, zRawMin, zRawMax, -1000, 1000);
    
    
    // re-scale to fractional Gs
    _ax = xAccScaled / 1000.0;
    _ay = yAccScaled / 1000.0;
    _az = zAccScaled / 1000.0;
     
     if (filterAcc)
     {
        _aF[0] = _ax;
        _aF[1] = _ay;
        _aF[2] = _az;
        accFilter(_aF);
     }
  #endif

  #ifdef MPU6050
  
    int accval[3];
    acc.readAccel(&accval[0], &accval[1], &accval[2]);
  
    _ax = ((float) accval[0]);
    _ay = ((float) accval[1]);
    _az = ((float) accval[2]);
  
    if (accFilt) {
      _aF[0] = _ax;
      _aF[1] = _ay;
      _aF[2] = _az;
      this->accFilter(_aF);
    } 
  #endif
}

/**
 * Get filtered Accelerometer's values - X Axis
 */
float Inertial::getAccXFilt() {
  return _aF[0];
}

/**
 * Get filtered Accelerometer's values - Y Axis
 */
float Inertial::getAccYFilt() {
  return _aF[1];
}

/**
 * Get filtered Accelerometer's values - Z Axis
 */
float Inertial::getAccZFilt() {
  return _aF[2];
}


/**
 * Get  Accelerometer's values - X Axis
 */
float Inertial::getAccX() {
  return _ax;
}

/**
 * Get  Accelerometer's values - X Axis
 */
float Inertial::getAccY() {
  return _ay;
}

/**
 * Get  Accelerometer's values - Z Axis
 */
float Inertial::getAccZ() {
  return _az;
}

/**
 * Get  Accelerometer's values - Z Axis
 */
float Inertial::getRollEstAcc() {
  return _phiAcc;
}


/**
 * Get  Accelerometer's values - Z Axis
 */
float Inertial::getPitchEstAcc() {
  return _thetaAcc;
}

 

