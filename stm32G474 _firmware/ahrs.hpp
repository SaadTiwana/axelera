/*
  Using the BNO08x IMU

  This example shows how to use the gyro integrated rotation vector.

  It outputs the GyroIntegrated i/j/k/real/angVelX/angVelY/angVelZ parts of the
  rotation vector.
  https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation

  Also, utilizing I2C and SPI read/write functions and code from the Adafruit
  BusIO library found here:
  https://github.com/adafruit/Adafruit_BusIO

  Hardware Connections:
  IoT RedBoard --> BNO08x
  QWIIC --> QWIIC
  A4  --> INT
  A5  --> RST

  BNO08x "mode" jumpers set for I2C (default):
  PSO: OPEN
  PS1: OPEN
*/


#ifndef AHRS_HPP
#define AHRS_HPP



#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"
#include "time_utils.hpp"


  // Define a quaternion (w, x, y, z)
  typedef struct {
      float w;
      float x;
      float y;
      float z;
  } Quaternion_t;

  // Define Euler angles (roll, pitch, yaw) 
  typedef struct {
      float roll;
      float pitch;
      float yaw;
  } EulerAngles_t;



// For the most reliable interaction with the SHTP bus, we need
// to use hardware reset control, and to monitor the H_INT pin.
// The H_INT pin will go low when its okay to talk on the SHTP bus.
// Note, these can be other GPIO if you like.
// Define as -1 to disable these features.
//#define BNO08X_INT  PB6
#define BNO08X_INT  -1
//#define BNO08X_RST  PB7
#define BNO08X_RST  -1

#define BNO08X_ADDR 0x4B  // SparkFun BNO08x Breakout (Qwiic) defaults to 0x4B
//#define BNO08X_ADDR 0x4A // Alternate address if ADR jumper is closed

#define PARAM_MAX_INTERVAL_BW_TIME_CORRELATED_SENSOR_REPORTS_US 1500 // Criteria to determine if both sensor reports are from roughly same time

#define DBG_AHRS false // Set to true to enable debug prints in AHRS class

class AHRS {
public:

    uint32_t lastGyroTimestamp_us = 0;
    uint32_t lastRotVecTimestamp_us = 0;
    uint32_t maxSampleAge_us = 2500; 


    AHRS(TwoWire& wireRef);
    void begin();
    bool update();

    bool update_old(); // Not to be used
    bool update_old2(); // Not to be used

    // Latest orientation and gyro values
    EulerAngles_t latestAttitudeEuler_rad;

    float gyroX = 0;
    float gyroY = 0;
    float gyroZ = 0;

private:

    TwoWire& wire;
    BNO08x myIMU;
    EulerAngles_t quaternionToEuler(Quaternion_t q);
    void setReports();
    void getGyroData();
    void getRotationVectorData();

};

#endif // AHRS_HPP
