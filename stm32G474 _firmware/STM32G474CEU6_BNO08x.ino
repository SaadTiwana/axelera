/*
Dependency libraries:

ACANFD_STM32 (Pierre Molirano). Tested with 1.1.2-rc1
Adafruit BNO08x (Adafruit). Tested with 1.2.5

Arduino IDE settings:
Board : Generic STM32G4 series
Port : <Actual>
Board Part Number: WeAct G474CE
Upload method: STM32CubeProgrammer (DFU)
USB support: CDC Generic Serial 
U(S)ART support: Enabled (Generic 'Serial')
*/

#include <iostream>
#include <Wire.h>
#include "motorController.hpp"
#include "ahrs.hpp"
#include "pid.hpp"
#include "lowpass_filter.hpp"
#include "UARTProtocol.hpp"
#include "SystemStatus.hpp"

// IMPORTANT:<ACANFD_STM32.h> should be included only once in a sketch, generally from the .ino file
//   From an other file, include <ACANFD_STM32_from_cpp.h>
#include <ACANFD_STM32.h>
// https://github.com/pierremolinaro/acanfd-stm32/blob/main/extras/acanfd-stm32.pdf
// Default TxCAN pins are used:
//   - fdcan1 (default pins): PA12 (TX), PA11 (RX) -> Used by USB on WeAct STM32G474 board
//   - fdcan2 (default pins): PB6  (TX), PB5  (RX)
//   - fdcan3 (default pins): PA15 (TX), PA8  (RX)
#define FDCAN1_TX PA12 
#define FDCAN1_RX PA11

#define FDCAN2_TX PB13
#define FDCAN2_RX PB12


#define I2C3_SCL PA8
#define I2C3_SDA PC11
TwoWire Wire3(I2C3_SDA, I2C3_SCL);

AHRS ahrs(Wire3);

// UART options:
//    USART1 TX (PA9), RX (PA10)  -> Pins available on WeAct STM32G474      (HardwareSerial "Serial1")
//    USART2 TX (PA2), RX (PA3)   -> Pins available on WeAct STM32G474      (HardwareSerial "Serial2")
//    USART3 TX (PB10), RX (PB11) -> Pins shared with Qspi flash (if used?) (HardwareSerial "Serial3")
// HardwareSerial Serial_sbc(PA3, PA2);  // RX, TX
// HardwareSerial Serial_debug(PA10, PA9);   // RX, TX
HardwareSerial Serial_debug(PA3, PA2);  // RX, TX
HardwareSerial Serial_sbc(PA10, PA9);   // RX, TX




/*

Angles are commonly defined according to the right-hand rule. Namely, they have positive values 
when they represent a rotation that appears clockwise when looking in the positive direction of the axis, 
and negative values when the rotation appears counter-clockwise. 

We use ENU frame for now (used in Land vehicles). X axis points forward, Y points left and z points up.

*/




// From: https://doc.synapticon.com/circulo/tutorials/tuning_guides/position_controller_with_cascaded_structure.html
// By default, the integral limit of the velocity controller should be set to motor maximum torque in [mNm], 
// and the integral limit of position controller should be set to motor maximum velocity in [rpm] (dps in our case?).
// Note to self: Adjust units as per our own values


// -------------- PITCH AXIS --------------
// --- velocity PID controller parameters and instance ---
float pitch_vel_P     = 0.01;
float pitch_vel_I     = 0.0;
float pitch_vel_D     = 0.0;
float pitch_vel_ramp  = 0.0;   // Disabled if 0.0
float pitch_vel_limit = 0.5;    // in Amperes if output is Torque current. in deg/s if output is angular velocity.
// Velocity pid controller (output will be velocity or torque command)
PIDController pidc_pitch_vel = PIDController(pitch_vel_P, pitch_vel_I, pitch_vel_D, pitch_vel_ramp, pitch_vel_limit);

float lpf_Tf_gyroY = 0.01; // Y axis gyro LPF time constant in seconds
LowPassFilter lpf_gyroY = LowPassFilter(lpf_Tf_gyroY); // For pitch axis, we use Y axis gyro

// --- Position PID controller parameters and instance ---
float pitch_pos_P     = 0.1;
float pitch_pos_I     = 0.1;
float pitch_pos_D     = 0.0;
float pitch_pos_ramp  = 0.0; // Disabled if 0.0
float pitch_pos_limit = 0.05;  // in deg? if output is angular velocity.

// Pitch Position controller (output will be a velocity command - dps)
PIDController pidc_pitch_pos = PIDController(pitch_pos_P, pitch_pos_I, pitch_pos_D, pitch_pos_ramp, pitch_pos_limit); 

float pitch_pos_tgt_startup = 0.0;


// -------------- YAW AXIS --------------
// --- velocity PID controller parameters and instance ---
float yaw_vel_P     = 0.1;
float yaw_vel_I     = 0.0;
float yaw_vel_D     = 0.0;
float yaw_vel_ramp  = 0.0;   // Disabled if 0.0
float yaw_vel_limit = 2.0;    // in Amperes if output is Torque current. in deg/s if output is angular velocity.
// Velocity pid controller (output will be velocity or torque command)
PIDController pidc_yaw_vel = PIDController(yaw_vel_P, yaw_vel_I, yaw_vel_D, yaw_vel_ramp, yaw_vel_limit);

// --- Position PID controller parameters and instance ---
float yaw_pos_P     = 0.1; //10.0
float yaw_pos_I     = 0.1; //5.0
float yaw_pos_D     = 0.0; //0.0
float yaw_pos_ramp  = 0.0; // Disabled if 0.0
float yaw_pos_limit = 1.0; //10.0

// Position controller (output will be a velocity command - dps)
PIDController pidc_yaw_pos = PIDController(yaw_pos_P, yaw_pos_I, yaw_pos_D, yaw_pos_ramp, yaw_pos_limit); 

float yaw_pos_tgt_startup = 0.0;

// -------------- STABILIZATION FLAGS --------------
bool STABILIZE_ON_START = false; // MASTER SWITCH. Always keep false (to be turned on manually via serial command)
bool STABILIZE_PITCH = true;
bool STABILIZE_YAW = true;


bool MOTION_MODE_BASIC = true; // If true only position control is used and no IMU based stabilization is done. If false, cascaded position and velocity control is used.


// FLAGS that track current states
bool DISP_LV = false; // Show live variable display on Serial port or not


// Class variables
MotorController motorController;
SystemStatus sysStatus;
UARTProtocol sbcInterface(Serial_sbc, sysStatus, &Serial);  // Pass pointer to Serial and stabilize flag

// Below are in MOTOR position coordinates, not AHRS coordinates
#define POS_LIM_PITCH_DEG_MIN -30.0 
#define POS_LIM_PITCH_DEG_MAX +90   

#define POS_LIM_YAW_DEG_MIN -90.0
#define POS_LIM_YAW_DEG_MAX +90.0


// Measure loop execution frequency
unsigned long lastTime = 0;
unsigned long loopCount = 0;
unsigned long executionFreq_Hz = 0;

unsigned long loopCount_ahrs = 0;

void stopAllMotion() {
  //motorController.SetMotorVelocityClosedLoop_blocking(MotorController::MOT_PITCH, 0);
  //motorController.SetMotorVelocityClosedLoop_blocking(MotorController::MOT_YAW, 0);

  // I suspect the setting velocity to zero at beginning may be causing sudden movements of the motor(s). Not confirmed. Let's try torque instead.
  // motorController.SetMotorTorqueCurrentClosedLoop_blocking(MotorController::MOT_PITCH, 0.0);
  //motorController.SetMotorTorqueCurrentClosedLoop_blocking(MotorController::MOT_YAW, 0.0);

  if (false) {
    /******Non blocking *******/
    motorController.SetMotorTorqueCurrentClosedLoop_withCallback(MotorController::MOT_PITCH, 0.0, nullptr);
    motorController.SetMotorTorqueCurrentClosedLoop_withCallback(MotorController::MOT_YAW, 0.0, nullptr);

  } else {
    /*****Blocking******/
    motorController.SetMotorTorqueCurrentClosedLoop_blocking(MotorController::MOT_PITCH, 0.0);
    motorController.SetMotorTorqueCurrentClosedLoop_blocking(MotorController::MOT_YAW, 0.0);
  }
}


void setup() 
{
  Serial.begin(115200);
  //while(!Serial) delay(10); // Causes code to not execute if serial port is not opened. 
  delay(5000);
  Serial.println();
  Serial.println("Tracker Program Beginning...");

  Serial.println("Setting AHRS ...");
  ahrs.begin();
  
  delay(100);

  sysStatus.stabilize = STABILIZE_ON_START;

  
  motorController.begin();
  delay(1000);
  Serial.println("Setting motor(s) to zero");

  sbcInterface.begin(115200);

  Serial_debug.begin(115200);

  // Stop motor(s) in case motors have an active command from before controller booting up.
  stopAllMotion();

  // Send to zero position
  // VERY IMPORTANT: The Motors should already be zeroed before this
  motorController.SetMultiturnMotorPosition_blocking(MotorController::MOT_PITCH, 0, 20);
  motorController.SetMultiturnMotorPosition_blocking(MotorController::MOT_YAW, 0, 20);

  delay(5000);

  // Turn off motors
  motorController.SetMotorTorqueCurrentClosedLoop_blocking(MotorController::MOT_PITCH, 0.0);
  motorController.SetMotorTorqueCurrentClosedLoop_blocking(MotorController::MOT_YAW, 0.0);
  
  // Load Initial Target angles to the sysStatus variables. Later they will be changed by sbc program
  sysStatus.mot_angle_pitch_deg_target = pitch_pos_tgt_startup;
  sysStatus.mot_angle_yaw_deg_target = yaw_pos_tgt_startup;

  lastTime = micros();
}



/*
EXAMPLE SERIAL COMMANDS:
SET PID PITCH POS P 7.5
SET PID PITCH POS LIM 1
SET PID YAW VEL P 0.2
SET PID PITCH VEL LIM 100
TOGGLE DISP_LV
SET TARGET PITCH -10.0
SET TARGET YAW 15.0
SET STABILIZE ON
*/
void handleSerialCommands() {
  static String inputLine = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        inputLine.trim();

        // Tokenize the input
        int tokenCount = 0;
        String tokens[8];
        int fromIndex = 0;
        while (fromIndex < inputLine.length() && tokenCount < 8) {
          int spaceIdx = inputLine.indexOf(' ', fromIndex);
          if (spaceIdx == -1) spaceIdx = inputLine.length();
          tokens[tokenCount++] = inputLine.substring(fromIndex, spaceIdx);
          fromIndex = spaceIdx + 1;
        }

        if ((tokenCount == 6) && (tokens[0] == "SET") && (tokens[1] == "PID")) {
          String axis = tokens[2];
          String loop = tokens[3];
          String param = tokens[4];
          float value = tokens[5].toFloat();
          PIDController* pid = nullptr;
          if (axis == "PITCH" && loop == "POS") pid = &pidc_pitch_pos;
          if (axis == "PITCH" && loop == "VEL") pid = &pidc_pitch_vel;
          if (axis == "YAW" && loop == "POS") pid = &pidc_yaw_pos;
          if (axis == "YAW" && loop == "VEL") pid = &pidc_yaw_vel;
          if (pid) {
            if (param == "P") pid->P = value;
            else if (param == "I") pid->I = value;
            else if (param == "D") pid->D = value;
            else if (param == "LIM") pid->limit = value;
            else if (param == "RAMP") pid->output_ramp = value;
            Serial.println("OK");
          } else {
            Serial.println("Unknown PID target");
          }

        } else if ((tokenCount == 4) && (tokens[0] == "SET") && (tokens[1] == "TARGET")) {
          String axis = tokens[2];
          float value = tokens[3].toFloat();
          if (axis == "PITCH") {
            sysStatus.mot_angle_pitch_deg_target = value;
            Serial.print("Pitch target set to ");
            Serial.println(sysStatus.mot_angle_pitch_deg_target, 3);
          } else if (axis == "YAW") {
            sysStatus.mot_angle_yaw_deg_target = value;
            Serial.print("Yaw target set to ");
            Serial.println(sysStatus.mot_angle_yaw_deg_target, 3);
          } else {
            Serial.println("Unknown axis for target");
          }

        } else if ((tokenCount == 3) && (tokens[0] == "SET") && (tokens[1] == "STABILIZE")) {
          String value = tokens[2];
          if (value == "ON") {
            sysStatus.stabilize = true;
            Serial.println("STABILIZE set to ON");
          } else if (value == "OFF") {
            sysStatus.stabilize = false;
            Serial.println("STABILIZE set to OFF");
          } else {
            Serial.println("Invalid value for STABILIZE (use ON or OFF)");
          }        
        
        } else if (inputLine.startsWith("TOGGLE DISP_LV")) {
          DISP_LV = !DISP_LV;
          Serial.print("DISP_LV is now ");
          Serial.println(DISP_LV ? "ON" : "OFF");
        } else {
          Serial.println("Invalid command");
        }
        inputLine = "";
      }
    } else {
      inputLine += c;
    }
  }
}


void cb_onPitchMotorPosition(uint motor_id, int status, const uint8_t* data) 
{
  // Although it should not happen, let's confirm motor ID anyway
  if (motor_id != MotorController::MOT_PITCH) {
    Serial.print("BUG - Received pitch motor callback for wrong motor ID: ");
    Serial.println(motor_id, HEX);
    return;
  }

    if (status != MotorController::STATUS_OK || !data) {
        Serial.print("Error retrieving position for PITCH motor, Status: ");
        Serial.println(status);
        return;
    }

    float multiturnAngle_deg = motorController.DecodeMultiturnMotorPositionReply(data);
      sysStatus.mot_angle_pitch_deg = multiturnAngle_deg;
      // Serial.print("(Callback) PITCH Motor pos: ");
      // Serial.println(multiturnAngle_deg, 3);
}

void cb_onYawMotorPosition(uint motor_id, int status, const uint8_t* data) 
{
  // Although it should not happen, let's confirm motor ID anyway
  if (motor_id != MotorController::MOT_YAW) {
    Serial.print("BUG - Received yaw motor callback for wrong motor ID: ");
    Serial.println(motor_id, HEX);
    return;
  }

    if (status != MotorController::STATUS_OK || !data) {
        Serial.print("Error retrieving position for YAW motor, Status: ");
        Serial.println(status);
        return;
    }

    float multiturnAngle_deg = motorController.DecodeMultiturnMotorPositionReply(data);
      sysStatus.mot_angle_yaw_deg = multiturnAngle_deg;
      // Serial.print("(Callback) YAW Motor pos: ");
      // Serial.println(multiturnAngle_deg, 3);
}


void requestMotorPositions() {


if (false) {
  /***** Non blocking (Async) position retrieval *****/

  // motorController already implements funcitonality to reject multiple requests if a request is already pending
  // so we do not need to track pending requests ourselves.
  motorController.RequestMultiturnMotorPosition_withCallback(MotorController::MOT_YAW, cb_onYawMotorPosition);
  motorController.RequestMultiturnMotorPosition_withCallback(MotorController::MOT_PITCH, cb_onPitchMotorPosition);
} else {
  /***** Blocking (Async) position retrieval *****/

  // Blocking motor-positions retrieval (not desirable as it takes ~2ms per command)

  float pitch, yaw;
  if (motorController.GetMultiturnMotorPosition_blocking(MotorController::MOT_PITCH, pitch) == MotorController::STATUS_OK) {
    sysStatus.mot_angle_pitch_deg = pitch;
  } else {
    Serial.println("ERROR: Mot pos retrieval failed_pitch");
  }

  if (motorController.GetMultiturnMotorPosition_blocking(MotorController::MOT_YAW, yaw) == MotorController::STATUS_OK) {
    //Serial.println("Updating mot_angle_yaw_deg");
    sysStatus.mot_angle_yaw_deg = yaw;
  } else {
    Serial.println("ERROR: Mot pos retrieval failed_yaw");
  }
}





}

/*
Stop motors IF:
1. Motor position is out of range (e.g., less than -45.0 or greater than 45.0 degrees).
2. Motor position retrieval fails.
3. AHRS update fails (e.g., if the sensor is not responding or if the data is invalid).

Else:
Send velocity command to motor to cancel the pitch gyro values
*/ 


void serviceCommHandlers()
{
  motorController.processCanReplies();  // Process any pending CAN replies -> Works with Async messaging
  sbcInterface.update(); // Handle UART commands from SBC
  handleSerialCommands();
}


#define MOT_ANGLE_STALE_LIMIT_US 20000 // 20ms

void loop_commtest() 
{
  loopCount++;
float pitch, yaw;
  if (motorController.GetMultiturnMotorPosition_blocking(MotorController::MOT_PITCH, pitch) == MotorController::STATUS_OK) {
    sysStatus.mot_angle_pitch_deg = pitch;
    Serial.print(".");
  } else {
    Serial.println("ERROR: Mot pos retrieval failed_pitch");
  }

  // Check every second (1,000,000 microseconds)
  if (micros() - lastTime >= 1000000) {
    Serial.print("************************************");
    Serial.print("Loop freq: ");
    Serial.print(loopCount);
    Serial.println(" Hz, ");
    loopCount = 0;
    lastTime = micros();
  }

  //delayMicroseconds(1000);

}


void loop() 
{
  bool error = false;

  //delayMicroseconds(100);

  //Serial_sbc.print("*");
  //Serial_debug.print("&");    
  
  serviceCommHandlers();
  requestMotorPositions();

  //delayMicroseconds(2000); // Slow down the loop (temporary fix for motors not being able to respond so fast)

  serviceCommHandlers();



  //Serial.print("Async Motor Pos YAW : ");
  //Serial.println(asyncLatestMotorPos_YAW, 3);
  // Serial.print("Async Motor Position PITCH : ");
  // Serial.println(asyncLatestMotorPos_PITCH, 3);


  // Check if the AHRS update was successful before using the data
  if (ahrs.update()) {

    // Use ahrs.ahrsRoll_rad, ahrs.ahrsPitch_rad, ahrs.ahrsYaw_rad, ahrs.gyroX, ahrs.gyroY, ahrs.gyroZ here
    float ahrsRoll_rad = ahrs.latestAttitudeEuler_rad.roll;
    float ahrsPitch_rad = ahrs.latestAttitudeEuler_rad.pitch;
    float ahrsYaw_rad = ahrs.latestAttitudeEuler_rad.yaw;
    float gyroX = ahrs.gyroX;
    float gyroY = ahrs.gyroY;
    float gyroZ = ahrs.gyroZ;



    /*
    Serial.print("Euler-deg (Quat): Yaw=");
    Serial.print(ahrsYaw_rad * 180.0 / PI, 2);
    Serial.print(" , Pitch=");
    Serial.print(ahrsPitch_rad * 180.0 / PI, 2);
    Serial.print(" , Roll=");
    Serial.print(ahrsRoll_rad * 180.0 / PI, 2);
    Serial.print(" gX:");
    Serial.print(gyroX * 180.0 / PI, 3);
    Serial.print(" gY:");
    Serial.print(gyroY * 180.0 / PI, 3);
    Serial.print(" gZ:");
    Serial.print(gyroZ * 180.0 / PI, 3);
    Serial.println();
    */

    
    //Serial.println("Attempting to retrive motor Position");

    /*
    float motorPosition;
    if (motorController.GetMultiturnMotorPosition_int32(MotorController::MOT1_CAN_ID, motorPosition) == MotorController::STATUS_OK) 
    {
      Serial.println("Motor position retrieval successful");
      Serial.println("Sending Position command");
      float pitch = ahrsPitch_rad * 180.0 / PI;
      if ((pitch > -40.0) && (pitch < 40.0)) 
      {
        if (motorController.SetMultiturnMotorPosition_int32(MotorController::MOT1_CAN_ID, pitch, 45) == MotorController::STATUS_OK)
          Serial.println("Pos success");
      }
    }
    */

    serviceCommHandlers();

    if (STABILIZE_PITCH)
    {
      // ensure we can talk to motors by checking that motor angle values are not stale
      if ( !sysStatus.mot_angle_pitch_deg.isStale(MOT_ANGLE_STALE_LIMIT_US)) 
      { 
        float motorPos_pitch = sysStatus.mot_angle_pitch_deg.value;       
        if ((motorPos_pitch > POS_LIM_PITCH_DEG_MIN) && (motorPos_pitch < POS_LIM_PITCH_DEG_MAX)) 
        {
          // --- Cascaded control ---
          // Outer loop: Position PID
          float ang_err = (sysStatus.mot_angle_pitch_deg_target - (ahrsPitch_rad * 180.0 / PI));
          float vel_setpoint = pidc_pitch_pos(ang_err); // deg/sec

          // Inner loop: Velocity PID
          float filteredGyroY_deg = lpf_gyroY(gyroY*180.0/PI); // Apply LPF to gyro Y
          float vel_measured = filteredGyroY_deg; // deg/sec
          float vel_err = vel_setpoint - vel_measured;

          // Velocity command to motors
          //float velCmd_dps = pidc_pitch_vel(vel_err);
          //if (sysStatus.stabilize)
          //  motorController.SetMotorVelocityClosedLoop_blocking(MotorController::MOT_PITCH, velCmd_dps);     

          // Torque command to motors
          // CASCADED position-velocity loops
          //float torqueCmd_A = pidc_pitch_vel(vel_err); // Output in Amps
          // Direct Position-to-torque
          float torqueCmd_A = vel_setpoint; // Output in Amps

          if (sysStatus.stabilize)
            if (MOTION_MODE_BASIC) {
              //motorController.SetMultiturnMotorPosition_withCallback(MotorController::MOT_PITCH, sysStatus.mot_angle_pitch_deg_target, 30, nullptr); // Basic position control mode
              motorController.SetMultiturnMotorPosition_blocking(MotorController::MOT_PITCH, sysStatus.mot_angle_pitch_deg_target, 30);
            }
            else
              motorController.SetMotorTorqueCurrentClosedLoop_withCallback(MotorController::MOT_PITCH, torqueCmd_A, nullptr); // CASCADED position-velocity control mode
          else
            stopAllMotion();
          
          if (DISP_LV){
            Serial.print("P:");
            Serial.print("\t");
            Serial.print(ahrsPitch_rad * 180.0 / PI , 3);
            Serial.print("\t");
            Serial.print(motorPos_pitch , 3);
            Serial.print("\t");
            Serial.print(gyroY * 180.0 / PI , 3);
            Serial.print("\t");
            //Serial.print(velCmd_dps, 3);
            Serial.print(torqueCmd_A, 4);
            //Serial.println();
            Serial.print("\t");
          }
        }
        else
        {
          Serial.print("Pitch Motor position out of range, stopping motor ");
          Serial.println(motorPos_pitch , 3);
          error = true; // Set error flag if position is out of range
        }
      }
      else 
      {
        Serial.print("ERROR: Motor position stale_PITCH. Age(us): ");
        Serial.println(sysStatus.mot_angle_pitch_deg.getAge_us());
        error = true; // Set error flag if position retrieval fails
      }

    }
    
    serviceCommHandlers();
   
    if (STABILIZE_YAW)
    {
      // ensure we can talk to motors by checking that motor angle values are not stale
      if ( !sysStatus.mot_angle_yaw_deg.isStale(MOT_ANGLE_STALE_LIMIT_US))  
      {    
        float motorPos_yaw = sysStatus.mot_angle_yaw_deg.value;    
        if ((motorPos_yaw > POS_LIM_YAW_DEG_MIN) && (motorPos_yaw < POS_LIM_YAW_DEG_MAX)) 
        {

          // --- Cascaded control ---
          // Outer loop: Position PID
          float ang_err = (sysStatus.mot_angle_yaw_deg_target - (ahrsYaw_rad * 180.0 / PI));
          float vel_setpoint = pidc_yaw_pos(ang_err); // deg/sec

          // Inner loop: Velocity PID
          float vel_measured = gyroZ * 180.0 / PI; // deg/sec
          float vel_err = vel_setpoint - vel_measured;

          // Velocity command to motors
          //float velCmd_dps = pidc_yaw_vel(vel_err);
          //IF (sysStatus.stabilize)
          //  motorController.SetMotorVelocityClosedLoop_blocking(MotorController::MOT_YAW, -1.0*velCmd_dps); // Multiplying by -1 because motor direction is opposite of AHRS/gyro polarity (Yaw)

          // Torque command to motors
          float torqueCmd_A = pidc_yaw_vel(vel_err); // Output in Amps
          if (sysStatus.stabilize)
            if (MOTION_MODE_BASIC){
              //motorController.SetMultiturnMotorPosition_withCallback(MotorController::MOT_YAW, sysStatus.mot_angle_yaw_deg_target, 30, nullptr); // Basic position control mode
              motorController.SetMultiturnMotorPosition_blocking(MotorController::MOT_YAW, sysStatus.mot_angle_yaw_deg_target, 30);
            }
            else
            motorController.SetMotorTorqueCurrentClosedLoop_withCallback(MotorController::MOT_YAW, -1.0 * torqueCmd_A, nullptr); // Multiplying by -1 because motor direction is opposite of AHRS/gyro polarity (Yaw)
          else
            stopAllMotion();

          if (DISP_LV){
            Serial.print("Y:");
            Serial.print("\t");
            Serial.print(ahrsYaw_rad * 180.0 / PI , 3);
            Serial.print("\t");
            Serial.print(motorPos_yaw , 3);
            Serial.print("\t");
            Serial.print(gyroZ * 180.0 / PI , 3);
            Serial.print("\t");
            //Serial.print(velCmd_dps, 3);
            Serial.print(torqueCmd_A, 4);
            Serial.print("\t");
            //Serial.println();
          }
        }
        else
        {
          Serial.print("Yaw Motor position out of range, stopping motor ");
          Serial.println(motorPos_yaw , 3);
        }
      }
      else 
      {
        Serial.print("ERROR: Motor position stale_YAW. Age(us): ");
        Serial.println(sysStatus.mot_angle_yaw_deg.getAge_us());
        error = true; // Set error flag if position retrieval fails
      }

    }

    loopCount_ahrs++;

  }
  /*
  else
  {
    Serial.println("AHRS update failed");
    error = true; // Set error flag if AHRS update fails
  }


  if (error) 
  {
    Serial.println("Stopping motor(s)");
    // Stop motor(s) in case of any error
    motorController.SetMotorVelocityClosedLoop_blocking(MotorController::MOT_PITCH, 0); 
    motorController.SetMotorVelocityClosedLoop_blocking(MotorController::MOT_YAW, 0); // Stop motor in case of any error
    // Send to zero position
    //motorController.SetMultiturnMotorPosition_blocking(MotorController::MOT_PITCH, 0, 45);

    // To clear integrators and prevent windup
    pidc_pitch_vel.reset();
    pidc_pitch_pos.reset();
    pidc_yaw_vel.reset();
    pidc_yaw_pos.reset();

  }*/

  serviceCommHandlers();

  if (DISP_LV){
    Serial.print("EX:");
    Serial.print("\t");
    Serial.print(executionFreq_Hz);
    Serial.print("\t");
    Serial.print(error);
    Serial.println();
  }
  

  loopCount++;

  // Check every second (1,000,000 microseconds)
  if (micros() - lastTime >= 1000000) {
    Serial.print("************************************");
    Serial.print("Loop freq: ");
    Serial.print(loopCount);
    Serial.print(" Hz, ");
    executionFreq_Hz = loopCount;
    loopCount = 0;
    lastTime = micros();

    Serial.print("AHRS freq: ");
    Serial.print(loopCount_ahrs);
    Serial.println(" Hz");
    loopCount_ahrs =0;

    motorController.PrintCanLinkStats(); // Print CAN link stats
  }

}




