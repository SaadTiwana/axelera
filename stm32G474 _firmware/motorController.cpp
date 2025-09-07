#include "motorController.hpp"
#include <string.h>

MotorController::PendingCanRequest MotorController::pendingRequests[MotorController::MAX_PENDING_QUEUE_SIZE] = {};




void MotorController::begin()
{
  ACANFD_STM32_Settings settings(1000 * 1000, DataBitRateFactor::x1);  
  //ACANFD_STM32_Settings settings(1000 * 1000, 50, DataBitRateFactor::x1, 50, 1000);

  Serial.print("CPU frequency: ");
  Serial.print(F_CPU);
  Serial.println(" Hz");
  Serial.print("PCLK1 frequency: ");
  Serial.print(HAL_RCC_GetPCLK1Freq());
  Serial.println(" Hz");
  Serial.print("PCLK2 frequency: ");
  Serial.print(HAL_RCC_GetPCLK2Freq());
  Serial.println(" Hz");
  Serial.print("HCLK frequency: ");
  Serial.print(HAL_RCC_GetHCLKFreq());
  Serial.println(" Hz");
  Serial.print("SysClock frequency: ");
  Serial.print(HAL_RCC_GetSysClockFreq());
  Serial.println(" Hz");
  Serial.print("FDCAN Clock: ");
  Serial.print(fdcanClock());
  Serial.println(" Hz");
 
  // settings.mModuleMode = ACANFD_STM32_Settings::EXTERNAL_LOOP_BACK ;
  settings.mModuleMode = ACANFD_STM32_Settings::NORMAL_FD;
  // settings.mModuleMode = ACANFD_STM32_Settings::INTERNAL_LOOP_BACK ;

  Serial.print("Bit Rate prescaler: ");
  Serial.println(settings.mBitRatePrescaler);
  Serial.print("Arbitration Phase segment 1: ");
  Serial.println(settings.mArbitrationPhaseSegment1);
  Serial.print("Arbitration Phase segment 2: ");
  Serial.println(settings.mArbitrationPhaseSegment2);
  Serial.print("Arbitration SJW: ");
  Serial.println(settings.mArbitrationSJW);
  Serial.print("Actual Arbitration Bit Rate: ");
  Serial.print(settings.actualArbitrationBitRate());
  Serial.println(" bit/s");
  Serial.print("Arbitration sample point: ");
  Serial.print(settings.arbitrationSamplePointFromBitStart());
  Serial.println("%");
  Serial.print("Exact Arbitration Bit Rate ? ");
  Serial.println(settings.exactArbitrationBitRate() ? "yes" : "no");
  Serial.print("Data Phase segment 1: ");
  Serial.println(settings.mDataPhaseSegment1);
  Serial.print("Data Phase segment 2: ");
  Serial.println(settings.mDataPhaseSegment2);
  Serial.print("Data SJW: ");
  Serial.println(settings.mDataSJW);
  Serial.print("Actual Data Bit Rate: ");
  Serial.print(settings.actualDataBitRate());
  Serial.println(" bit/s");
  Serial.print("Data sample point: ");
  Serial.print(settings.dataSamplePointFromBitStart());
  Serial.println("%");
  Serial.print("Exact Data Bit Rate ? ");
  Serial.println(settings.exactDataBitRate() ? "yes" : "no");

  //--- beginFD is called without any receive filter, all sent frames are received
  //    by receiveFD0 throught receiveFIFO0

  uint32_t errorCode;
  /*
  uint32_t errorCode = fdcan1.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("fdcan1 configuration ok") ;
  }else{
    Serial.print ("Error fdcan1: 0x") ;
    Serial.println (errorCode, HEX) ;
  }
  */

  errorCode = fdcan2.beginFD(settings);
  if (0 == errorCode)
  {
    Serial.println("fdcan2 configuration ok");
  }
  else
  {
    Serial.print("Error fdcan2: 0x");
    Serial.println(errorCode, HEX);
  }
}


int MotorController::SendCanMessageToMotorAndReturnReplyBlocking(uint motor_id, uint8_t *cmd_data)
{
  uint8_t sentCmd = cmd_data[0];

  CANMessage message;
  message.ext = false;   // Standard message
  message.id = motor_id; // Motor IDs: 0x140 + 1/2/3..8
  message.len = 8;       // Always 8 bytes

  memcpy(message.data, cmd_data, message.len);

  //--- tryToSendReturnStatusFD returns 0 if message has been successfully
  //    appended in transmit buffer.
  //  A not zero returned value contains an error code (see doc)
  uint32_t sendStatus = fdcan2.tryToSendReturnStatusFD(message);
  if (sendStatus != 0)
  {
    can_tx_failed_count++;
    return STATUS_FAILED;
  }
    
  /*else
  {
    gSentCount1 += 1 ;
    Serial.print ("fdcan1 sent: ") ;
    Serial.println (gSentCount1) ;
  }*/

  // delay(2); //in ms; TODO: Remove/tune
  delayMicroseconds(2000);

  CANFDMessage rx_frame;
  if (fdcan2.receiveFD0(rx_frame))
  {
    // Make sure it's the reply for same message, from same motor
    if ((rx_frame.id == motor_id) && (rx_frame.data[0] == sentCmd) && rx_frame.len == 8)
    {
      // Copy the received data back into cmd_data
      memcpy(cmd_data, rx_frame.data, rx_frame.len);
      return STATUS_OK;
    }
  }
  // If execution reaches here, means something failed.
  return STATUS_FAILED;
}


// To be called repeatedly in loop() to process CAN replies
// This function checks for any CAN messages received and matches them to pending requests.
// Also removes stale requests that have not been completed within a certain timeout period.
void MotorController::processCanReplies()
{
    CANFDMessage rx_frame;
    uint32_t now = _micros();

    // 1. Reply handling (process all received messages first)
    while (fdcan2.receiveFD0(rx_frame)) {
        can_rx_count++;
        bool matched = false;
        for (int i = 0; i < MAX_PENDING_QUEUE_SIZE; ++i) {
            if (pendingRequests[i].in_use &&
                rx_frame.id == pendingRequests[i].motor_id &&
                rx_frame.data[0] == pendingRequests[i].cmd_id &&
                rx_frame.len == 8) {
                can_rx_good_count++;
                memcpy(pendingRequests[i].reply_buffer, rx_frame.data, 8);
                if (pendingRequests[i].callback) 
                {
                    pendingRequests[i].callback(rx_frame.id, STATUS_OK, rx_frame.data);
                }
                pendingRequests[i].in_use = false;
                pendingRequests[i].callback = nullptr;
                matched = true;
                break;
            }
        }
        if (!matched) {
            can_rx_unknown++;
            if (DEBUG_MOT_CTRL) {
                Serial.print("Unknown reply from motor ID: ");
                Serial.print(rx_frame.id, HEX);
                Serial.print(" Data: ");
                for (int j = 0; j < rx_frame.len; ++j) {
                    Serial.print(rx_frame.data[j], HEX);
                    Serial.print(" ");
                }
            }
        }
    }

    // 2. Timeout handling (now only mark as timeout if reply wasn't already processed)
    // NOTE: This MUST be done AFTER handling all messages already in receive buffer
    // so that an older request is not marked stale if reply already exists in receive buffer (bug earlier)
    for (int i = 0; i < MAX_PENDING_QUEUE_SIZE; ++i) {
        if (pendingRequests[i].in_use &&
            (now - pendingRequests[i].timestamp_us > SETTING_MAX_REQUEST_AGE_US)) {
            if (pendingRequests[i].callback) {
                pendingRequests[i].callback(pendingRequests[i].motor_id, STATUS_TIMEOUT, nullptr);
            }
            pendingRequests[i].in_use = false;
            pendingRequests[i].callback = nullptr;
            can_no_rep_count++;
        }
    }
}


// Send a CAN message to the motor and register a callback for the reply
int MotorController::SendCanMessageWithCallback(uint motor_id, uint8_t* cmd_data, MotorReplyCallback callback)
{
    // Only send if not already pending for this motor/cmd
    for (int i = 0; i < MAX_PENDING_QUEUE_SIZE; ++i) {
        if (pendingRequests[i].in_use &&
            pendingRequests[i].motor_id == motor_id &&
            pendingRequests[i].cmd_id == cmd_data[0]) 
            {
              // Already pending, do not send again
            can_tx_declined_pending_count++;
            return STATUS_FAILED;
        }
    }

    CANMessage message;
    message.ext = false;
    message.id = motor_id;
    message.len = 8;
    memcpy(message.data, cmd_data, 8);

    uint32_t sendStatus = fdcan2.tryToSendReturnStatusFD(message);
    if (sendStatus != 0) {
        can_tx_failed_count++;
        return STATUS_FAILED;
    }
    can_tx_count++;

    // Register the pending request with callback
    for (int i = 0; i < MAX_PENDING_QUEUE_SIZE; ++i) {
        if (!pendingRequests[i].in_use) {
            pendingRequests[i].motor_id = motor_id;
            pendingRequests[i].cmd_id = cmd_data[0];
            memset(pendingRequests[i].reply_buffer, 0, 8);
            pendingRequests[i].timestamp_us = _micros();
            pendingRequests[i].in_use = true;
            pendingRequests[i].callback = callback;
            return STATUS_OK;
        }
    }

    // If we reach here, it means no free slot was found
    can_tx_declined_bfull_count++;
    return STATUS_FAILED;
}




// Request Multiturn motor position with a callback
int MotorController::RequestMultiturnMotorPosition_withCallback(uint motor_id, MotorReplyCallback callback)
{
    uint8_t cmd_data[8] = {RMD_CAN_CMD_GET_POS_MULTITURN, 0, 0, 0, 0, 0, 0, 0};
    return SendCanMessageWithCallback(motor_id, cmd_data, callback);
}

// Decode Multiturn motor position reply
float MotorController::DecodeMultiturnMotorPositionReply(const uint8_t *data)
{

  int32_t angle_int32 = 0;
  for (int j = 1; j < 5; j++)
    *((uint8_t *)(&angle_int32) + j - 1) = data[j];
  float multiturnAngle_deg = angle_int32 / 100.0f;

  return multiturnAngle_deg;
}


int MotorController::GetMultiturnMotorPosition_blocking(uint motor_id, float &multiturnAngle_deg)
{
  uint8_t cmd_data[8];

  // Initialize the command data
  cmd_data[0] = RMD_CAN_CMD_GET_POS_MULTITURN;
  cmd_data[1] = 0x00;
  cmd_data[2] = 0x00;
  cmd_data[3] = 0x00;
  cmd_data[4] = 0x00;
  cmd_data[5] = 0x00;
  cmd_data[6] = 0x00;
  cmd_data[7] = 0x00;

  int status = SendCanMessageToMotorAndReturnReplyBlocking(motor_id, cmd_data);
  if (status != STATUS_OK)
    return STATUS_FAILED;

  int32_t angle_int32 = 0;
  // Original message format for this protocol version has 7 bytes of int64
  // which will cause issues parsing negative numbers to int64 type due to missing MSB.
  // So instead I am converting it to int32 by taking the 4 LSBs. Works for positive and negative both
  for (int j = 1; j < 5; j++)
    *((uint8_t *)(&angle_int32) + j - 1) = cmd_data[j];

  multiturnAngle_deg = angle_int32 / 100.0; // 0.01 degree per LSB
  // Serial.print("MotAngle: "); Serial.print(multiturnAngle_deg); Serial.println();

  return STATUS_OK;
}


int MotorController::SetMultiturnMotorPosition_blocking(uint motor_id, float multiturnAngle_deg, uint16_t maxSpeed_dps)
{
  int32_t angleControl = (int32_t)(multiturnAngle_deg * 100);

  // Send Speed limited Motor multiturn position command
  uint8_t cmd_data[8] = {
      RMD_CAN_CMD_SET_POS_MULTITURN_SPD_LIM,
      0x00,
      (uint8_t)(maxSpeed_dps),
      (uint8_t)(maxSpeed_dps >> 8),
      (uint8_t)(angleControl),
      (uint8_t)(angleControl >> 8),
      (uint8_t)(angleControl >> 16),
      (uint8_t)(angleControl >> 24)};

  int status = SendCanMessageToMotorAndReturnReplyBlocking(motor_id, cmd_data);
  if (status != STATUS_OK)
    return STATUS_FAILED;

  return STATUS_OK;
}


int MotorController::SetMultiturnMotorPosition_withCallback(uint motor_id, float multiturnAngle_deg, uint16_t maxSpeed_dps,  MotorReplyCallback callback)
{
  int32_t angleControl = (int32_t)(multiturnAngle_deg * 100);

  // Send Speed limited Motor multiturn position command
  uint8_t cmd_data[8] = {
      RMD_CAN_CMD_SET_POS_MULTITURN_SPD_LIM,
      0x00,
      (uint8_t)(maxSpeed_dps),
      (uint8_t)(maxSpeed_dps >> 8),
      (uint8_t)(angleControl),
      (uint8_t)(angleControl >> 8),
      (uint8_t)(angleControl >> 16),
      (uint8_t)(angleControl >> 24)};

  return SendCanMessageWithCallback(motor_id, cmd_data, callback);
}


int MotorController::SetMotorVelocityClosedLoop_blocking(uint motor_id, float speed_dps)
{
  // Convert speed from degrees per second to the format used by the motor controller
  // Speed in dps, 0.01 dps per LSB
  // For example, if speed_dps = 100.0, then speedControl = 10000
  int32_t speedControl = (int32_t)(speed_dps * 100);

  // Send Speed control command
  uint8_t cmd_data[8] = {
      RMD_CAN_CMD_SET_VELOCITY,
      0x00,
      0x00,
      0x00,
      *(uint8_t *)(&speedControl),
      *((uint8_t *)(&speedControl) + 1),
      *((uint8_t *)(&speedControl) + 2),
      *((uint8_t *)(&speedControl) + 3) // Speed in dps, 0.01 dps per LSB
  };

  int status = SendCanMessageToMotorAndReturnReplyBlocking(motor_id, cmd_data);
  if (status != STATUS_OK)
    return STATUS_FAILED;

  return STATUS_OK;
}

int MotorController::SetMotorVelocityClosedLoop_withCallback(uint motor_id, float speed_dps,  MotorReplyCallback callback)
{
    // Convert speed from degrees per second to the format used by the motor controller
  // Speed in dps, 0.01 dps per LSB
  // For example, if speed_dps = 100.0, then speedControl = 10000
  int32_t speedControl = (int32_t)(speed_dps * 100);

  // Send Speed control command
  uint8_t cmd_data[8] = {
      RMD_CAN_CMD_SET_VELOCITY,
      0x00,
      0x00,
      0x00,
      *(uint8_t *)(&speedControl),
      *((uint8_t *)(&speedControl) + 1),
      *((uint8_t *)(&speedControl) + 2),
      *((uint8_t *)(&speedControl) + 3) // Speed in dps, 0.01 dps per LSB
  };

  return SendCanMessageWithCallback(motor_id, cmd_data, callback);
}


int MotorController::SetMotorTorqueCurrentClosedLoop_blocking(uint motor_id, float torque_current_A)
{
  // The host sends the command to control torque current output of the motor. Iq Control is int16_t type, the value
  // range: -2000~2000, corresponding to the actual torque current range -32A~32A (the bus current and the actual
  // torque of the motor vary with different motors)

  // Convert torque from Amperes to the format used by the motor controller
  int16_t iqControl = (int16_t)(torque_current_A * 62.5); // 2000/32 = 62.5

  // Limit the iqControl to the range -2000 to 2000
  if (iqControl < -2000)
    iqControl = -2000;
  else if (iqControl > 2000)
    iqControl = 2000;

  // Send Torque control command
  uint8_t cmd_data[8] = {
      RMD_CAN_CMD_SET_TORQUE_CURRENT,
      0x00,
      0x00,
      0x00,
      *(uint8_t *)(&iqControl),
      *((uint8_t *)(&iqControl) + 1),
      0x00,
      0x00};

  int status = SendCanMessageToMotorAndReturnReplyBlocking(motor_id, cmd_data);
  if (status != STATUS_OK)
    return STATUS_FAILED;

  return STATUS_OK;
}

int MotorController::SetMotorTorqueCurrentClosedLoop_withCallback(uint motor_id, float torque_current_A,  MotorReplyCallback callback)
{
  // The host sends the command to control torque current output of the motor. Iq Control is int16_t type, the value
  // range: -2000~2000, corresponding to the actual torque current range -32A~32A (the bus current and the actual
  // torque of the motor vary with different motors)

  // Convert torque from Amperes to the format used by the motor controller
  int16_t iqControl = (int16_t)(torque_current_A * 62.5); // 2000/32 = 62.5

  // Limit the iqControl to the range -2000 to 2000
  if (iqControl < -2000)
    iqControl = -2000;
  else if (iqControl > 2000)
    iqControl = 2000;

  // Send Torque control command
  uint8_t cmd_data[8] = {
      RMD_CAN_CMD_SET_TORQUE_CURRENT,
      0x00,
      0x00,
      0x00,
      *(uint8_t *)(&iqControl),
      *((uint8_t *)(&iqControl) + 1),
      0x00,
      0x00};

  return SendCanMessageWithCallback(motor_id, cmd_data, callback);
}

void MotorController::PrintCanLinkStats()
{
  Serial.print("CAN Link Stats:: ");
  Serial.print("Sent: ");
  Serial.print(can_tx_count);
  Serial.print(", Rx: ");
  Serial.print(can_rx_count);
  Serial.print(", Rx Good: ");
  Serial.print(can_rx_good_count);
  Serial.print(", Tx Failed: ");
  Serial.print(can_tx_failed_count);
  Serial.print(", Decl(pend): ");
  Serial.print(can_tx_declined_pending_count);
  Serial.print(", Decl(bfull): ");
  Serial.print(can_tx_declined_bfull_count);
  Serial.print(", Not Rcvd: ");
  Serial.print(can_no_rep_count);
  Serial.print(", Rx Unknown: ");
  Serial.println(can_rx_unknown);
}
