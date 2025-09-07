//-----------------------------------------------------------------
// This demo runs on WeActStudio STM32G474CEU6 board
// https://github.com/WeActStudio/WeActStudio.STM32G474CoreBoard
// The can clock frequency is 170 MHz.
// The 3 FDCAN modules are configured in external loop back mode: it
// internally receives every CAN frame it sends, and emitted frames
// can be observed on TxCAN pins. No external hardware is required.
// Default TxCAN pins are used:
//   - fdcan1: PA_12
//   - fdcan2: PB_6
//   - fdcan3: PA_15
//-----------------------------------------------------------------

#ifndef ARDUINO_WEACT_G474CE
  #error This sketch runs on WeActStudio STM32G474CEU6 board
#endif

//-----------------------------------------------------------------
// IMPORTANT:
//   <ACANFD_STM32.h> should be included only once in a sketch, generally from the .ino file
//   From an other file, include <ACANFD_STM32_from_cpp.h>
//-----------------------------------------------------------------

//#include <ACANFD_STM32.h>
#include <ACANFD_STM32_from_cpp.h>
extern ACANFD_STM32 fdcan2; // Since the <ACANFD_STM32.h> is included from .ino sketch
//-----------------------------------------------------------------

#ifndef MOTOR_CONTROLLER_HPP
#define MOTOR_CONTROLLER_HPP


#include <Arduino.h>
#include "time_utils.hpp"

typedef void (*MotorReplyCallback)(uint motor_id, int status, const uint8_t* data);


class MotorController {
public:
    // CAN IDs
    static constexpr uint16_t MOT1_CAN_ID = 0x140 + 1;
    static constexpr uint16_t MOT2_CAN_ID = 0x140 + 2;

    // Canonical Names for motors
    #define MOT_PITCH MOT1_CAN_ID
    #define MOT_YAW   MOT2_CAN_ID

    // Command codes
    static constexpr uint8_t RMD_CAN_CMD_SET_POS_MULTITURN_SPD_LIM = 0xA4;
    static constexpr uint8_t RMD_CAN_CMD_GET_POS_MULTITURN = 0x92;
    static constexpr uint8_t RMD_CAN_CMD_SET_TORQUE_CURRENT = 0xA1;
    static constexpr uint8_t RMD_CAN_CMD_SET_VELOCITY = 0xA2;

    bool DEBUG_MOT_CTRL = true; // Set to true to enable debug prints

    // Status codes
    static constexpr int STATUS_OK = 0;
    static constexpr int STATUS_FAILED = -1;

    static constexpr int STATUS_TIMEOUT = -2; // Timeout waiting for reply


    MotorController() = default;
    void begin();    

    static float DecodeMultiturnMotorPositionReply(const uint8_t *data);

    int GetMultiturnMotorPosition_blocking(uint motor_id, float& multiturnAngle_deg);
    int SetMultiturnMotorPosition_blocking(uint motor_id, float multiturnAngle_deg, uint16_t maxSpeed_dps);
    int SetMotorTorqueCurrentClosedLoop_blocking(uint motor_id, float torque_current_A);
    int SetMotorVelocityClosedLoop_blocking(uint motor_id, float speed_dps);
    
    void processCanReplies(); 
    int SendCanMessageWithCallback(uint motor_id, uint8_t* cmd_data, MotorReplyCallback callback);
    int RequestMultiturnMotorPosition_withCallback(uint motor_id, MotorReplyCallback callback);
    int SetMotorTorqueCurrentClosedLoop_withCallback(uint motor_id, float torque_current_A,  MotorReplyCallback callback);
    int SetMultiturnMotorPosition_withCallback(uint motor_id, float multiturnAngle_deg, uint16_t maxSpeed_dps,  MotorReplyCallback callback);
    int SetMotorVelocityClosedLoop_withCallback(uint motor_id, float speed_dps,  MotorReplyCallback callback);

    void PrintCanLinkStats();

  private:
    // Pending CAN requests structure and Queue
    // This structure is used to manage pending CAN requests to motors.
    struct PendingCanRequest {
        bool in_use;
        uint motor_id;
        uint8_t cmd_id;
        uint32_t timestamp_us;
        uint8_t reply_buffer[8];
        MotorReplyCallback callback;
    };

    static constexpr uint8_t MAX_PENDING_QUEUE_SIZE = 10;              // Maximum number of pending requests
    static PendingCanRequest pendingRequests[MAX_PENDING_QUEUE_SIZE]; // Adjust size as needed

    // Maximum age of a request in queue before it is considered stale (timeout occurs)
    // IMPORTANT: This MUST be bigger than the time it takes to receive reply from a motor
    static constexpr uint32_t SETTING_MAX_REQUEST_AGE_US = 50000; 

    // CAN link statistics
    uint32_t can_tx_count = 0;           // Count of transmitted CAN messages
    uint32_t can_rx_count = 0;           // Count of received CAN messages
    uint32_t can_rx_good_count = 0;      // Count of received CAN messages that are recognized as replies to pending requests
    uint32_t can_tx_failed_count = 0;    // Count of failed CAN transmissions
    uint32_t can_tx_declined_pending_count = 0; 
    uint32_t can_tx_declined_bfull_count = 0;    // Count of CAN messages that were not sent due to no free slot in the queue  

    uint32_t can_no_rep_count = 0;       // Count of CAN msgs that didn't receive reply within timeout)
    uint32_t can_rx_unknown = 0;         // Count of received CAN messages that are not recognized as replies to any pending request

    int SendCanMessageToMotorAndReturnReplyBlocking(uint motor_id, uint8_t *cmd_data);

    // Callback functions for Asynchronous replies (so we don't keep waiting for CAN replies which can take up to 2ms)
    static void HandleMultiturnMotorPositionReply(const uint8_t* data, void* userData, int status);

};

#endif // MOTOR_CONTROLLER_HPP

// Move all function implementations to the .cpp file



