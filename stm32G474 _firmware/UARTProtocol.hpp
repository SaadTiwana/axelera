#ifndef UART_PROTOCOL_HPP
#define UART_PROTOCOL_HPP

/*
Binary Protocol Specification
Message Frame Structure
[START] [CMD] [LEN] [DATA] [CHECKSUM] [END]


START: 0xAA (1 byte) - Frame start marker
CMD: Command/Response ID (1 byte) - See command table below
LEN: Data length (1 byte) - Number of bytes in DATA field (0-255)
DATA: Variable length payload (0-255 bytes)
CHECKSUM: CRC-8 (1 byte) - Calculated over CMD, LEN, and DATA
END: 0xBB (1 byte) - Frame end marker

Command IDs:
ID	    Direction	Description	Data Format
0x01	SBC → MCU	GET_ANGLES	None
0x02	SBC → MCU	SET_ANGLES	2 floats (pitch, yaw) - little-endian
0x03	SBC → MCU	HEARTBEAT_PING	None
0x04	SBC → MCU	SET_STABILIZATION_ON	None
0x05	SBC → MCU	SET_STABILIZATION_OFF	None
0x81	MCU → SBC	ANGLES_RESPONSE	2 floats (pitch, yaw)
0x82	MCU → SBC	SET_ANGLES_RESPONSE	1 byte (0=OK, 1=ERROR_INVALID_ANGLE, 2=ERROR_MOTOR_FAULT)
0x83	MCU → SBC	HEARTBEAT_PONG	None
0xFF	MCU → SBC	ERROR	1 byte error code + optional message

Error Codes:
0x01: INVALID_COMMAND
0x02: INVALID_DATA_LENGTH
0x03: CHECKSUM_ERROR
0x05: TIMEOUT
0x06: MOTOR_OUT_OF_RANGE
0x07: DATA_STALE
CRC-8 Implementation
Using CRC-8-CCITT polynomial: x^8 + x^2 + x + 1 (0x07)

*/



#include <Arduino.h>
#include "SystemStatus.hpp"

// Debug configuration
#define DEBUG true
#ifndef DEBUG_SERIAL
    // Default debug serial: use Serial for debug output
    #define DEBUG_SERIAL Serial
#endif

// Debug serial will be a Print pointer (Serial for debug output)

// Protocol version
#define PROTOCOL_VERSION 0x01

// Testing helper: set to 1 to allow returning last-known angles even when data
// is considered stale. Leave 0 for strict behavior.
#ifndef ALLOW_STALE_ANGLES
#define ALLOW_STALE_ANGLES 0
#endif

// Motor position limits (degrees) - Should correspond with limits in main program
//TODO: Sync these limits with main program limits, or implement this check elsewhere
static constexpr float PITCH_MIN = -30.0f;
static constexpr float PITCH_MAX = 90.0f;
static constexpr float YAW_MIN = -180.0f;
static constexpr float YAW_MAX = 180.0f;


// Protocol constants and timeouts
static constexpr size_t MAX_FRAME_SIZE = 512;
static constexpr size_t MAX_DATA_LEN = 255;
static constexpr uint32_t FRAME_TIMEOUT_MS = 500;    // Increased timeout for testing
static constexpr uint32_t MAX_DATA_AGE_US = 20000;   // 20ms - maximum age for angle data

// Protocol bytes
static constexpr uint8_t START_BYTE = 0xAA;
static constexpr uint8_t END_BYTE = 0xBB;

// Command IDs
static constexpr uint8_t RSP_OK = 0x00;
static constexpr uint8_t CMD_GET_ANGLES = 0x01;
static constexpr uint8_t CMD_SET_ANGLES = 0x02;
static constexpr uint8_t CMD_HEARTBEAT_PING = 0x03;
static constexpr uint8_t CMD_SET_STABILIZATION_ON = 0x04;
static constexpr uint8_t CMD_SET_STABILIZATION_OFF = 0x05;
static constexpr uint8_t RSP_ANGLES = 0x81;
static constexpr uint8_t RSP_SET_ANGLES = 0x82;
static constexpr uint8_t RSP_HEARTBEAT_PONG = 0x83;
static constexpr uint8_t RSP_ERROR = 0xFF;

// Error codes
enum class ProtocolError : uint8_t {
    NONE = 0x00,
    INVALID_COMMAND = 0x01,
    INVALID_DATA_LENGTH = 0x02,
    CHECKSUM_ERROR = 0x03,
    TIMEOUT = 0x05,
    MOTOR_OUT_OF_RANGE = 0x06,
    DATA_STALE = 0x07,
    BUFFER_OVERFLOW = 0x08
};

// Protocol statistics
struct ProtocolStats {
    uint32_t frames_received = 0;
    uint32_t frames_sent = 0;
    uint32_t errors = 0;
    uint32_t timeouts = 0;
    uint32_t checksum_errors = 0;
    uint32_t buffer_overflows = 0;
    uint32_t unknown_commands = 0;

    void reset() {
        frames_received = 0;
        frames_sent = 0;
        errors = 0;
        timeouts = 0;
        checksum_errors = 0;
        buffer_overflows = 0;
        unknown_commands = 0;
    }
};

// CRC-8 table for polynomial x^8 + x^2 + x + 1 (0x07)
const uint8_t crc8_table[256] = {
  0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15,
  0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
  0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65,
  0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
  0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5,
  0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
  0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85,
  0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
  0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2,
  0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
  0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2,
  0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
  0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32,
  0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
  0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42,
  0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
  0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C,
  0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
  0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC,
  0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
  0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C,
  0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
  0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C,
  0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
  0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B,
  0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
  0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B,
  0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
  0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB,
  0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
  0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB,
  0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};

class UARTProtocol {
private:
    // Static buffer for CRC calculations
    static inline uint8_t crc_data[MAX_DATA_LEN + 3];

    // Communication interfaces
    HardwareSerial &serial;
    Print *debug_serial;
    SystemStatus &systemStatus;
    
    // Protocol state
    uint8_t buffer[MAX_FRAME_SIZE];
    size_t buffer_pos = 0;
    uint32_t last_byte_time = 0;

    // Statistics
    ProtocolStats stats;

    // Helper functions
    uint8_t crc8(const uint8_t *data, size_t len) {
        uint8_t crc = 0x00;
        for (size_t i = 0; i < len; ++i) {
            crc = crc8_table[crc ^ data[i]];
        }
        return crc;
    }

    void resetBuffer() {
        if (DEBUG && buffer_pos > 0 && debug_serial) {
            debug_serial->println("Resetting buffer");
        }
        buffer_pos = 0;
        last_byte_time = 0;
    }

    bool validateFrame(size_t data_len) {
        const size_t min_frame_size = 5;  // START + CMD + LEN + CRC + END
        const size_t expected_size = min_frame_size + data_len;

        if (DEBUG && debug_serial) {
            debug_serial->println("Validating frame:");
            debug_serial->print("  START_BYTE check: ");
            debug_serial->println(buffer[0] == START_BYTE ? "OK" : "FAIL");
            debug_serial->print("  Data length check (");
            debug_serial->print(data_len);
            debug_serial->print(" <= ");
            debug_serial->print(MAX_DATA_LEN);
            debug_serial->println("): ");
            debug_serial->println(data_len <= MAX_DATA_LEN ? "OK" : "FAIL");
            debug_serial->print("  Buffer size check (");
            debug_serial->print(buffer_pos);
            debug_serial->print(" == ");
            debug_serial->print(expected_size);
            debug_serial->println("): ");
            debug_serial->println(buffer_pos == expected_size ? "OK" : "FAIL");
            debug_serial->print("  END_BYTE check: ");
            debug_serial->println(buffer[buffer_pos - 1] == END_BYTE ? "OK" : "FAIL");
        }

        // Frame must start with START_BYTE
        if (buffer[0] != START_BYTE) return false;

        // Data length must be valid
        if (data_len > MAX_DATA_LEN) return false;

        // Frame must be complete (START + CMD + LEN + DATA + CRC + END)
        if (buffer_pos != expected_size) return false;

        // Frame must end with END_BYTE
        if (buffer[buffer_pos - 1] != END_BYTE) return false;

        return true;
    }

    void handleFrameTimeout() {
        if (buffer_pos > 0 && (millis() - last_byte_time) > FRAME_TIMEOUT_MS) {
            if (DEBUG) {
                if (debug_serial) {
                    debug_serial->print("Frame timeout after ");
                    debug_serial->print(millis() - last_byte_time);
                    debug_serial->println("ms with incomplete frame:");
                    for (size_t i = 0; i < buffer_pos; i++) {
                        debug_serial->print("0x");
                        debug_serial->print(buffer[i], HEX);
                        debug_serial->print(" ");
                    }
                    debug_serial->println();
                }
            }
            stats.timeouts++;
            resetBuffer();
        }
    }

    void sendFrame(uint8_t cmd, const uint8_t *data, size_t len) {
        static uint8_t frame[MAX_FRAME_SIZE];  // Static to avoid stack allocation
        size_t pos = 0;

        if (len > MAX_DATA_LEN || (6 + len) > MAX_FRAME_SIZE) {
            stats.errors++;
            return;
        }

        frame[pos++] = START_BYTE;
        frame[pos++] = cmd;
        frame[pos++] = len;

        if (data && len > 0) {
            memcpy(&frame[pos], data, len);
            pos += len;
        }

        // Calculate CRC over CMD, LEN, DATA
        uint8_t crc = crc8(&frame[1], len + 2);
        frame[pos++] = crc;
        frame[pos++] = END_BYTE;

        // Print out the reply being sent
        if (DEBUG && debug_serial)
        {
            debug_serial->print("UART TX Frame: ");
            for (size_t i = 0; i < pos; ++i)
            {
                if (i > 0)
                    debug_serial->print(" ");
                debug_serial->print("0x");
                if (frame[i] < 0x10)
                    debug_serial->print("0");
                debug_serial->print(frame[i], HEX);
            }
            debug_serial->println();
        }

        serial.write(frame, pos);
        stats.frames_sent++;
    }

    void sendError(ProtocolError error) {
        uint8_t error_code = static_cast<uint8_t>(error);
        sendFrame(RSP_ERROR, &error_code, 1);
        stats.errors++;
    }

    void processCommand(uint8_t cmd, const uint8_t *data, size_t len) {
        if (DEBUG && debug_serial) {
            debug_serial->print("Processing command: 0x");
            debug_serial->print(cmd, HEX);
            debug_serial->print(" with ");
            debug_serial->print(len);
            debug_serial->println(" bytes of data");
        }

        switch (cmd) {
            case CMD_GET_ANGLES: {
                if (DEBUG && debug_serial) debug_serial->println("Recvd: CMD_GET_ANGLES");
                
                // Check data freshness (can be relaxed via ALLOW_STALE_ANGLES for testing)
#if ALLOW_STALE_ANGLES
                (void)MAX_DATA_AGE_US; // silence unused warning when macro enabled
#else
                if (systemStatus.mot_angle_pitch_deg.isStale(MAX_DATA_AGE_US) || 
                    systemStatus.mot_angle_yaw_deg.isStale(MAX_DATA_AGE_US)) {
                    sendError(ProtocolError::DATA_STALE);
                    return;
                }
#endif

                // Send current angles
                float angles[2] = {
                    systemStatus.mot_angle_pitch_deg.value,
                    systemStatus.mot_angle_yaw_deg.value
                };
                sendFrame(RSP_ANGLES, reinterpret_cast<uint8_t*>(angles), sizeof(angles));
                break;
            }

            case CMD_SET_ANGLES: {
                if (DEBUG && debug_serial) {
                    debug_serial->print("Recvd: CMD_SET_ANGLES (0x");
                    debug_serial->print(CMD_SET_ANGLES, HEX);
                    debug_serial->print(") LEN=");
                    debug_serial->println(len);
                }
                
                if (len != 8) {
                    sendError(ProtocolError::INVALID_DATA_LENGTH);
                    return;
                }

                float pitch, yaw;
                // Handle potential endianness differences
                #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
                    // Swap bytes if we're on a big-endian system
                    uint32_t temp;
                    memcpy(&temp, data, 4);
                    temp = __builtin_bswap32(temp);
                    memcpy(&pitch, &temp, 4);
                    
                    memcpy(&temp, data + 4, 4);
                    temp = __builtin_bswap32(temp);
                    memcpy(&yaw, &temp, 4);
                #else
                    // Little-endian system, direct copy
                    memcpy(&pitch, data, 4);
                    memcpy(&yaw, data + 4, 4);
                #endif
                
                // Validate angle ranges 
                
                if (pitch < PITCH_MIN || pitch > PITCH_MAX ||
                    yaw < YAW_MIN || yaw > YAW_MAX) {
                    sendError(ProtocolError::MOTOR_OUT_OF_RANGE);
                    return;
                }
                
                // Update system status with new target angles
                systemStatus.mot_angle_pitch_deg_target = pitch;
                systemStatus.mot_angle_yaw_deg_target = yaw;

                if (DEBUG && debug_serial) {
                    debug_serial->print("Setting angles - Pitch: ");
                    debug_serial->print(pitch);
                    debug_serial->print(", Yaw: ");
                    debug_serial->println(yaw);
                }
                
                uint8_t status = 0;  // OK - command accepted
                sendFrame(RSP_SET_ANGLES, &status, 1);
                break;
            }

            case CMD_HEARTBEAT_PING: {
                if (DEBUG && debug_serial) {
                    debug_serial->println("Recvd: CMD_HEARTBEAT_PING");
                    debug_serial->println("Sending PONG response...");
                }
                // Send response immediately
                sendFrame(RSP_HEARTBEAT_PONG, nullptr, 0);
                if (DEBUG && debug_serial) debug_serial->println("PONG sent");
                break;
            }

            case CMD_SET_STABILIZATION_ON: {
                if (DEBUG && debug_serial) debug_serial->println("Recvd: CMD_SET_STABILIZATION_ON");
                systemStatus.stabilize = true;
                uint8_t status = 0;  // OK
                sendFrame(RSP_OK, &status, 1);
                break;
            }

            case CMD_SET_STABILIZATION_OFF: {
                if (DEBUG && debug_serial) debug_serial->println("Recvd: CMD_SET_STABILIZATION_OFF");
                systemStatus.stabilize = false;
                uint8_t status = 0;  // OK
                sendFrame(RSP_OK, &status, 1);
                break;
            }

            default: {
                if (DEBUG && debug_serial) debug_serial->println("Recvd: INVALID_COMMAND");
                stats.unknown_commands++;
                sendError(ProtocolError::INVALID_COMMAND);
            }
        }
    }

public:
    UARTProtocol(HardwareSerial &s, SystemStatus &ss, Print *debug = nullptr)
        : serial(s), debug_serial(nullptr), systemStatus(ss) {
            // Choose a sane default debug serial if none provided.
#if defined(ARDUINO_ARCH_STM32)
            if (debug) debug_serial = debug; else debug_serial = &Serial;
#else
            if (debug) debug_serial = debug; else debug_serial = &Serial;
#endif
        }

    void begin(unsigned long baud) {
        serial.begin(baud);
        resetBuffer();
    }

    void update() {
        handleFrameTimeout();
        
        const int MAX_BYTES_PER_UPDATE = 64;
        int bytes_processed = 0;
        
        while (serial.available() && bytes_processed < MAX_BYTES_PER_UPDATE) {
            bytes_processed++;
            uint8_t byte = serial.read();
            last_byte_time = millis();

            if (DEBUG && debug_serial) {
                debug_serial->print("Received byte: 0x");
                debug_serial->println(byte, HEX);
            }

            // If we see a START_BYTE, always start a new frame
            if (byte == START_BYTE) {
                if (buffer_pos > 0) {
                    if (DEBUG) {
                        if (debug_serial) debug_serial->println("Found new START_BYTE while collecting frame, starting new frame");
                    }
                }
                resetBuffer();
                buffer[buffer_pos++] = byte;
                continue;
            }

            // If we haven't seen a START_BYTE yet, ignore this byte
            if (buffer_pos == 0) {
                if (DEBUG && debug_serial) debug_serial->println("Ignoring byte before START_BYTE");
                continue;
            }

            // Protect against buffer overflow
                if (buffer_pos >= MAX_FRAME_SIZE) {
                if (DEBUG && debug_serial) debug_serial->println("Buffer overflow detected");
                stats.buffer_overflows++;
                resetBuffer();
                continue;
            }

            // Add byte to buffer
            buffer[buffer_pos++] = byte;
            
                if (DEBUG && debug_serial) {
                    debug_serial->print("Buffer now contains ");
                    debug_serial->print(buffer_pos);
                    debug_serial->print(" bytes: ");
                    for (size_t i = 0; i < buffer_pos; i++) {
                        debug_serial->print("0x");
                        debug_serial->print(buffer[i], HEX);
                        debug_serial->print(" ");
                    }
                    debug_serial->println();
                }

            if (byte == END_BYTE) {
                stats.frames_received++;
                if (buffer_pos < 5) {  // Minimum frame size (START + CMD + LEN + CRC + END)
                    if (DEBUG && debug_serial) debug_serial->println("Frame too short");
                    stats.errors++;
                    resetBuffer();
                    continue;
                }

                size_t data_len = buffer[2];  // LEN is at position 2 (START, CMD, LEN)
                const size_t min_frame_size = 5;  // START + CMD + LEN + CRC + END
                const size_t expected_size = min_frame_size + data_len;

                if (DEBUG && debug_serial) {
                    debug_serial->print("Processing frame - Data length: ");
                    debug_serial->print(data_len);
                    debug_serial->print(", Expected size: ");
                    debug_serial->print(expected_size);
                    debug_serial->print(", Actual size: ");
                    debug_serial->println(buffer_pos);
                }

                if (!validateFrame(data_len)) {
                    if (DEBUG && debug_serial) debug_serial->println("Frame validation failed");
                    stats.errors++;
                    resetBuffer();
                    continue;
                }

                // Copy data for CRC check (CMD + LEN + DATA)
                memcpy(crc_data, &buffer[1], 2 + data_len);
                uint8_t calculated_crc = crc8(crc_data, 2 + data_len);
                uint8_t received_crc = buffer[3 + data_len];  // CRC is at position 3 + data_len

                if (DEBUG && debug_serial) {
                    debug_serial->print("CRC validation - Data (");
                    debug_serial->print(2 + data_len);
                    debug_serial->print(" bytes): ");
                    for (size_t i = 0; i < 2 + data_len; i++) {
                        debug_serial->print("0x");
                        debug_serial->print(crc_data[i], HEX);
                        debug_serial->print(" ");
                    }
                    debug_serial->println();
                    debug_serial->print("CRC check: calculated=0x");
                    debug_serial->print(calculated_crc, HEX);
                    debug_serial->print(", received=0x");
                    debug_serial->println(received_crc, HEX);
                }

                if (calculated_crc == received_crc) {
                    if (DEBUG && debug_serial) {
                        debug_serial->println("CRC validation passed");
                    }

                    // Process command (CMD is at position 1, DATA starts at position 3)
                    processCommand(buffer[1], &buffer[3], data_len);
                } else {
                    if (DEBUG && debug_serial) debug_serial->println("CRC error, sending error response");
                    stats.checksum_errors++;
                    sendError(ProtocolError::CHECKSUM_ERROR);
                }
                resetBuffer();
            }
        }
    }

    // Get protocol statistics
    ProtocolStats getStats() {
        return stats;
    }

    // Reset statistics
    void resetStats() {
        stats.reset();
    }
};

#endif