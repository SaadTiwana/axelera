"""
UART Communication Module for STM32 Gimbal Control
"""
import serial
import struct
import time
import threading
from typing import Optional, Tuple, Callable
from config import ConfigManager


# Protocol constants (matching UARTProtocol.hpp)
START_BYTE = 0xAA
END_BYTE = 0xBB

# Command IDs
RSP_OK = 0x00
CMD_GET_ANGLES = 0x01
CMD_SET_ANGLES = 0x02
CMD_HEARTBEAT_PING = 0x03
CMD_SET_STABILIZATION_ON = 0x04
CMD_SET_STABILIZATION_OFF = 0x05
RSP_ANGLES = 0x81
RSP_SET_ANGLES = 0x82
RSP_HEARTBEAT_PONG = 0x83
RSP_ERROR = 0xFF

# CRC-8 table (matching UARTProtocol.hpp)
crc8_table = [
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
]


def crc8(data: bytes) -> int:
    """Calculate CRC-8 checksum"""
    try:
        if not isinstance(data, (bytes, bytearray)):
            print(f"⚠️ CRC8: invalid data type: {type(data)}")
            return 0

        crc = 0x00
        for byte in data:
            if isinstance(byte, int):
                crc = crc8_table[crc ^ byte]
            else:
                print(f"⚠️ CRC8: invalid byte type: {type(byte)}")
                return 0
        return crc
    except (TypeError, IndexError, ValueError) as e:
        print(f"⚠️ CRC8 calculation error: {e}")
        return 0


class GimbalController:
    """Controls gimbal movement via UART communication with STM32"""

    def __init__(self, config: ConfigManager):
        self.config = config
        self.uart_config = config.uart
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self._lock = threading.Lock()

    def connect(self) -> bool:
        """Establish UART connection with STM32"""
        try:
            self.serial = serial.Serial(
                port=self.uart_config.port,
                baudrate=self.uart_config.baudrate,
                timeout=self.uart_config.timeout
            )
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            # Test connection with heartbeat
            if self._send_frame(CMD_HEARTBEAT_PING):
                self.connected = True
                print("✅ Connected to STM32 gimbal controller")
                print(f"   Port: {self.uart_config.port}")
                print(f"   Baudrate: {self.uart_config.baudrate}")
                print(f"   Timeout: {self.uart_config.timeout}s")
                return True
            else:
                print("❌ STM32 connection test failed")
                return False

        except serial.SerialException as e:
            print(f"❌ Serial connection failed: {e}")
            return False

    def disconnect(self) -> None:
        """Close UART connection"""
        if self.serial:
            self.serial.close()
        self.connected = False

    def get_angles(self) -> Optional[Tuple[float, float]]:
        """Get current gimbal angles (pitch, yaw) with retry logic"""
        if not self.connected:
            return None

        # Get retry configuration
        gimbal_config = self.config.gimbal
        max_retries = getattr(gimbal_config, 'max_retries', 3)

        for attempt in range(max_retries):
            with self._lock:
                cmd, data = self._send_frame(CMD_GET_ANGLES)
                if cmd == RSP_ANGLES and len(data) == 8:
                    try:
                        pitch, yaw = struct.unpack('<ff', data)
                        # Validate that we got reasonable float values
                        if isinstance(pitch, float) and isinstance(yaw, float):
                            return pitch, yaw
                        else:
                            print(f"❌ Invalid angle data types: pitch={type(pitch)}, yaw={type(yaw)}")
                    except (struct.error, ValueError) as e:
                        print(f"❌ Failed to unpack angle data: {e}")
                elif cmd == RSP_ERROR:
                    error_code = data[0] if len(data) > 0 else 0
                    print(f"❌ MCU Error getting angles: 0x{error_code:02x}")
                elif cmd is None:
                    print("❌ Failed to get angles: No response received")
                else:
                    print(f"❌ Unexpected response getting angles: 0x{cmd:02x}")

            if attempt < max_retries - 1:
                print(f"⚠️ Get angles failed on attempt {attempt + 1}, retrying...")
                time.sleep(0.5)

        print("❌ Failed to get angles after all retries")
        return None

    def set_angles(self, pitch: float, yaw: float) -> bool:
        """Set gimbal target angles"""
        if not self.connected:
            return False

        with self._lock:
            data = struct.pack('<ff', pitch, yaw)
            cmd, response_data = self._send_frame(CMD_SET_ANGLES, data)

            if cmd == RSP_SET_ANGLES:
                if len(response_data) > 0 and response_data[0] == 0:
                    return True
                else:
                    print(f"❌ Set angles failed: {response_data[0] if len(response_data) > 0 else 'unknown'}")
            elif cmd == RSP_ERROR:
                error_code = response_data[0] if len(response_data) > 0 else 0
                print(f"❌ MCU Error setting angles: 0x{error_code:02x}")
            elif cmd is None:
                print("❌ Failed to set angles: No response received")
            else:
                print(f"❌ Unexpected response setting angles: 0x{cmd:02x}")

        return False

    def set_stabilization_on(self) -> bool:
        """Turn stabilization on with retry logic"""
        if not self.connected:
            return False

        # Get retry configuration
        gimbal_config = self.config.gimbal
        max_retries = getattr(gimbal_config, 'max_retries', 3)

        for attempt in range(max_retries):
            with self._lock:
                cmd, data = self._send_frame(CMD_SET_STABILIZATION_ON)
                if cmd == RSP_OK and len(data) > 0 and data[0] == 0:
                    return True
                elif cmd == RSP_ERROR:
                    error_code = data[0] if len(data) > 0 else 0
                    print(f"❌ MCU Error turning stabilization on: 0x{error_code:02x}")
                elif cmd is None:
                    print("❌ Failed to turn stabilization on: No response received")
                else:
                    print(f"❌ Failed to turn stabilization on: cmd=0x{cmd:02x}")

            if attempt < max_retries - 1:
                print(f"⚠️ Stabilization ON failed on attempt {attempt + 1}, retrying...")
                time.sleep(0.5)

        print("❌ Failed to turn stabilization on after all retries")
        return False

    def set_stabilization_off(self) -> bool:
        """Turn stabilization off with retry logic"""
        if not self.connected:
            return False

        # Get retry configuration
        gimbal_config = self.config.gimbal
        max_retries = getattr(gimbal_config, 'max_retries', 3)

        for attempt in range(max_retries):
            with self._lock:
                cmd, data = self._send_frame(CMD_SET_STABILIZATION_OFF)
                if cmd == RSP_OK and len(data) > 0 and data[0] == 0:
                    return True
                elif cmd == RSP_ERROR:
                    error_code = data[0] if len(data) > 0 else 0
                    print(f"❌ MCU Error turning stabilization off: 0x{error_code:02x}")
                elif cmd is None:
                    print("❌ Failed to turn stabilization off: No response received")
                else:
                    print(f"❌ Failed to turn stabilization off: cmd=0x{cmd:02x}")

            if attempt < max_retries - 1:
                print(f"⚠️ Stabilization OFF failed on attempt {attempt + 1}, retrying...")
                time.sleep(0.5)

        print("❌ Failed to turn stabilization off after all retries")
        return False

    def move_to_position(self, azimuth: float, elevation: float) -> bool:
        """Move gimbal to specific azimuth/elevation position"""
        try:
            # Validate input parameters
            if not isinstance(azimuth, (int, float)) or azimuth is None:
                print(f"❌ Invalid azimuth value: {azimuth}")
                return False
            if not isinstance(elevation, (int, float)) or elevation is None:
                print(f"❌ Invalid elevation value: {elevation}")
                return False

            # Validate config
            gimbal_config = self.config.gimbal
            if gimbal_config is None:
                print("❌ Gimbal config is None")
                return False

            # Convert azimuth/elevation to pitch/yaw
            # This depends on your gimbal mounting configuration
            pitch = elevation
            yaw = azimuth

            print(f"🎯 Moving to azimuth={azimuth:.1f} deg, elevation={elevation:.1f} deg")

            # Try multiple times with timeout
            for attempt in range(gimbal_config.max_retries):
                try:
                    if self.set_angles(pitch, yaw):
                        # Wait for movement to complete
                        time.sleep(gimbal_config.move_timeout_seconds)

                        # Verify position
                        current_angles = self.get_angles()
                        if current_angles:
                            current_pitch, current_yaw = current_angles
                            # Check if values are valid numbers
                            if (isinstance(current_pitch, (int, float)) and current_pitch is not None and
                                isinstance(current_yaw, (int, float)) and current_yaw is not None):
                                # Check if we're close enough to target
                                pitch_error = abs(current_pitch - pitch)
                                yaw_error = abs(current_yaw - yaw)

                                if pitch_error < 2.0 and yaw_error < 2.0:  # 2° tolerance
                                    print(f"✅ Position reached: pitch={current_pitch:.1f} deg, yaw={current_yaw:.1f} deg")
                                    time.sleep(gimbal_config.settle_time_seconds)  # Settle time
                                    return True
                                else:
                                    print(f"⚠️ Position error: pitch_err={pitch_error:.1f} deg, yaw_err={yaw_error:.1f} deg")
                            else:
                                print(f"⚠️ Invalid angle values received: pitch={current_pitch}, yaw={current_yaw}")
                        else:
                            print("⚠️ Could not verify gimbal position")
                    else:
                        print(f"⚠️ Set angles failed on attempt {attempt + 1}")
                except Exception as attempt_error:
                    print(f"⚠️ Error on attempt {attempt + 1}: {attempt_error}")

                if attempt < gimbal_config.max_retries - 1:
                    print(f"⚠️ Attempt {attempt + 1} failed, retrying...")
                    time.sleep(0.5)

            print("❌ Failed to reach target position")
            return False

        except Exception as e:
            print(f"❌ Unexpected error in move_to_position: {e}")
            return False

    def _send_frame(self, cmd: int, data: bytes = b'') -> Tuple[Optional[int], bytes]:
        """Send frame and receive response"""
        try:
            if not self.serial:
                return None, b''

            # Validate inputs
            if not isinstance(cmd, int) or cmd < 0 or cmd > 255:
                print(f"❌ Invalid command: {cmd}")
                return None, b''

            if not isinstance(data, (bytes, bytearray)):
                print(f"❌ Invalid data type: {type(data)}")
                return None, b''

            # Build frame
            frame = bytearray()
            frame.append(START_BYTE)
            frame.append(cmd)
            frame.append(len(data))
            frame.extend(data)

            # Calculate CRC
            crc_data = bytes([cmd, len(data)]) + data
            frame.append(crc8(crc_data))
            frame.append(END_BYTE)

            # Send frame
            self.serial.write(frame)
            self.serial.flush()

            # Receive response
            return self._read_response()

        except (serial.SerialException, AttributeError, OSError, ValueError) as e:
            print(f"❌ Serial communication error: {e}")
            return None, b''

    def _read_response(self) -> Tuple[Optional[int], bytes]:
        """Read response frame from serial"""
        try:
            start_time = time.time()
            buffer = bytearray()

            while time.time() - start_time < self.uart_config.timeout:
                if self.serial and self.serial.in_waiting > 0:
                    # Read all available bytes
                    available_bytes = self.serial.read(self.serial.in_waiting)

                    if available_bytes:
                        for byte in available_bytes:
                            buffer.append(byte)

                            # Look for complete frame
                            if len(buffer) >= 2 and buffer[-1] == END_BYTE:
                                if len(buffer) >= 5 and buffer[0] == START_BYTE:
                                    return self._process_frame(buffer)

                time.sleep(0.001)

            return None, b''
        except (serial.SerialException, AttributeError, OSError) as e:
            print(f"⚠️ Error reading response: {e}")
            return None, b''

    def _process_frame(self, buffer: bytearray) -> Tuple[Optional[int], bytes]:
        """Process received frame"""
        try:
            if len(buffer) < 5:
                return None, b''

            start = buffer[0]
            cmd = buffer[1]
            data_len = buffer[2]

            # Validate basic frame structure
            if start != START_BYTE:
                return None, b''

            # Validate data length
            if data_len < 0 or len(buffer) < 5 + data_len:
                return None, b''

            # Extract data
            data = buffer[3:3+data_len]

            # Extract CRC and END byte
            crc_received = buffer[3+data_len]
            end = buffer[3+data_len+1] if 3+data_len+1 < len(buffer) else 0

            # Validate END byte
            if end != END_BYTE:
                return None, b''

            # Calculate and validate CRC
            crc_data = bytes([cmd, data_len]) + data
            crc_calculated = crc8(crc_data)

            if crc_calculated != crc_received:
                print(f"⚠️ CRC mismatch: calculated=0x{crc_calculated:02x}, received=0x{crc_received:02x}")
                return None, b''

            return cmd, data
        except (IndexError, ValueError, TypeError) as e:
            print(f"⚠️ Error processing frame: {e}")
            return None, b''


def test_gimbal_interface():
    """Test function for gimbal UART interface"""
    print("🎯 Starting Gimbal UART Interface Test")
    print("=" * 50)

    # Create configuration (you may need to adjust these values)
    class TestConfig:
        class UART:
            port = "/dev/ttyUSB0"  # Adjust this to match your system
            baudrate = 115200
            timeout = 2.0

        class Gimbal:
            max_retries = 3
            move_timeout_seconds = 5.0
            settle_time_seconds = 1.0

        uart = UART()
        gimbal = Gimbal()

    config = TestConfig()

    # Initialize gimbal controller
    print("🔌 Connecting to gimbal...")
    gimbal = GimbalController(config)

    try:
        if not gimbal.connect():
            print("❌ Failed to connect to gimbal")
            return

        print("✅ Connected to gimbal successfully")
        print()

        # Test 1: Send heartbeat and verify response
        print("1️⃣ Testing Heartbeat...")
        cmd, data = gimbal._send_frame(CMD_HEARTBEAT_PING)
        if cmd == RSP_HEARTBEAT_PONG:
            print("✅ Heartbeat successful - received PONG")
        elif cmd is None:
            print("❌ Heartbeat failed - No response received")
        else:
            print(f"❌ Heartbeat failed - received cmd=0x{cmd:02x}")
        print()

        # Test 2: Get motor angles
        print("2️⃣ Testing Get Angles...")
        angles = gimbal.get_angles()
        if angles:
            pitch, yaw = angles
            print(f"✅ Angles retrieved: Pitch={pitch:.2f} deg, Yaw={yaw:.2f} deg")
        else:
            print("❌ Failed to get angles")
        print()

        # Test 3: Turn on stabilization
        print("3️⃣ Turning ON stabilization...")
        if gimbal.set_stabilization_on():
            print("✅ Stabilization turned ON")
        else:
            print("❌ Failed to turn ON stabilization")
        print()

        # Test 4: Move to angles 5,5
        print("4️⃣ Moving to angles (5 deg, 5 deg)...")
        if gimbal.set_angles(5.0, 5.0):
            print("✅ Move command sent successfully")
            time.sleep(2)  # Wait for movement
            # Verify position
            current_angles = gimbal.get_angles()
            if current_angles:
                pitch, yaw = current_angles
                print(f"📍 Current position: Pitch={pitch:.2f} deg, Yaw={yaw:.2f} deg")
        else:
            print("❌ Failed to send move command")
        print()

        # Test 5: Move to angles 0,0
        print("5️⃣ Moving to angles (0 deg, 0 deg)...")
        if gimbal.set_angles(0.0, 0.0):
            print("✅ Move command sent successfully")
            time.sleep(2)  # Wait for movement
            # Verify position
            current_angles = gimbal.get_angles()
            if current_angles:
                pitch, yaw = current_angles
                print(f"📍 Current position: Pitch={pitch:.2f} deg, Yaw={yaw:.2f} deg")
        else:
            print("❌ Failed to send move command")
        print()

        # Test 6: Turn off stabilization
        print("6️⃣ Turning OFF stabilization...")
        if gimbal.set_stabilization_off():
            print("✅ Stabilization turned OFF")
        else:
            print("❌ Failed to turn OFF stabilization")
        print()

        print("🎉 Gimbal interface test completed!")

    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("🔌 Disconnecting from gimbal...")
        gimbal.disconnect()
        print("✅ Disconnected")


# Run test if this file is executed directly
if __name__ == "__main__":
    test_gimbal_interface()