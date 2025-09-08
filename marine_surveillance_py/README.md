# Maritime Surveillance System

A robust Python-based maritime surveillance system running on RK3588 with Axelera Metis M.2 AI accelerator, featuring real-time object detection, precise gimbal control, and intelligent restricted area monitoring with audio alerts.

## 🏗️ System Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   STM32 Gimbal  │◄──►│   RK3588 SBC    │◄──►│  USB Camera     │
│   Controller    │    │  ┌────────────┐ │    │  (/dev/video20) │
└─────────────────┘    │  │ Axelera     │ │    └─────────────────┘
                       │  │ Metis M.2   │ │
                       │  │ AI Engine   │ │
                       │  └────────────┘ │
                       └─────────────────┘
```

## 📁 Project Structure

```
Axelera_program/
├── main.py                 # 🚀 Main application orchestrator with alarm management
├── config.py              # ⚙️ Configuration management and validation
├── uart_comm.py           # 📡 UART communication with STM32 gimbal
├── scan_planner.py        # 🗺️ FOV-based scan planning with serpentine patterns
├── camera.py              # 📷 Camera capture with buffer management
├── detector.py            # 🎯 AI object detection with coordinate conversion
├── logger.py              # 📊 Detection logging and data persistence
├── monitor.py             # 🚨 Restricted area monitoring with audio alerts
├── settings.json          # 🔧 Configuration file with current settings
├── requirements.txt       # 📦 Python dependencies
├── documentation/         # 📚 System documentation and diagrams
├── utilities/             # 🛠️ Testing and utility scripts
└── README.md              # 📖 This documentation
```

## 🚀 Quick Start

### Prerequisites

1. **Hardware Requirements:**
   - RK3588 single-board computer
   - Axelera Metis M.2 AI accelerator
   - USB camera (tested with /dev/video20)
   - STM32-based gimbal controller
   - Bluetooth speaker (optional, for audio warnings)

2. **Software Requirements:**
   ```bash
   pip install opencv-python numpy pyserial
   pip install degirum  # Degirum SDK for object detection
   ```
Note: The voyager-sdk needs to be installed on the system and Metis device working properly. If the degirum sdk cannot find the Metis device, type "axdevice --refresh" from inside the voyager-sdk's venv.

### Configuration

Edit `settings.json` to configure your system:

```json
{
  "camera": {
    "device": "/dev/video20",
    "resolution": [1920, 1280],
    "fov_degrees": [8.28, 5.53]
  },
  "scan_area": {
    "azimuth_range": [-20.0, 20.0],
    "elevation_range": [-20.0, 0.0],
    "min_overlap_percentage": 0.1,
    "scan_cycles": 1
  },
  "restricted_areas": [
    {
      "name": "Bridge Entrance",
      "polygon": [
        [-45.0, -10.0],
        [-30.0, -5.0],
        [-15.0, -10.0],
        [-30.0, -15.0]
      ],
      "warning_threshold_seconds": 30,
      "classes": ["boat_small", "boat_medium"]
    },
    {
      "name": "Marine Sanctuary",
      "polygon": [
        [-45.0, -10.0],
        [-30.0, -5.0],
        [-15.0, -10.0],
        [-30.0, -15.0]
      ],
      "warning_threshold_seconds": 30,
      "classes": ["boat_large", "ship"]
    }
  ]
}
```

### Running the System

#### Continuous Monitoring Mode (Default)
```bash
python3 main.py
```

#### Single Scan Mode
```bash
python3 main.py --mode single
```

#### Hardware Testing Only
```bash
python3 main.py --test
```

#### Custom Configuration
```bash
python3 main.py --config my_settings.json --mode continuous
```

#### Live Video Feed (Camera Focusing)
```bash
python3 main.py --live-video
```
Use this mode to focus and adjust your camera before starting surveillance. The system will:
- Display a live video feed with a crosshair for focusing
- Show camera resolution and FPS information
- Allow you to adjust camera settings while viewing the feed
- Press Ctrl+C in the terminal to exit and return to normal operation

## ⚙️ Configuration Details

### Camera Configuration
- **device**: Video device path (e.g., `/dev/video20`)
- **resolution**: Camera resolution [width, height]
- **fov_degrees**: Camera field of view [azimuth, elevation] in degrees

### Scan Area Configuration
- **azimuth_range**: Azimuth scan range [min, max] in degrees
- **elevation_range**: Elevation scan range [min, max] in degrees
- **min_overlap_percentage**: Frame overlap percentage (0.0-1.0) for automatic step calculation from camera FOV
- **scan_cycles**: Number of complete scan cycles to perform (1/2/3/... or -1 for indefinite)

### Restricted Areas
- **name**: Human-readable area name
- **polygon**: List of [azimuth, elevation] points defining the area boundary
  - Supports any number of points (minimum 3 for a valid polygon)
  - Points should be specified in clockwise or counter-clockwise order
  - Supports complex shapes: triangles, rectangles, irregular polygons
  - Automatically closed by the system (connects last point to first)
- **warning_threshold_seconds**: Time threshold for triggering audio warnings
- **classes**: List of object classes to monitor (optional, monitors all classes if empty)
  - Example: `["boat_small", "boat_medium", "ship"]`
  - Only specified classes will trigger warnings in this area

### Gimbal Configuration
- **move_timeout_seconds**: Maximum time to wait for gimbal movement
- **settle_time_seconds**: Time to wait after movement for stabilization
- **max_retries**: Maximum retry attempts for failed movements

### Detection Configuration
- **model_path**: Path to Degirum model file
- **confidence_threshold**: Minimum confidence for detections (0.0-1.0)
- **tile_size**: Image tile size for processing large images
- **overlap**: Overlap between tiles (0.0-1.0)

## 🔧 Hardware Setup

### UART Connection
Connect the RK3588 UART to STM32:
- RK3588 TX → STM32 RX
- RK3588 RX → STM32 TX
- Ground connection
- Baud rate: 115200

### Camera Setup
1. Connect USB camera to RK3588
2. Verify device path: `ls /dev/video*`
3. Update `settings.json` with correct device path

### Audio Setup (Optional)
For Bluetooth speaker audio warnings:
```bash
# Install Bluetooth audio support
sudo apt-get install bluealsa

# Pair and connect Bluetooth speaker
bluetoothctl
# pair XX:XX:XX:XX:XX:XX
# connect XX:XX:XX:XX:XX:XX
```

## 📊 System Operation

### Scan Process
1. **Position Planning**: System calculates optimal scan positions
2. **Gimbal Movement**: Moves to each scan position with verification
3. **Image Capture**: Captures high-resolution image
4. **Object Detection**: Runs AI detection using Degirum SDK
5. **Data Logging**: Saves detections with position and timestamp
6. **Area Monitoring**: Checks for restricted area violations
7. **Audio Warnings**: Issues alerts for violations

### Detection Results
- **Real-time Display**: Shows detections on captured images
- **CSV Logging**: Saves all detections to timestamped files
- **Image Storage**: Saves annotated images for review
- **Statistics**: Tracks detection rates and patterns

## 🔍 Scan Planning and Execution

### Automatic Step Calculation
The system automatically calculates optimal scan steps based on:
- **Camera Field of View**: Uses actual camera FOV angles
- **Overlap Percentage**: Configurable frame overlap (default 10%)
- **Formula**: `step = FOV × (1 - overlap_percentage)`

Example: With 60° FOV and 10% overlap → step = 60° × 0.9 = 54°

### Vertical Serpentine Scan Pattern
- **Systematic Coverage**: Starts from top-left of scan area
- **Vertical Serpentine Movement**: Alternates direction on each column
  - Column 1: Top to bottom (down)
  - Column 2: Bottom to top (up)
  - Column 3: Top to bottom (down)
  - And so on...
- **Complete Coverage**: Ensures no gaps in surveillance area

### Scan Cycle Control
- **Finite Cycles**: Specify exact number of complete scans (1, 2, 3, etc.)
- **Indefinite Monitoring**: Set `scan_cycles: -1` for continuous operation
- **Progress Tracking**: Shows current cycle and total progress

### Restricted Area Handling
- **Full Coverage**: All positions in scan area are visited (no exclusions)
- **Monitoring vs Exclusion**: Restricted areas are monitored, not avoided
- **Class-Specific Alerts**: Only specified object classes trigger warnings per area
- **Flexible Polygons**: Support for complex shapes with 3+ vertices

## 🎯 Key Features

### Intelligent Scan Planning
- **FOV-based step calculation** using camera field of view and overlap percentage
- **Vertical serpentine scan pattern** for systematic coverage (top-left to bottom-right)
- **Automatic overlap management** to ensure no gaps in surveillance
- **Configurable scan cycles** (1-N cycles or indefinite monitoring)
- **Complete area coverage** including restricted zones (monitoring vs exclusion)

### Robust Gimbal Control
- **Position verification** after each movement with angle validation
- **Retry logic** for failed movements with configurable timeouts
- **Emergency stop** functionality with motor centering
- **UART communication** with CRC error checking and protocol validation

### Advanced Object Detection
- **Degirum SDK integration** for high-performance AI inference
- **Image tiling** for processing large images (1920x1280)
- **Coordinate system conversion** from normalized to azimuth/elevation angles
- **Confidence thresholding** to filter noise and false positives
- **Multi-class detection** for various maritime objects

### Restricted Area Monitoring
- **Polygon-based zones** supporting complex shapes (triangles to irregular polygons)
- **Class-specific monitoring** - monitor only specified object classes per area
- **Dwell time tracking** for persistent violations with configurable thresholds
- **Audio warnings** via Bluetooth speaker for immediate alerts
- **Visual alarm indicators** with red bounding boxes and status display
- **Comprehensive violation logging** with timestamps and object details

## 📈 Monitoring and Logging

### Real-time Statistics
- Detection counts and confidence scores
- Gimbal movement success rates
- System performance metrics
- Hardware status monitoring

### Data Export
- **CSV files** with detection data
- **Annotated images** for visual verification
- **System logs** for troubleshooting
- **Performance statistics** for optimization

## 🛠️ Troubleshooting

### Common Issues

#### 1. UART Communication Problems
```bash
# Check UART device
ls /dev/ttyS* /dev/ttyUSB*

# Test serial connection
python3 -c "import serial; s=serial.Serial('/dev/ttyS0', 115200, timeout=1); print('UART OK' if s.is_open else 'UART FAIL')"
```

#### 2. Camera Issues
```bash
# Check camera device
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext /dev/video20


# Test camera capture
ffmpeg -f v4l2 -i /dev/video20 -vframes 1 test.jpg

# Run comprehensive camera test
python3 utilities/camera_test.py
```

#### 3. Degirum SDK Issues
```bash
# Check Degirum installation (from inside degirum venv)
python3 -c "import degirum as dg; print('Degirum OK')"

degirum sys-info

# Verify model file
ls -la models/yolo_v5s_maritime
```

#### 4. Gimbal Movement Issues
- Check UART wiring and baud rate
- Verify STM32 is running and UARTProtocol is active
- Check gimbal angle limits in settings
- Monitor STM32 serial output for error messages

### Camera Testing and Debugging
The system includes comprehensive camera testing utilities:

```bash
# Run comprehensive camera test
python3 utilities/camera_test.py

# This tests:
# - Device existence and permissions
# - Multiple OpenCV backends (V4L2, ANY)
# - Different resolutions (640x480, 1280x720, 1920x1080, 1920x1280)
# - Frame capture capabilities
# - Camera properties and settings
# - Continuous capture stability
```

### Debug Mode
Enable detailed logging by modifying `settings.json`:
```json
{
  "logging": {
    "log_level": "DEBUG"
  }
}
```

### Camera Backend Fixes
If you encounter camera issues, the system automatically tries multiple OpenCV backends:
1. **V4L2 Backend** (recommended for Linux)
2. **ANY Backend** (auto-detection fallback)

The camera module includes:
- ✅ Multiple backend support
- ✅ Retry logic for failed captures
- ✅ Stabilization delays
- ✅ **Camera warm-up sequence** (discards first 10 frames)
- ✅ Comprehensive error handling
- ✅ Format and resolution testing

### Camera Warm-up Process
After opening the camera and setting resolution, the system automatically:
1. Waits 0.5 seconds for initial stabilization
2. Discards the first 10 frames to allow:
   - Auto-exposure adjustment
   - Auto-white balance stabilization
   - Auto-focus completion (if available)
   - Overall camera parameter optimization
3. Ensures consistent image quality for detection

## 🔧 API Reference

### GimbalController
```python
gimbal = GimbalController(config)
gimbal.connect()                    # Establish UART connection
gimbal.get_angles()                 # Get current angles (pitch, yaw)
gimbal.set_angles(pitch, yaw)       # Set target angles
gimbal.move_to_position(az, el)     # Move to azimuth/elevation
```

### ObjectDetector
```python
detector = ObjectDetector(config)
detector.initialize()               # Initialize Degirum model
detections = detector.detect_with_tiling(image)  # Run detection
```

### AreaMonitor
```python
monitor = AreaMonitor(config)
warnings = monitor.process_detections(detections, scan_pos)
```

## 📝 Development Notes

### Adding New Features
1. Create new module in appropriate file
2. Update `settings.json` for configuration
3. Integrate with main orchestrator in `main.py`
4. Add proper error handling and logging

### Performance Optimization
- **Automatic FOV-based step calculation** eliminates manual tuning
- **Overlap percentage adjustment** for coverage vs speed trade-off
- **Scan cycle configuration** for mission-specific operation duration
- **Class-specific monitoring** reduces false positive processing
- **Flexible polygon definitions** for precise restricted area boundaries
- Monitor CPU/GPU usage with Axelera tools

### Safety Considerations
- Implement proper emergency stop procedures
- Add watchdog timers for critical operations
- Validate all input parameters
- Handle hardware failures gracefully

## 📄 License

This project is part of the maritime surveillance system for RK3588 with Axelera Metis M.2.

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Add comprehensive tests
4. Submit a pull request

## 📞 Support

For issues and questions:
1. Check the troubleshooting section
2. Review system logs
3. Verify hardware connections
4. Check configuration settings

---

**System Status**: ✅ Production Ready with Enhanced Features
**Hardware**: RK3588 + Axelera Metis M.2 + STM32 Gimbal Controller
**Software**: Python 3.8+ with Degirum SDK + OpenCV
**Key Features**: Coordinate conversion, audio alerts, visual alarms, robust error handling, real-time monitoring