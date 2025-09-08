#!/usr/bin/env python
"""
Maritime Surveillance System - Main Application
RK3588 with Axelera Metis M.2 + STM32 Gimbal Control
"""
import sys
import time
import signal
import threading
import os
from datetime import datetime
from typing import Optional

# Import our modules
from config import ConfigManager
from uart_comm import GimbalController
from scan_planner import ScanPlanner, ScanPosition
from camera import CameraCapture
from detector import ObjectDetector, DetectionResult
from logger import DetectionLogger
from monitor import AreaMonitor


class MaritimeSurveillanceSystem:
    """Main application orchestrator"""

    def __init__(self, config_file: str = "settings.json"):
        print("🌊 Initializing Maritime Surveillance System...")

        # Load configuration
        try:
            self.config = ConfigManager(config_file)
        except Exception as e:
            print(f"❌ Failed to load configuration: {e}")
            print("🚦 Using default configuration values")
            # Create a config manager with defaults
            self.config = ConfigManager.__new__(ConfigManager)
            self.config._config = {}

        # Validate configuration
        errors = self.config.validate_config()
        if errors:
            print("⚠️ Configuration warnings:")
            for error in errors:
                print(f"  - {error}")
            print("🚦 Continuing with default values where possible")

        # Initialize components
        self.gimbal = GimbalController(self.config)
        self.scan_planner = ScanPlanner(self.config)
        self.camera = CameraCapture(self.config)
        self.detector = ObjectDetector(self.config)
        self.logger = DetectionLogger(self.config)
        self.monitor = AreaMonitor(self.config)

        # System state
        self.running = False
        self.shutting_down = False  # Flag to prevent duplicate shutdown
        self.scan_positions: list = []
        self.current_scan_index = 0

        print("✅ System initialized successfully")

    def initialize_hardware(self) -> bool:
        """Initialize all hardware components"""
        print("🔧 Initializing hardware...")

        # Check if shutdown was requested
        if not self.running:
            print("🛑 Shutdown requested during hardware initialization")
            return False

        # Initialize gimbal communication
        if not self.gimbal.connect():
            print("❌ Failed to connect to gimbal")
            return False

        # Check if shutdown was requested
        if not self.running:
            print("🛑 Shutdown requested during hardware initialization")
            return False

        # Initialize camera
        if not self.camera.open_camera():
            print("❌ Failed to open camera")
            return False

        # Check if shutdown was requested
        if not self.running:
            print("🛑 Shutdown requested during hardware initialization")
            return False

        # Test camera
        if not self.camera.test_camera():
            print("❌ Camera test failed")
            return False

        # Check if shutdown was requested
        if not self.running:
            print("🛑 Shutdown requested during hardware initialization")
            return False

        # Initialize object detector
        if not self.detector.initialize():
            print("❌ Failed to initialize object detector")
            return False

        print("✅ Hardware initialization complete")
        return True

    def prepare_scan_plan(self) -> bool:
        """Prepare the scan plan"""
        print("📋 Preparing scan plan...")

        # Check if shutdown was requested
        if not self.running:
            print("🛑 Shutdown requested during scan planning")
            return False

        # Generate scan positions
        self.scan_positions = self.scan_planner.generate_scan_positions()

        # Note: Restricted areas are not filtered out - they are used for monitoring only
        # All positions in the scan area are included for complete coverage

        if not self.scan_positions:
            print("❌ No valid scan positions found")
            return False

        # Check if shutdown was requested
        if not self.running:
            print("🛑 Shutdown requested during scan planning")
            return False

        # Get scan statistics
        stats = self.scan_planner.get_scan_statistics(self.scan_positions)
        print("📊 Scan Plan Statistics:")
        print(f"  - Total positions: {stats['total_positions']}")
        az_range = stats['azimuth_range']
        el_range = stats['elevation_range']
        print(f"  - Azimuth range: {az_range[0]:.1f}° to {az_range[1]:.1f}°")
        print(f"  - Elevation range: {el_range[0]:.1f}° to {el_range[1]:.1f}°")
        print(f"  - Total movement: {stats['total_movement']:.1f}°")
        print(f"  - Avg movement per step: {stats['avg_movement_per_step']:.1f}°")

        print("✅ Scan plan prepared")
        return True

    def run_scan_cycle(self) -> None:
        """Run one complete scan cycle"""
        if not self.scan_positions:
            print("❌ No scan positions available")
            return

        print(f"🔄 Starting scan cycle with {len(self.scan_positions)} positions...")

        # Initialize alarm_detections for the entire cycle
        alarm_detections = []

        # Turn stabilization on before starting scan
        if not self.gimbal.set_stabilization_on():
            print("⚠️ Failed to turn stabilization on")
        time.sleep(1)  # 1s delay as requested

        for i, scan_pos in enumerate(self.scan_positions):
            if not self.running:
                break

            print(f"📍 Scanning position {i+1}/{len(self.scan_positions)}: {scan_pos}")

            # Check if shutdown was requested before starting gimbal movement
            if not self.running:
                print("🛑 Shutdown requested, aborting current scan position")
                break

            # Move gimbal to scan position
            print(f"🎯 Starting gimbal movement to {scan_pos}...")
            if not self.gimbal.move_to_position(scan_pos.azimuth, scan_pos.elevation):
                print(f"⚠️ Failed to move to position {scan_pos}, skipping...")
                continue
            print(f"✅ Gimbal movement completed for {scan_pos}")

            # Check if shutdown was requested after gimbal movement
            if not self.running:
                print("🛑 Shutdown requested, aborting current scan position")
                break

            # Capture fresh image (flush buffer to get most recent frame)
            print(f"📷 Starting fresh camera capture...")
            image = self.camera.capture_image_with_retry()
            if image is None:
                print("⚠️ Failed to capture fresh image, skipping detection...")
                continue
            print(f"✅ Fresh camera capture completed, image shape: {image.shape if hasattr(image, 'shape') else 'unknown'}")

            # Dataset collection will be handled after detection

            # Check if shutdown was requested after camera capture
            if not self.running:
                print("🛑 Shutdown requested, aborting current scan position")
                break

            # Run object detection
            print(f"🎯 Starting object detection on image shape: {image.shape}...")

            # Check detection configuration
            use_tiling = self.config.detection.use_tiling
            print(f"🎯 Detection mode: {'Tiled' if use_tiling else 'Simple'}")

            if use_tiling:
                print(f"🎯 Running tiled detection...")
                detections = self.detector.detect_with_tiling(image)
            else:
                print(f"🎯 Running simple detection...")
                detections = self.detector.detect_objects(image)

            print(f"🎯 Object detection completed, found {len(detections) if detections else 0} detections")

            # Check if shutdown was requested after detection
            if not self.running:
                print("🛑 Shutdown requested, aborting current scan position")
                break

            # Ensure detections is a list
            if detections is None:
                detections = []
                print(f"⚠️ Detections was None, setting to empty list")

            # Process detections
            if detections:
                print(f"🎯 Found {len(detections)} objects")

                # Check for restricted area violations first to get alarm_detections
                warnings, alarm_detections = self.monitor.process_detections(detections, scan_pos.to_tuple())
                for warning in warnings:
                    print(warning)

                # Log detections
                image_filename = self.logger.save_image(image, detections, scan_pos.to_tuple(), alarm_detections)
                self.logger.log_detections(detections, scan_pos.to_tuple(), image_filename)
            else:
                print("✨ No objects detected")
                # No detections - reset alarm state for this frame
                warnings = []
                alarm_detections = []

            # Dataset collection (handle both cases)
            if self.config.dataset.enabled:
                dataset_config = self.config.dataset
                should_save = dataset_config.save_all_images or (detections and not dataset_config.save_all_images)  # Save when detections found OR when save_all_images is true

                if should_save:
                    try:
                        dataset_folder = dataset_config.folder_path
                        os.makedirs(dataset_folder, exist_ok=True)

                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                        filename = f"{timestamp}_{scan_pos.azimuth:.1f}_{scan_pos.elevation:.1f}.jpg"
                        dataset_filepath = os.path.join(dataset_folder, filename)

                        import cv2
                        success = cv2.imwrite(dataset_filepath, image)
                        if success:
                            detection_status = "with detections" if detections else "no detections"
                            print(f"💾 Dataset image saved ({detection_status}): {dataset_filepath}")
                        else:
                            print(f"⚠️ Failed to save dataset image: {dataset_filepath}")

                    except Exception as e:
                        print(f"⚠️ Dataset saving error: {e}")

            # Check if shutdown was requested before display
            if not self.running:
                print("🛑 Shutdown requested, aborting current scan position")
                break

            # Display results in window (always show, regardless of detections)
            print(f"🎨 Displaying detection results for position {i+1}")
            try:
                import cv2

                # Test GUI availability
                if i == 0:
                    try:
                        test_window = "Test Window"
                        cv2.namedWindow(test_window, cv2.WINDOW_NORMAL)
                        cv2.destroyWindow(test_window)
                        print("✅ OpenCV GUI available")
                    except Exception as gui_test_error:
                        print(f"⚠️ OpenCV GUI test failed: {gui_test_error}")
                        print("📺 Continuing without visual display")
                        return

                # Create window on first scan position
                window_title = "Maritime Surveillance - Live Detections"
                if i == 0:
                    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(window_title, 960, 640)  # Set reasonable initial size
                    print(f"📺 Opened detection display window: {window_title}")
                    print(f"📊 Window size set to 960x640 for optimal viewing")

                annotated_image = self.detector.draw_detections(image, detections, alarm_detections)

                # Add scan position info to display
                height, width = annotated_image.shape[:2]
                cv2.putText(annotated_image, f"Position {i+1}/{len(self.scan_positions)}: {scan_pos}",
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(annotated_image, f"Detections: {len(detections)}",
                          (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                # Add alarm status
                alarm_active = len(warnings) > 0
                alarm_text = "Alarm: ACTIVE" if alarm_active else "Alarm: None"
                color = (0, 0, 255) if alarm_active else (0, 255, 0)  # BGR: Red for active, Green for none
                cv2.putText(annotated_image, alarm_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                # Show in persistent window
                cv2.imshow(window_title, annotated_image)
                print(f"📺 Updated detection window with position {i+1}/{len(self.scan_positions)}, detections: {len(detections)}")

                # Check if shutdown was requested before display
                if not self.running:
                    print("🛑 Shutdown requested, skipping display")
                    break

                # Check for window close or key press
                key = cv2.waitKey(100)  # Brief pause to show image
                if key == 27:  # ESC key
                    print("🛑 ESC pressed - stopping surveillance")
                    self.running = False
                    break

                # Check if shutdown was requested during display
                if not self.running:
                    print("🛑 Shutdown requested during display")
                    break

                # Check window status safely
                try:
                    if cv2.getWindowProperty(window_title, cv2.WND_PROP_VISIBLE) < 1:
                        print("🛑 Detection window closed - stopping surveillance")
                        self.running = False
                        break
                except cv2.error:
                    print("🛑 Detection window closed - stopping surveillance")
                    self.running = False
                    break

            except ImportError:
                pass  # OpenCV not available
            except Exception as display_error:
                print(f"⚠️ Display error: {display_error}")

        # Turn stabilization off after scan
        if not self.gimbal.set_stabilization_off():
            print("⚠️ Failed to turn stabilization off")

        print("✅ Scan cycle complete")

    def run_continuous_monitoring(self) -> None:
        """Run continuous monitoring mode"""
        scan_cycles = self.config.scan_area.scan_cycles
        current_cycle = 0

        if scan_cycles == -1:
            print("🔄 Starting continuous monitoring (indefinite cycles)...")
        else:
            print(f"🔄 Starting continuous monitoring ({scan_cycles} cycles)...")

        while self.running:
            try:
                current_cycle += 1
                if scan_cycles != -1 and current_cycle > scan_cycles:
                    print(f"✅ Completed {scan_cycles} scan cycles as requested")
                    break

                print(f"🔄 Starting scan cycle {current_cycle}" + (f"/{scan_cycles}" if scan_cycles != -1 else ""))
                self.run_scan_cycle()

                # Brief pause between cycles
                time.sleep(2.0)

            except KeyboardInterrupt:
                break
            except Exception as e:
                try:
                    error_msg = str(e) if e is not None else "Unknown error"
                    print(f"❌ Error in monitoring cycle {current_cycle}: {error_msg}")
                except Exception:
                    print(f"❌ Error in monitoring cycle {current_cycle}: (formatting failed)")
                time.sleep(5.0)  # Wait before retrying

        print("🛑 Continuous monitoring stopped")

    def run_single_scan(self) -> None:
        """Run a single scan cycle"""
        print("🔄 Running single scan...")
        self.run_scan_cycle()
        print("✅ Single scan complete")

    def run_inference_test(self) -> None:
        """Run continuous inference testing"""
        print("🧪 Starting Inference Testing Mode")
        print("🎯 Press Ctrl+C to exit inference testing")

        # Check detection configuration
        use_tiling = self.config.detection.use_tiling
        print(f"🎯 Detection mode: {'Tiled' if use_tiling else 'Simple'}")

        inference_count = 0
        alarm_detections = []  # Initialize alarm_detections for the entire test

        try:
            while self.running:
                inference_count += 1
                print(f"\n📸 Inference Test #{inference_count}")

                # Capture fresh image
                image = self.camera.capture_image_with_retry()
                if image is None:
                    print("⚠️ Failed to capture image, skipping...")
                    time.sleep(1)
                    continue

                print(f"✅ Image captured, shape: {image.shape}")

                # Run detection
                if use_tiling:
                    detections = self.detector.detect_with_tiling(image)
                else:
                    detections = self.detector.detect_objects(image)

                print(f"🎯 Detection completed, found {len(detections) if detections else 0} objects")

                # Ensure detections is a list
                if detections is None:
                    detections = []

                # Process detections
                if detections:
                    print(f"🎯 Found {len(detections)} objects")

                    # Check for restricted area violations first to get alarm_detections
                    dummy_position = (0.0, 0.0)  # azimuth, elevation
                    warnings, alarm_detections = self.monitor.process_detections(detections, dummy_position)
                    for warning in warnings:
                        print(warning)

                    # Log detections
                    image_filename = self.logger.save_image(image, detections, dummy_position, alarm_detections)
                    self.logger.log_detections(detections, dummy_position, image_filename)
                else:
                    print("✨ No objects detected")
                    # No detections - reset alarm state for this frame
                    warnings = []
                    alarm_detections = []

                # Display results
                try:
                    import cv2

                    # Create window on first inference
                    window_title = "Inference Test - Live Results"
                    if inference_count == 1:
                        cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
                        cv2.resizeWindow(window_title, 960, 540)

                    annotated_image = self.detector.draw_detections(image, detections, alarm_detections)

                    # Add test info to display
                    height, width = annotated_image.shape[:2]
                    cv2.putText(annotated_image, f"Inference Test #{inference_count}",
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(annotated_image, f"Detections: {len(detections)}",
                              (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    cv2.putText(annotated_image, f"Mode: {'Tiled' if use_tiling else 'Simple'}",
                              (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

                    # Add alarm status
                    alarm_active = len(warnings) > 0
                    alarm_text = "Alarm: ACTIVE" if alarm_active else "Alarm: None"
                    color = (0, 0, 255) if alarm_active else (0, 255, 0)  # BGR: Red for active, Green for none
                    cv2.putText(annotated_image, alarm_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

                    cv2.imshow(window_title, annotated_image)

                    # Check for window close or key press
                    key = cv2.waitKey(100)
                    if key == 27:  # ESC key
                        print("🛑 ESC pressed - stopping inference test")
                        self.running = False
                        break

                    # Check window status
                    try:
                        if cv2.getWindowProperty(window_title, cv2.WND_PROP_VISIBLE) < 1:
                            print("🛑 Inference test window closed")
                            self.running = False
                            break
                    except cv2.error:
                        print("🛑 Inference test window closed")
                        self.running = False
                        break

                except ImportError:
                    pass  # OpenCV not available

                # Brief pause between inferences
                time.sleep(0.5)

        except KeyboardInterrupt:
            print("\n🛑 Inference test interrupted by user")
        except Exception as e:
            print(f"❌ Inference test error: {e}")

        print(f"✅ Inference testing complete - ran {inference_count} inferences")

    def get_system_status(self) -> dict:
        """Get comprehensive system status"""
        return {
            'gimbal_connected': self.gimbal.connected,
            'camera_open': self.camera.is_open,
            'detector_initialized': self.detector.initialized,
            'scan_positions': len(self.scan_positions),
            'monitoring_stats': self.monitor.get_monitoring_stats(),
            'detection_stats': self.logger.get_statistics(hours=1)
        }

    def start(self, mode: str = "continuous") -> None:
        """Start the surveillance system"""
        print("🚀 Starting Maritime Surveillance System...")

        # Set running to True before initialization so shutdown checks work properly
        self.running = True
        self.shutting_down = False

        # Initialize hardware
        if not self.initialize_hardware():
            print("❌ Hardware initialization failed")
            return

        # Check if shutdown was requested after hardware init
        if not self.running:
            print("🛑 Shutdown requested after hardware initialization")
            self.stop()
            return

        # Prepare scan plan
        if not self.prepare_scan_plan():
            print("❌ Scan plan preparation failed")
            return

        # Check if shutdown was requested after scan planning
        if not self.running:
            print("🛑 Shutdown requested after scan planning")
            self.stop()
            return

        print("🚦 Signal handler active - press Ctrl+C to quit")

        try:
            if mode == "continuous":
                self.run_continuous_monitoring()
            elif mode == "single":
                self.run_single_scan()
            else:
                print(f"❌ Unknown mode: {mode}")
                return

        except KeyboardInterrupt:
            print("\n🛑 Received interrupt signal")
        except Exception as e:
            try:
                error_msg = str(e) if e is not None else "Unknown error"
                print(f"❌ Unexpected error: {error_msg}")
            except Exception:
                print("❌ Unexpected error: (formatting failed)")
        finally:
            if not self.shutting_down:  # Only call stop() if not already shutting down
                self.stop()

    def stop(self, timeout_seconds: int = 30) -> None:
        """Stop the surveillance system with timeout protection"""
        if self.shutting_down:
            return  # Already shutting down

        print("🛑 Stopping Maritime Surveillance System...")
        self.shutting_down = True
        self.running = False

        shutdown_start = time.time()

        def check_timeout():
            return (time.time() - shutdown_start) > timeout_seconds

        # Turn off stabilization with timeout protection
        print("🔧 Turning off stabilization...")
        try:
            if not check_timeout():
                self.gimbal.set_stabilization_off()
            else:
                print("⏰ Timeout: Skipping stabilization shutdown")
        except Exception as e:
            print(f"⚠️ Error turning off stabilization: {e}")

        # Close hardware connections with error handling
        print("🔌 Disconnecting hardware...")
        try:
            if not check_timeout():
                self.gimbal.disconnect()
            else:
                print("⏰ Timeout: Skipping gimbal disconnect")
        except Exception as e:
            print(f"⚠️ Error disconnecting gimbal: {e}")

        try:
            if not check_timeout():
                self.camera.close_camera()
            else:
                print("⏰ Timeout: Skipping camera close")
        except Exception as e:
            print(f"⚠️ Error closing camera: {e}")

        # Clean up OpenCV windows
        print("🖼️ Cleaning up display windows...")
        try:
            if not check_timeout():
                import cv2
                cv2.destroyAllWindows()
            else:
                print("⏰ Timeout: Skipping window cleanup")
        except ImportError:
            pass
        except Exception as e:
            print(f"⚠️ Error cleaning up windows: {e}")

        # Get final statistics (with error handling and timeout)
        print("📊 Gathering final statistics...")
        try:
            if not check_timeout():
                final_stats = self.get_system_status()
                print("📊 Final System Status:")
                print(f"  - Gimbal connected: {final_stats['gimbal_connected']}")
                print(f"  - Camera open: {final_stats['camera_open']}")
                print(f"  - Detector initialized: {final_stats['detector_initialized']}")
                print(f"  - Scan positions: {final_stats['scan_positions']}")

                monitoring_stats = final_stats['monitoring_stats']
                print(f"  - Tracked objects: {monitoring_stats['total_tracked_objects']}")
                print(f"  - Objects in restricted areas: {monitoring_stats['objects_in_restricted_areas']}")
                print(f"  - Warnings issued: {monitoring_stats['warnings_issued']}")

                detection_stats = final_stats['detection_stats']
                print(f"  - Total detections (1h): {detection_stats['total_detections']}")
            else:
                print("⏰ Timeout: Skipping final statistics")
        except Exception as e:
            print(f"⚠️ Error getting final statistics: {e}")

        shutdown_duration = time.time() - shutdown_start
        print(f"✅ System shutdown complete (took {shutdown_duration:.1f}s)")

    def emergency_stop(self) -> None:
        """Emergency stop - immediately stop all operations"""
        print("🚨 EMERGENCY STOP ACTIVATED")

        # Set shutdown flags immediately
        self.running = False
        self.shutting_down = True

        # Quick cleanup - don't call full stop() to avoid recursion
        print("🔧 Emergency cleanup...")

        # Turn off stabilization (quick, no retries)
        try:
            self.gimbal.set_stabilization_off()
        except Exception as e:
            print(f"⚠️ Emergency stabilization shutdown failed: {e}")

        # Try to center the gimbal quickly
        try:
            self.gimbal.move_to_position(0.0, 0.0)
        except Exception as e:
            print(f"⚠️ Emergency gimbal centering failed: {e}")

        # Clean up OpenCV windows
        try:
            import cv2
            cv2.destroyAllWindows()
        except (ImportError, Exception) as e:
            print(f"⚠️ Emergency window cleanup failed: {e}")

        print("✅ Emergency stop complete")


# Global variables for signal handling
system_instance = None
shutdown_requested = False
shutdown_lock = threading.Lock()

def signal_handler(signum, frame):
    """Handle system signals with thread-safe shutdown"""
    global shutdown_requested, system_instance

    with shutdown_lock:
        # Prevent multiple shutdown attempts
        if shutdown_requested:
            return

        shutdown_requested = True

        try:
            if isinstance(signum, int):
                print(f"\n📡 Received signal {signum}")
            else:
                print(f"\n📡 Received signal: {signum}")

            if system_instance is not None:
                # Set the instance's running flag to False immediately
                system_instance.running = False

                # Only call emergency_stop if the system has been initialized
                if hasattr(system_instance, 'shutting_down'):
                    system_instance.emergency_stop()
                else:
                    # System not fully initialized, just exit
                    print("⚠️ System not fully initialized, exiting immediately")
                    import os
                    os._exit(0)
            else:
                print("⚠️ System instance not available, exiting immediately")
                import os
                os._exit(0)
        except Exception as e:
            print(f"\n📡 Signal handler error: {e}")
            # Force exit if signal handler fails
            import os
            os._exit(1)



def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="Maritime Surveillance System")
    parser.add_argument("--config", default="settings.json",
                       help="Configuration file path")
    parser.add_argument("--mode", choices=["continuous", "single"],
                       default="continuous", help="Operation mode")
    parser.add_argument("--test", action="store_true",
                       help="Run hardware tests only")
    parser.add_argument("--camera-test", action="store_true",
                       help="Run camera capture test only")
    parser.add_argument("--detection-test", action="store_true",
                       help="Run detection test only")
    parser.add_argument("--live-video", action="store_true",
                         help="Show live camera feed for testing/focusing (press 'i' for angles, Ctrl+C to exit)")
    parser.add_argument("--test-inference", action="store_true",
                        help="Run continuous inference testing (capture, detect, display, log)")

    args = parser.parse_args()

    # Signal handlers will be set up in start() method after running = True

    # Create system instance and make it globally accessible
    global system_instance, shutdown_requested
    system_instance = MaritimeSurveillanceSystem(args.config)
    shutdown_requested = False  # Reset shutdown flag for new run

    # Set up signal handlers EARLY - before any long-running operations
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    if args.test:
        # Run hardware tests only (no signal handlers)
        print("🧪 Running hardware tests...")
        try:
            success = system_instance.initialize_hardware()
            if success:
                print("✅ All hardware tests passed")
                system_instance.camera.close_camera()
                system_instance.gimbal.disconnect()
            else:
                print("❌ Hardware tests failed")
        except KeyboardInterrupt:
            print("\n🛑 Test interrupted by user")
            try:
                system_instance.camera.close_camera()
                system_instance.gimbal.disconnect()
            except:
                pass
        return

    if args.camera_test:
        # Run camera capture test only
        print("📷 Running camera capture test...")

        # Set running flag for hardware initialization
        system_instance.running = True

        try:
            # Test camera opening first
            print("📷 Testing camera opening...")
            camera_success = system_instance.camera.open_camera()
            if not camera_success:
                print("❌ Camera opening failed")
                return

            print("✅ Camera opened successfully")

            # Test gimbal connection
            print("🔗 Testing gimbal connection...")
            gimbal_success = system_instance.gimbal.connect()
            if gimbal_success:
                print("✅ Gimbal connected successfully")
                system_instance.gimbal.disconnect()
            else:
                print("⚠️ Gimbal connection failed (continuing with camera test)")

            print("📷 Testing fresh camera capture...")
            image = system_instance.camera.capture_fresh_image()
            if image is not None:
                print(f"✅ Fresh camera capture successful, image shape: {image.shape}")
            else:
                print("❌ Fresh camera capture failed")
                # Try with retry mechanism
                print("📷 Trying with retry mechanism...")
                image = system_instance.camera.capture_image_with_retry(max_retries=1)
                if image is not None:
                    print(f"✅ Camera capture with retry successful, image shape: {image.shape}")
                else:
                    print("❌ Camera capture with retry also failed")

        except KeyboardInterrupt:
            print("\n🛑 Camera test interrupted by user")
        finally:
            try:
                system_instance.camera.close_camera()
            except:
                pass
        return

    if args.live_video:
        # Run live video feed for camera testing/focusing
        print("📹 Starting live video feed for camera testing...")
        print("🎯 Press 'i' to query current gimbal angles")
        print("🎯 Press Ctrl+C to exit live video mode")

        # Set running flag for hardware initialization
        system_instance.running = True

        # Initialize hardware
        success = system_instance.initialize_hardware()
        if not success:
            print("❌ Hardware initialization failed")
            return

        try:
            import cv2
            print("📹 Live video started - focus camera and press Ctrl+C when done")

            # State for displaying angles on video
            angle_display_start = 0
            current_angles = None
            angle_display_duration = 5.0  # Show angles for 5 seconds

            # PIP toggle state
            pip_display_enabled = True  # Start with PIP enabled by default

            while True:
                # Capture fresh image for live video (quiet mode to reduce console spam)
                image = system_instance.camera.capture_image(quiet=True)
                if image is not None:
                    # Add timestamp and instructions
                    display_image = image.copy()
                    height, width = display_image.shape[:2]

                    # Add overlay text
                    cv2.putText(display_image, f"Camera Live Feed - {width}x{height}",
                              (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(display_image, f"FPS: {int(1000/30)} | Focus camera as needed",
                              (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    # Add background for better visibility
                    pip_status = "ON" if pip_display_enabled else "OFF"
                    text = f"Press 'i' for gimbal angles | 'p' to toggle PIP ({pip_status}) | Close window or [ESC] to stop"
                    (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
                    cv2.rectangle(display_image,
                                (5, height - text_height - 25),
                                (15 + text_width, height - 5),
                                (0, 0, 0), -1)

                    cv2.putText(display_image, text,
                              (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


                    # Add Picture-in-Picture (PIP) zoomed view
                    try:
                        live_video_config = system_instance.config.live_video
                        if live_video_config and getattr(live_video_config, 'pip_enabled', False) and pip_display_enabled:
                            pip_zoom = getattr(live_video_config, 'pip_zoom_factor', 2.0)
                            pip_center_pct = getattr(live_video_config, 'pip_center_percentage', 0.3)

                            # Calculate center region to zoom
                            center_region_size = int(min(width, height) * pip_center_pct)
                            x1 = center_x - center_region_size // 2
                            y1 = center_y - center_region_size // 2
                            x2 = x1 + center_region_size
                            y2 = y1 + center_region_size

                            # Ensure bounds
                            x1, y1 = max(0, x1), max(0, y1)
                            x2, y2 = min(width, x2), min(height, y2)

                            # Extract center region
                            center_region = display_image[y1:y2, x1:x2].copy()

                            # Resize for zoom effect
                            zoomed_width = int(center_region.shape[1] * pip_zoom)
                            zoomed_height = int(center_region.shape[0] * pip_zoom)
                            zoomed_region = cv2.resize(center_region, (zoomed_width, zoomed_height), interpolation=cv2.INTER_LINEAR)

                            # Position PIP at top-middle
                            pip_x = (width - zoomed_width) // 2
                            pip_y = 20  # Top margin

                            # Ensure PIP fits within image bounds
                            pip_x = max(0, min(pip_x, width - zoomed_width))
                            pip_y = max(0, min(pip_y, height - zoomed_height))

                            # Add border around PIP
                            cv2.rectangle(display_image, (pip_x-2, pip_y-2), (pip_x + zoomed_width + 2, pip_y + zoomed_height + 2), (255, 255, 255), 2)

                            # Overlay zoomed region
                            display_image[pip_y:pip_y + zoomed_height, pip_x:pip_x + zoomed_width] = zoomed_region

                            # Add crosshair to PIP (centered)
                            pip_center_x = pip_x + zoomed_width // 2
                            pip_center_y = pip_y + zoomed_height // 2
                            crosshair_size = 20
                            cv2.line(display_image, (pip_center_x - crosshair_size, pip_center_y), (pip_center_x + crosshair_size, pip_center_y), (0, 255, 0), 1)
                            cv2.line(display_image, (pip_center_x, pip_center_y - crosshair_size), (pip_center_x, pip_center_y + crosshair_size), (0, 255, 0), 1)

                    except Exception as pip_error:
                        # Silently handle PIP errors to avoid disrupting main video feed
                        pass

                    # Add crosshair for pointing screen's center
                    # We add this AFTER the PIP so that the crossshair doesn't get magnified also
                    center_x, center_y = width // 2, height // 2
                    cv2.line(display_image, (center_x - 40, center_y), (center_x + 40, center_y), (0, 255, 0), 2)
                    cv2.line(display_image, (center_x, center_y - 40), (center_x, center_y + 40), (0, 255, 0), 2)


                    # Display angles on video if recently queried
                    current_time = time.time()
                    if current_angles and (current_time - angle_display_start) < angle_display_duration:
                        pitch, yaw = current_angles
                        # Display in top-right corner with background
                        angle_text = f"Yaw: {yaw:.3f} deg"
                        (text_width, text_height), _ = cv2.getTextSize(angle_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)

                        # Background rectangle
                        cv2.rectangle(display_image,
                                    (width - text_width - 20, 10),
                                    (width - 10, 10 + text_height + 20),
                                    (0, 0, 0), -1)

                        # Angle text
                        cv2.putText(display_image, angle_text,
                                  (width - text_width - 15, 10 + text_height + 5),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                        angle_text2 = f"Pitch: {pitch:.3f} deg"
                        (text_width2, _), _ = cv2.getTextSize(angle_text2, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)

                        cv2.rectangle(display_image,
                                    (width - text_width2 - 20, 10 + text_height + 25),
                                    (width - 10, 10 + 2*text_height + 35),
                                    (0, 0, 0), -1)

                        cv2.putText(display_image, angle_text2,
                                  (width - text_width2 - 15, 10 + 2*text_height + 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    elif current_angles and (current_time - angle_display_start) >= angle_display_duration:
                        # Clear angles after timeout
                        current_angles = None

                    # Show image
                    cv2.imshow("Camera Live Feed", display_image)

                    # Check for window close
                    key = cv2.waitKey(30)  # 30ms delay for smooth display
                    if key == 27:  # ESC key
                        print("🛑 ESC pressed - exiting live video")
                        break
                    elif key == ord('i') or key == ord('I'):  # 'i' key for info
                        # Query current gimbal angles
                        print("📍 Querying current gimbal angles...")
                        angles = system_instance.gimbal.get_angles()
                        if angles:
                            pitch, yaw = angles
                            current_angles = angles
                            angle_display_start = time.time()
                            print("🎯 Current Gimbal Position:")
                            print(f"   Yaw: {yaw:.3f} deg")
                            print(f"   Pitch: {pitch:.3f} deg")
                            print("💡 Use these coordinates for scan ranges or restricted areas")
                            print("📺 Angles also displayed on video feed")
                        else:
                            print("❌ Failed to get gimbal angles")
                    elif key == ord('p') or key == ord('P'):  # 'p' key to toggle PIP
                        pip_display_enabled = not pip_display_enabled
                        status = "ENABLED" if pip_display_enabled else "DISABLED"
                        print(f"📺 PIP display {status}")

                    # Check window status safely
                    try:
                        if cv2.getWindowProperty("Camera Live Feed", cv2.WND_PROP_VISIBLE) < 1:
                            print("🛑 Live video window closed")
                            break
                    except cv2.error:
                        # Window might have been destroyed
                        print("🛑 Live video window closed")
                        break
                else:
                    print("⚠️ Failed to capture image")
                    time.sleep(1)

        except KeyboardInterrupt:
            print("\n📹 Live video stopped by user")
        except Exception as e:
            print(f"❌ Live video error: {e}")
        finally:
            cv2.destroyAllWindows()
            system_instance.camera.close_camera()
            system_instance.gimbal.disconnect()
        return

    if args.detection_test:
        # Run detection test only
        print("🎯 Running detection test...")

        # Set running flag for hardware initialization
        system_instance.running = True

        # Initialize hardware
        success = system_instance.initialize_hardware()
        if not success:
            print("❌ Hardware initialization failed")
            return

        # Capture fresh test image
        print("📷 Capturing fresh test image...")
        image = system_instance.camera.capture_fresh_image()
        if image is None:
            print("❌ Camera capture failed")
            system_instance.camera.close_camera()
            system_instance.gimbal.disconnect()
            return

        print(f"✅ Captured test image, shape: {image.shape}")

        # Test detection
        print("🎯 Testing object detection...")
        detections = system_instance.detector.detect_objects(image)
        print(f"🎯 Detection test completed, found {len(detections)} detections")

        # Cleanup
        system_instance.camera.close_camera()
        system_instance.gimbal.disconnect()
        return

    if args.test_inference:
        # Run inference testing mode
        print("🧪 Running inference testing mode...")

        # Set running flag for hardware initialization
        system_instance.running = True

        # Initialize hardware
        success = system_instance.initialize_hardware()
        if not success:
            print("❌ Hardware initialization failed")
            return

        try:
            system_instance.run_inference_test()
        except KeyboardInterrupt:
            print("\n🛑 Inference testing interrupted by user")
        finally:
            # Cleanup
            system_instance.camera.close_camera()
            system_instance.gimbal.disconnect()
        return

    # Start the system
    system_instance.start(args.mode)


if __name__ == "__main__":
    main()