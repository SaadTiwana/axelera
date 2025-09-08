"""
Camera Capture Module for USB Camera
"""
import cv2
import numpy as np
import time
from typing import Optional, Tuple
from config import ConfigManager


class CameraCapture:
    """Handles USB camera capture and image acquisition"""

    def __init__(self, config: ConfigManager):
        self.config = config
        self.camera_config = config.camera
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_open = False

    def open_camera(self) -> bool:
        """Open camera device using simplified working approach with retry mechanism"""
        max_retries = 3

        # Check if device exists
        import os
        if not os.path.exists(self.camera_config.device):
            print(f"❌ Camera device {self.camera_config.device} does not exist")
            # List available video devices
            try:
                video_devices = [f for f in os.listdir('/dev') if f.startswith('video')]
                if video_devices:
                    print(f"📷 Available video devices: {video_devices}")
                else:
                    print("📷 No video devices found in /dev/")
            except Exception as e:
                print(f"⚠️ Could not list video devices: {e}")
            return False

        for attempt in range(max_retries):
            try:
                print(f"📷 Opening camera (attempt {attempt + 1}/{max_retries})...")

                # Use the working approach
                self.cap = cv2.VideoCapture(self.camera_config.device, cv2.CAP_V4L2)

                if not self.cap.isOpened():
                    print(f"❌ Failed to open camera: {self.camera_config.device}")
                    # Try alternative backends
                    print("🔄 Trying alternative camera backends...")
                    for backend in [cv2.CAP_ANY, cv2.CAP_V4L]:
                        try:
                            self.cap = cv2.VideoCapture(self.camera_config.device, backend)
                            if self.cap.isOpened():
                                print(f"✅ Camera opened with backend {backend}")
                                break
                        except Exception as backend_error:
                            print(f"⚠️ Backend {backend} failed: {backend_error}")
                            continue
                    else:
                        if attempt < max_retries - 1:
                            print("⏳ Retrying in 1 second...")
                            import time
                            time.sleep(1)
                            continue
                        return False

                # Set camera properties to minimize latency and buffering
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_config.resolution[0])
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_config.resolution[1])
                self.cap.set(cv2.CAP_PROP_FPS, 30)  # Reduced from 50 to reduce buffering

                # Minimize buffering for fresh frames
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimal buffer size
                self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus for faster capture

                # Camera warm-up: discard first few frames to allow stabilization
                print("🔥 Warming up camera (discarding bad frames)...")
                import time
                time.sleep(0.5)  # Initial stabilization delay

                # Discard first 5 frames to allow camera to adjust
                for i in range(5):
                    try:
                        ret, _ = self.cap.read()
                        if ret:
                            print(f"   Discarded frame {i+1}")
                        else:
                            print(f"   Frame {i+1} failed (expected during warm-up)")
                    except Exception as e:
                        print(f"⚠️  Frame {i+1} error during warm-up: {str(e)[:50]}...")
                        # Continue with warm-up despite errors
                    time.sleep(0.1)

                print("✅ Camera warm-up complete")

                # Verify settings after warm-up
                actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

                print(f"✅ Camera initialized: {self.camera_config.device}")
                if actual_width is not None and actual_height is not None:
                    print(f"   Resolution: {int(actual_width)}x{int(actual_height)}")
                else:
                    print(f"   Resolution: unknown")
                if actual_fps is not None:
                    print(f"   FPS: {int(actual_fps)}")
                else:
                    print(f"   FPS: unknown")
                print(f"   Format: MJPG")

                self.is_open = True
                return True

            except Exception as e:
                print(f"❌ Camera initialization error (attempt {attempt + 1}): {e}")
                if self.cap:
                    self.cap.release()
                if attempt < max_retries - 1:
                    print("⏳ Retrying camera initialization...")
                    import time
                    time.sleep(1)
                else:
                    print("❌ All camera initialization attempts failed")
                    return False

        return False

    def close_camera(self) -> None:
        """Close camera device"""
        if self.cap:
            self.cap.release()
        self.is_open = False
        print("📷 Camera closed")

    def capture_image(self, quiet: bool = False) -> Optional[np.ndarray]:
        """Capture single image from camera"""
        if not quiet:
            print(f"📷 capture_image called, camera open: {self.is_open}, cap exists: {self.cap is not None}")
        if not self.is_open or not self.cap:
            if not quiet:
                print("❌ Camera not open")
            return None

        try:
            if not quiet:
                print(f"📷 Calling cap.read()...")
            # Add timeout to prevent hanging
            import threading
            result = [None, None]
            def capture_with_timeout():
                result[0], result[1] = self.cap.read()

            capture_thread = threading.Thread(target=capture_with_timeout)
            capture_thread.start()
            capture_thread.join(timeout=5.0)  # 5 second timeout

            if capture_thread.is_alive():
                if not quiet:
                    print("❌ Camera capture timed out")
                return None

            ret, frame = result
            if not quiet:
                print(f"📷 cap.read() returned, ret: {ret}, frame type: {type(frame)}")
            if not ret:
                if not quiet:
                    print("❌ Image not retrieved")
                return None

            # Validate the captured frame
            if frame is None:
                if not quiet:
                    print("❌ Captured frame is None")
                return None

            if not hasattr(frame, 'shape') or len(frame.shape) < 2:
                if not quiet:
                    print(f"❌ Invalid frame format: {type(frame)}")
                return None

            if not quiet:
                print(f"✅ Frame captured successfully, shape: {frame.shape}")
            return frame

        except Exception as e:
            if not quiet:
                print(f"❌ Image capture error: {e}")
            return None

    def flush_camera_buffer(self, frames_to_discard: int = 5) -> None:
        """Flush camera buffer by discarding old frames to get fresh image"""
        if not self.is_open or not self.cap:
            print("❌ Camera not open - cannot flush buffer")
            return

        print(f"🧹 Flushing camera buffer (discarding {frames_to_discard} frames)...")
        discarded_count = 0

        for i in range(frames_to_discard):
            try:
                ret, _ = self.cap.read()
                if ret:
                    discarded_count += 1
                else:
                    print(f"⚠️ Failed to read frame {i+1} during buffer flush")
            except Exception as e:
                print(f"⚠️ Error during buffer flush frame {i+1}: {e}")

        print(f"✅ Buffer flush complete - discarded {discarded_count}/{frames_to_discard} frames")
        # Small delay to allow camera to capture new frame
        time.sleep(0.1)

    def capture_fresh_image(self) -> Optional[np.ndarray]:
        """Capture a fresh image by flushing buffer first"""
        print(f"📷 Capturing fresh image...")
        # Flush buffer to get most recent frame
        self.flush_camera_buffer(frames_to_discard=3)

        # Now capture the fresh frame
        return self.capture_image()

    def capture_image_with_retry(self, max_retries: int = 3) -> Optional[np.ndarray]:
        """Capture image with retry mechanism"""
        print(f"📷 Starting capture with {max_retries} max retries...")
        for attempt in range(max_retries):
            print(f"📷 Attempt {attempt + 1}/{max_retries}...")
            # Use fresh capture for first attempt, fallback to regular for retries
            if attempt == 0:
                image = self.capture_fresh_image()
            else:
                image = self.capture_image()

            if image is not None:
                print(f"✅ Capture successful on attempt {attempt + 1}")
                return image

            if attempt < max_retries - 1:
                print(f"⚠️ Capture attempt {attempt + 1} failed, retrying...")
                time.sleep(0.5)

        print("❌ All capture attempts failed")
        return None

    def capture_image_simple(self) -> Optional[np.ndarray]:
        """Simple camera capture without timeout for testing"""
        print(f"📷 Simple capture: camera open: {self.is_open}, cap exists: {self.cap is not None}")
        if not self.is_open or not self.cap:
            print("❌ Camera not open")
            return None

        try:
            print(f"📷 Calling simple cap.read()...")
            ret, frame = self.cap.read()
            print(f"📷 Simple cap.read() returned, ret: {ret}")
            if not ret:
                print("❌ Image not retrieved")
                return None

            if frame is None:
                print("❌ Captured frame is None")
                return None

            print(f"✅ Simple capture successful, shape: {frame.shape if hasattr(frame, 'shape') else 'unknown'}")
            return frame

        except Exception as e:
            print(f"❌ Simple capture error: {e}")
            return None


    def get_camera_info(self) -> dict:
        """Get camera information and capabilities"""
        if not self.is_open or not self.cap:
            return {}

        info = {
            'device': self.camera_config.device,
            'resolution': self.camera_config.resolution,
            'fov_degrees': self.camera_config.fov_degrees,
            'backend': self.cap.getBackendName(),
            'fps': self.cap.get(cv2.CAP_PROP_FPS),
            'brightness': self.cap.get(cv2.CAP_PROP_BRIGHTNESS),
            'contrast': self.cap.get(cv2.CAP_PROP_CONTRAST),
            'saturation': self.cap.get(cv2.CAP_PROP_SATURATION),
            'hue': self.cap.get(cv2.CAP_PROP_HUE),
            'gain': self.cap.get(cv2.CAP_PROP_GAIN),
            'exposure': self.cap.get(cv2.CAP_PROP_EXPOSURE)
        }

        return info

    def test_camera(self) -> bool:
        """Test camera functionality"""
        print("🧪 Testing camera...")

        # Check if camera is already open (from initialization)
        if not self.is_open:
            if not self.open_camera():
                return False

        # Capture test image using existing camera instance
        test_image = self.capture_image()
        if test_image is None:
            print("❌ Camera test failed - cannot capture image")
            return False

        # Check image properties
        try:
            height, width = test_image.shape[:2]
            print(f"✅ Camera test passed")
            print(f"   Image size: {width}x{height}")
            print(f"   Data type: {getattr(test_image, 'dtype', 'unknown')}")
            print(f"   Channels: {test_image.shape[2] if len(test_image.shape) > 2 else 1}")
        except (AttributeError, ValueError, TypeError) as e:
            print(f"✅ Camera test passed (limited info available: {e})")
            print(f"   Image type: {type(test_image)}")

        # Save test image for verification
        test_filename = "test_capture.jpg"
        if self.save_image(test_image, test_filename):
            print(f"💾 Test image saved: {test_filename}")

        return True


    def save_image(self, image: np.ndarray, filename: str) -> bool:
        """Save image to file"""
        try:
            if image is None:
                print(f"❌ Failed to save image: image is None")
                return False

            if not hasattr(image, 'shape'):
                print(f"❌ Failed to save image: invalid image format")
                return False

            cv2.imwrite(filename, image)
            print(f"💾 Image saved: {filename}")
            return True
        except Exception as e:
            print(f"❌ Failed to save image: {e}")
            return False

    def display_image(self, image: np.ndarray, window_name: str = "Camera Feed",
                     wait_time: int = 1) -> None:
        """Display image in OpenCV window"""
        try:
            cv2.imshow(window_name, image)
            cv2.waitKey(wait_time)
        except Exception as e:
            print(f"❌ Display error: {e}")

    def pixel_to_angle(self, pixel_x: int, pixel_y: int, image_width: int, image_height: int) -> Tuple[float, float]:
        """Convert pixel coordinates to azimuth/elevation angles"""
        # Camera FOV in degrees
        fov_azimuth, fov_elevation = self.camera_config.fov_degrees

        # Convert pixel coordinates to normalized device coordinates (-1 to 1)
        ndc_x = (2.0 * pixel_x / image_width) - 1.0
        ndc_y = (2.0 * pixel_y / image_height) - 1.0

        # Convert to angles (assuming camera is looking forward)
        azimuth_angle = ndc_x * (fov_azimuth / 2.0)
        elevation_angle = -ndc_y * (fov_elevation / 2.0)  # Negative because Y increases downward

        return azimuth_angle, elevation_angle

    def angle_to_pixel(self, azimuth: float, elevation: float, image_width: int, image_height: int) -> Tuple[int, int]:
        """Convert azimuth/elevation angles to pixel coordinates"""
        # Camera FOV in degrees
        fov_azimuth, fov_elevation = self.camera_config.fov_degrees

        # Convert angles to normalized device coordinates
        ndc_x = azimuth / (fov_azimuth / 2.0)
        ndc_y = -elevation / (fov_elevation / 2.0)  # Negative because Y increases downward

        # Convert to pixel coordinates
        pixel_x = int((ndc_x + 1.0) * image_width / 2.0)
        pixel_y = int((ndc_y + 1.0) * image_height / 2.0)

        # Clamp to image bounds
        pixel_x = max(0, min(image_width - 1, pixel_x))
        pixel_y = max(0, min(image_height - 1, pixel_y))

        return pixel_x, pixel_y