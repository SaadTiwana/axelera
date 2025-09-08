"""
Configuration management for Maritime Surveillance System
"""
import json
import os
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass


@dataclass
class UARTConfig:
    port: str
    baudrate: int
    timeout: float


@dataclass
class CameraConfig:
    device: str
    resolution: Tuple[int, int]
    fov_degrees: Tuple[float, float]


@dataclass
class ScanAreaConfig:
    azimuth_range: Tuple[float, float]
    elevation_range: Tuple[float, float]
    min_overlap_percentage: float
    scan_cycles: int


@dataclass
class RestrictedArea:
    name: str
    polygon: List[Tuple[float, float]]
    warning_threshold_seconds: int
    classes: List[str]


@dataclass
class GimbalConfig:
    move_timeout_seconds: float
    settle_time_seconds: float
    max_retries: int


@dataclass
class DetectionConfig:
    model_path: str
    model_name: str
    confidence_threshold: float
    tile_size: int
    overlap: float
    use_tiling: bool


@dataclass
class LoggingConfig:
    detections_file: str
    images_directory: str
    log_level: str


@dataclass
class AudioConfig:
    warning_sound: str
    device: str


@dataclass
class LiveVideoConfig:
    pip_enabled: bool
    pip_zoom_factor: float
    pip_center_percentage: float
    pip_position: str


@dataclass
class DatasetConfig:
    enabled: bool
    save_all_images: bool
    folder_path: str


class ConfigManager:
    """Manages application configuration from JSON file"""

    def __init__(self, config_file: str = "settings.json"):
        self.config_file = config_file
        self._config = {}
        try:
            self.load_config()
        except Exception as e:
            print(f"⚠️ Config loading failed in __init__: {e}, using defaults")
            # Keep empty config, properties will return defaults

    def load_config(self) -> None:
        """Load configuration from JSON file"""
        try:
            with open(self.config_file, 'r') as f:
                config_data = json.load(f)
                # Ensure we have a valid dict
                if isinstance(config_data, dict):
                    self._config = config_data
                else:
                    print(f"⚠️ Config file does not contain a valid dictionary, using defaults")
                    self._config = {}
        except FileNotFoundError:
            print(f"⚠️ Configuration file {self.config_file} not found, using defaults")
            self._config = {}
        except json.JSONDecodeError as e:
            print(f"⚠️ Invalid JSON in configuration file: {e}, using defaults")
            self._config = {}
        except Exception as e:
            print(f"⚠️ Error loading configuration: {e}, using defaults")
            self._config = {}

    def save_config(self) -> None:
        """Save current configuration to file"""
        with open(self.config_file, 'w') as f:
            json.dump(self._config, f, indent=2)

    @property
    def uart(self) -> UARTConfig:
        try:
            uart = self._config.get('uart', {})
            if not uart:
                raise KeyError("UART config missing")
            return UARTConfig(
                port=uart.get('port', '/dev/ttyUSB0'),
                baudrate=uart.get('baudrate', 115200),
                timeout=uart.get('timeout', 1.0)
            )
        except (KeyError, TypeError, ValueError) as e:
            # Return default values if config is invalid
            return UARTConfig(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=1.0
            )

    @property
    def camera(self) -> CameraConfig:
        try:
            cam = self._config.get('camera', {})
            if not cam:
                raise KeyError("Camera config missing")
            return CameraConfig(
                device=cam.get('device', '/dev/video0'),
                resolution=tuple(cam.get('resolution', [1920, 1280])),
                fov_degrees=tuple(cam.get('fov_degrees', [60.0, 40.0]))
            )
        except (KeyError, TypeError, ValueError) as e:
            # Return default values if config is invalid
            return CameraConfig(
                device='/dev/video0',
                resolution=(1920, 1280),
                fov_degrees=(60.0, 40.0)
            )

    @property
    def scan_area(self) -> ScanAreaConfig:
        try:
            sa = self._config.get('scan_area', {})
            if not sa:
                raise KeyError("Scan area config missing")
            return ScanAreaConfig(
                azimuth_range=tuple(sa.get('azimuth_range', [-20.0, 20.0])),
                elevation_range=tuple(sa.get('elevation_range', [-20.0, 0.0])),
                min_overlap_percentage=sa.get('min_overlap_percentage', 0.1),
                scan_cycles=sa.get('scan_cycles', 1)
            )
        except (KeyError, TypeError, ValueError) as e:
            # Return default values if config is invalid
            return ScanAreaConfig(
                azimuth_range=(-20.0, 20.0),
                elevation_range=(-20.0, 0.0),
                min_overlap_percentage=0.1,
                scan_cycles=1
            )

    @property
    def restricted_areas(self) -> List[RestrictedArea]:
        areas = []
        try:
            restricted_areas_config = self._config.get('restricted_areas', [])
            if not restricted_areas_config:
                return areas

            for area in restricted_areas_config:
                if isinstance(area, dict):
                    try:
                        name = area.get('name', 'Unknown Area')
                        polygon = [tuple(point) for point in area.get('polygon', [])] if area.get('polygon') else []
                        threshold = area.get('warning_threshold_seconds', 30)
                        classes = area.get('classes', [])
                        areas.append(RestrictedArea(
                            name=name,
                            polygon=polygon,
                            warning_threshold_seconds=threshold,
                            classes=classes
                        ))
                    except (KeyError, TypeError, ValueError):
                        continue
        except Exception:
            pass
        return areas

    @property
    def gimbal(self) -> GimbalConfig:
        try:
            g = self._config.get('gimbal', {})
            if not g:
                raise KeyError("Gimbal config missing")
            return GimbalConfig(
                move_timeout_seconds=g.get('move_timeout_seconds', 3.0),
                settle_time_seconds=g.get('settle_time_seconds', 1.0),
                max_retries=g.get('max_retries', 3)
            )
        except (KeyError, TypeError, ValueError) as e:
            # Return default values if config is invalid
            return GimbalConfig(
                move_timeout_seconds=3.0,
                settle_time_seconds=1.0,
                max_retries=3
            )

    @property
    def detection(self) -> DetectionConfig:
        try:
            d = self._config.get('detection', {})
            if not d:
                raise KeyError("Detection config missing")
            return DetectionConfig(
                model_path=d.get('model_path', '/axelera/degirum/my-zoo/axelera/'),
                model_name=d.get('model_name', 'yolo11m_marine_detector_v4--640x640_quant_axelera_metis_2'),
                confidence_threshold=d.get('confidence_threshold', 0.5),
                tile_size=d.get('tile_size', 640),
                overlap=d.get('overlap', 0.1),
                use_tiling=d.get('use_tiling', False)
            )
        except (KeyError, TypeError, ValueError) as e:
            # Return default values if config is invalid
            return DetectionConfig(
                model_path='/axelera/degirum/my-zoo/axelera/',
                model_name='yolo11m_marine_detector_v4--640x640_quant_axelera_metis_2',
                confidence_threshold=0.5,
                tile_size=640,
                overlap=0.1,
                use_tiling=False
            )

    @property
    def logging(self) -> LoggingConfig:
        try:
            l = self._config.get('logging', {})
            if not l:
                raise KeyError("Logging config missing")
            return LoggingConfig(
                detections_file=l.get('detections_file', 'logs/detections.csv'),
                images_directory=l.get('images_directory', 'logs/images/'),
                log_level=l.get('log_level', 'INFO')
            )
        except (KeyError, TypeError, ValueError) as e:
            # Return default values if config is invalid
            return LoggingConfig(
                detections_file='logs/detections.csv',
                images_directory='logs/images/',
                log_level='INFO'
            )

    @property
    def audio(self) -> AudioConfig:
        try:
            a = self._config.get('audio', {})
            if not a:
                raise KeyError("Audio config missing")
            return AudioConfig(
                warning_sound=a.get('warning_sound', 'sounds/alarm.wav'),
                device=a.get('device', 'bluealsa')
            )
        except (KeyError, TypeError, ValueError) as e:
            # Return default values if config is invalid
            return AudioConfig(
                warning_sound='sounds/alarm.wav',
                device='bluealsa'
            )

    @property
    def live_video(self) -> LiveVideoConfig:
        try:
            lv = self._config.get('live_video', {})
            if not lv:
                raise KeyError("Live video config missing")
            return LiveVideoConfig(
                pip_enabled=lv.get('pip_enabled', True),
                pip_zoom_factor=lv.get('pip_zoom_factor', 2.0),
                pip_center_percentage=lv.get('pip_center_percentage', 0.3),
                pip_position=lv.get('pip_position', 'top_middle')
            )
        except (KeyError, TypeError, ValueError) as e:
            # Return default values if config is invalid
            return LiveVideoConfig(
                pip_enabled=True,
                pip_zoom_factor=2.0,
                pip_center_percentage=0.3,
                pip_position='top_middle'
            )

    @property
    def dataset(self) -> DatasetConfig:
        try:
            ds = self._config.get('dataset', {})
            if not ds:
                raise KeyError("Dataset config missing")
            return DatasetConfig(
                enabled=ds.get('enabled', False),
                save_all_images=ds.get('save_all_images', False),
                folder_path=ds.get('folder_path', 'logs/dataset/')
            )
        except (KeyError, TypeError, ValueError) as e:
            # Return default values if config is invalid
            return DatasetConfig(
                enabled=False,
                save_all_images=False,
                folder_path='logs/dataset/'
            )

    def get_raw_config(self) -> Dict[str, Any]:
        """Get raw configuration dictionary"""
        return self._config.copy()

    def update_config(self, key: str, value: Any) -> None:
        """Update a configuration value"""
        keys = key.split('.')
        config = self._config

        # Navigate to the nested key
        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]

        # Update the value
        config[keys[-1]] = value

    def validate_config(self) -> List[str]:
        """Validate configuration and return list of errors"""
        errors = []

        try:
            # Validate camera settings
            camera_config = self.camera
            if camera_config and camera_config.device:
                if not os.path.exists(camera_config.device):
                    errors.append(f"Camera device {camera_config.device} does not exist")
        except Exception as e:
            errors.append(f"Camera config validation failed: {e}")

        try:
            # Validate scan area ranges
            scan_config = self.scan_area
            if scan_config:
                if scan_config.azimuth_range and len(scan_config.azimuth_range) >= 2:
                    if scan_config.azimuth_range[0] >= scan_config.azimuth_range[1]:
                        errors.append("Invalid azimuth range: min >= max")
                if scan_config.elevation_range and len(scan_config.elevation_range) >= 2:
                    if scan_config.elevation_range[0] >= scan_config.elevation_range[1]:
                        errors.append("Invalid elevation range: min >= max")

                # Validate overlap percentage
                if scan_config.min_overlap_percentage is not None and (scan_config.min_overlap_percentage < 0 or scan_config.min_overlap_percentage >= 1):
                    errors.append("Min overlap percentage must be between 0.0 and 1.0")
        except Exception as e:
            errors.append(f"Scan area config validation failed: {e}")

        try:
            # Validate restricted areas
            for area in self.restricted_areas:
                if area and area.polygon and len(area.polygon) < 3:
                    errors.append(f"Restricted area '{area.name or 'unknown'}' must have at least 3 points")
        except Exception as e:
            errors.append(f"Restricted areas config validation failed: {e}")

        return errors