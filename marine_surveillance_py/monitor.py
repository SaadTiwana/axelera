"""
Restricted Area Monitoring and Audio Warning System
"""
import time
import threading
import subprocess
from typing import List, Dict, Any, Optional
from datetime import datetime, timedelta
from config import ConfigManager, RestrictedArea
from detector import DetectionResult


class TrackedObject:
    """Represents a tracked object with position history"""

    def __init__(self, detection: DetectionResult, scan_position: tuple,
                 timestamp: datetime):
        self.label = detection.label
        self.first_seen = timestamp
        self.last_seen = timestamp
        self.scan_positions = [scan_position]
        self.detection_history = [detection]
        self.consecutive_detections = 1
        self.in_restricted_area = False
        self.warning_issued = False

    def update(self, detection: DetectionResult, scan_position: tuple,
              timestamp: datetime) -> None:
        """Update object with new detection"""
        self.last_seen = timestamp
        self.scan_positions.append(scan_position)
        self.detection_history.append(detection)
        self.consecutive_detections += 1

    def get_time_in_area(self) -> float:
        """Get time spent in restricted area in seconds"""
        if not self.in_restricted_area:
            return 0.0
        try:
            return (self.last_seen - self.first_seen).total_seconds()
        except (AttributeError, TypeError):
            return 0.0

    def should_issue_warning(self, threshold_seconds: int) -> bool:
        """Check if warning should be issued"""
        if self.warning_issued:
            return False

        time_in_area = self.get_time_in_area()
        return time_in_area >= threshold_seconds


class AreaMonitor:
    """Monitors restricted areas and manages audio warnings"""

    def __init__(self, config: ConfigManager):
        self.config = config
        self.restricted_areas = config.restricted_areas
        self.audio_config = config.audio

        # Tracking state
        self.tracked_objects: Dict[str, TrackedObject] = {}
        self.next_object_id = 0

        # Audio system
        self.audio_enabled = True
        self.last_warning_time = datetime.min

    def process_detections(self, detections: List[DetectionResult],
                            scan_position: tuple) -> tuple:
        """Process detections and check for restricted area violations

        Returns:
            tuple: (warnings_issued, alarm_detections)
                - warnings_issued: List of warning messages
                - alarm_detections: List of DetectionResult objects that triggered alarms
        """
        current_time = datetime.now()
        warnings_issued = []
        alarm_detections = []

        for detection in detections:
            # Convert detection coordinates to azimuth/elevation angles
            detection_angles = self._convert_detection_to_angles(detection, scan_position)

            # Check if detection is in any restricted area and matches specified classes
            for area in self.restricted_areas:
                if self._point_in_polygon(detection_angles, area.polygon):
                    # Check if detection class is in the area's monitored classes
                    if not area.classes or detection.label in area.classes:
                        if area.warning_threshold_seconds == 0:
                            # Issue immediate warning
                            warning_msg = self._issue_warning_immediate(detection, area.name)
                            warnings_issued.append(warning_msg)
                            alarm_detections.append(detection)
                        else:
                            # Use tracking logic for threshold > 0
                            object_id = self._get_or_create_object_id(detection, scan_position, current_time)
                            tracked_obj = self.tracked_objects[object_id]
                            tracked_obj.in_restricted_area = True

                            # Check if warning should be issued
                            if tracked_obj.should_issue_warning(area.warning_threshold_seconds):
                                warning_msg = self._issue_warning(tracked_obj, area.name)
                                warnings_issued.append(warning_msg)
                                tracked_obj.warning_issued = True
                                alarm_detections.append(detection)
                        break  # Detection can only be in one area

        # Clean up old tracked objects
        self._cleanup_old_objects(current_time)

        return warnings_issued, alarm_detections

    def _get_or_create_object_id(self, detection: DetectionResult,
                                scan_position: tuple, timestamp: datetime) -> str:
        """Get existing object ID or create new one"""
        # Simple tracking: match by label and proximity
        # In a real system, you'd use more sophisticated tracking algorithms

        for obj_id, tracked_obj in self.tracked_objects.items():
            # Check if this detection matches an existing tracked object
            if (tracked_obj.label == detection.label and
                self._detections_match(tracked_obj.detection_history[-1], detection)):

                tracked_obj.update(detection, scan_position, timestamp)
                return obj_id

        # Create new tracked object
        object_id = f"{detection.label}_{self.next_object_id}"
        self.next_object_id += 1

        self.tracked_objects[object_id] = TrackedObject(
            detection, scan_position, timestamp
        )

        return object_id

    def _detections_match(self, detection1: DetectionResult,
                         detection2: DetectionResult) -> bool:
        """Check if two detections likely represent the same object"""
        # Simple matching based on bounding box overlap
        iou = self._calculate_iou(detection1.bbox, detection2.bbox)
        return iou > 0.5  # 50% overlap threshold

    def _calculate_iou(self, bbox1, bbox2) -> float:
        """Calculate Intersection over Union"""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2

        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)

        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0

        intersection_area = (x2_i - x1_i) * (y2_i - y1_i)

        # Calculate union
        bbox1_area = (x2_1 - x1_1) * (y2_1 - y1_1)
        bbox2_area = (x2_2 - x1_2) * (y2_2 - y1_2)
        union_area = bbox1_area + bbox2_area - intersection_area

        return intersection_area / union_area if union_area > 0 else 0.0

    def _point_in_polygon(self, point: tuple, polygon: List[tuple]) -> bool:
        """Check if point is inside polygon using ray casting algorithm"""
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def _convert_detection_to_angles(self, detection: DetectionResult, scan_position: tuple) -> tuple:
        """Convert detection normalized coordinates to azimuth/elevation angles"""
        try:
            scan_azimuth, scan_elevation = scan_position
            center_x, center_y = detection.get_center()

            # Get camera FOV from config
            fov_azimuth, fov_elevation = self.config.camera.fov_degrees

            # Convert normalized coordinates to angles using the same formula as logger.py
            center_azimuth = (scan_azimuth - (fov_azimuth / 2.0)) + (fov_azimuth * center_x)
            center_elevation = (scan_elevation - (fov_elevation / 2.0)) + (fov_elevation * center_y)

            return (center_azimuth, center_elevation)
        except (AttributeError, TypeError, ValueError) as e:
            print(f"⚠️ Error converting detection coordinates: {e}")
            return (0.0, 0.0)


    def _issue_warning(self, tracked_obj: TrackedObject, area_name: str) -> str:
        """Issue audio warning for restricted area violation"""
        try:
            time_in_area = tracked_obj.get_time_in_area()
            if isinstance(time_in_area, (int, float)) and time_in_area is not None:
                warning_msg = f"🚨 ALERT: {tracked_obj.label} detected in {area_name} for {time_in_area:.1f} seconds"
            else:
                warning_msg = f"🚨 ALERT: {tracked_obj.label} detected in {area_name}"
        except (ValueError, TypeError):
            warning_msg = f"🚨 ALERT: {tracked_obj.label} detected in {area_name}"

        print(warning_msg)

        # Play audio warning
        if self.audio_enabled:
            self._play_audio_warning()

        self.last_warning_time = datetime.now()
        return warning_msg

    def _issue_warning_immediate(self, detection: DetectionResult, area_name: str) -> str:
        """Issue immediate audio warning for restricted area violation (threshold=0)"""
        warning_msg = f"🚨 IMMEDIATE ALERT: {detection.label} detected in {area_name}"

        print(warning_msg)

        # Play audio warning
        if self.audio_enabled:
            self._play_audio_warning()

        self.last_warning_time = datetime.now()
        return warning_msg

    def _play_audio_warning(self) -> None:
        """Play audio warning sound"""
        try:
            if self.audio_config.device == "bluealsa":
                # For Bluetooth speaker
                cmd = ["aplay", self.audio_config.warning_sound]
            else:
                # Default audio device
                cmd = ["aplay", "-D", self.audio_config.device, self.audio_config.warning_sound]

            subprocess.run(cmd, capture_output=True, timeout=5)

        except subprocess.TimeoutExpired:
            print("⚠️ Audio playback timed out")
        except FileNotFoundError:
            print("⚠️ Audio file not found")
        except Exception as e:
            print(f"⚠️ Audio playback failed: {e}")

    def _cleanup_old_objects(self, current_time: datetime) -> None:
        """Remove old tracked objects"""
        # Remove objects not seen for more than 5 minutes
        cutoff_time = current_time - timedelta(minutes=5)

        objects_to_remove = []
        for obj_id, tracked_obj in self.tracked_objects.items():
            if tracked_obj.last_seen < cutoff_time:
                objects_to_remove.append(obj_id)

        for obj_id in objects_to_remove:
            del self.tracked_objects[obj_id]

        if objects_to_remove:
            print(f"🧹 Cleaned up {len(objects_to_remove)} old tracked objects")

    def get_monitoring_stats(self) -> Dict[str, Any]:
        """Get monitoring statistics"""
        total_objects = len(self.tracked_objects)
        restricted_objects = sum(1 for obj in self.tracked_objects.values()
                               if obj.in_restricted_area)
        warnings_issued = sum(1 for obj in self.tracked_objects.values()
                            if obj.warning_issued)

        return {
            'total_tracked_objects': total_objects,
            'objects_in_restricted_areas': restricted_objects,
            'warnings_issued': warnings_issued,
            'monitored_areas': len(self.restricted_areas)
        }

    def reset_tracking(self) -> None:
        """Reset all tracking data"""
        self.tracked_objects.clear()
        self.next_object_id = 0
        print("🔄 Tracking data reset")

    def set_audio_enabled(self, enabled: bool) -> None:
        """Enable or disable audio warnings"""
        self.audio_enabled = enabled
        print(f"🔊 Audio warnings {'enabled' if enabled else 'disabled'}")