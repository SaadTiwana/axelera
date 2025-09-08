"""
Logging and Data Management Module
"""
import csv
import json
import os
import time
from datetime import datetime
from typing import List, Dict, Any, Optional
import numpy as np  # Add numpy import
from config import ConfigManager
from detector import DetectionResult


class DetectionLogger:
    """Logs detection results and manages data storage"""

    def __init__(self, config: ConfigManager):
        self.config = config
        self.logging_config = config.logging

        # Ensure directories exist
        os.makedirs(self.logging_config.images_directory, exist_ok=True)
        os.makedirs(os.path.dirname(self.logging_config.detections_file), exist_ok=True)

        # Initialize CSV file with headers if it doesn't exist
        self._initialize_csv()

    def _initialize_csv(self) -> None:
        """Initialize CSV file with headers"""
        if not os.path.exists(self.logging_config.detections_file):
            with open(self.logging_config.detections_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'scan_azimuth', 'scan_elevation',
                    'detection_label', 'confidence', 'bbox_x1', 'bbox_y1',
                    'bbox_x2', 'bbox_y2', 'center_azimuth', 'center_elevation',
                    'area', 'image_filename'
                ])

    def log_detections(self, detections: List[DetectionResult],
                      scan_position: tuple, image_filename: str = "") -> None:
        """Log detection results to CSV file"""
        if not detections:
            return

        scan_azimuth, scan_elevation = scan_position
        timestamp = datetime.now().isoformat()

        with open(self.logging_config.detections_file, 'a', newline='') as f:
            writer = csv.writer(f)

            for detection in detections:
                try:
                    bbox_x1, bbox_y1, bbox_x2, bbox_y2 = detection.bbox
                    center_x, center_y = detection.get_center()
                    area = detection.get_area()

                    # Ensure all values are valid
                    if (all(isinstance(val, (int, float)) and val is not None for val in [bbox_x1, bbox_y1, bbox_x2, bbox_y2, center_x, center_y, area]) and
                        isinstance(detection.confidence, (int, float)) and detection.confidence is not None):

                        # Convert pixel coordinates to angles (this would need camera calibration)
                        # For now, we'll assume there is no distortion in image so pixels per unit angle is constant across the image.
                        # It is also assumed that the camera is aligned to the center of gimbal, meaning center of image corresponds to
                        # angles (scan_azimuth, scan_elevation)
                        # center_x and center_y are normalized coordinates of detection, 0,0 at top-left of image
                        fov_azimuth, fov_elevation = self.config.camera.fov_degrees
                        center_azimuth =    (scan_azimuth - (fov_azimuth/2.0))      + (fov_azimuth * center_x)
                        center_elevation =  (scan_elevation - (fov_elevation/2.0))  + (fov_elevation * center_y)

                        writer.writerow([
                            timestamp,
                            scan_azimuth,
                            scan_elevation,
                            detection.label,
                            detection.confidence,
                            bbox_x1,
                            bbox_y1,
                            bbox_x2,
                            bbox_y2,
                            center_azimuth,
                            center_elevation,
                            area,
                            image_filename
                        ])
                    else:
                        print(f"⚠️ Skipping detection with invalid values: {detection}")
                        continue
                except (ValueError, TypeError, AttributeError) as e:
                    print(f"⚠️ Error logging detection: {e}, detection: {detection}")
                    continue

        print(f"📝 Logged {len(detections)} detections")

    def save_image(self, image: np.ndarray, detections: List[DetectionResult],
                   scan_position: tuple, alarm_detections: Optional[List[DetectionResult]] = None) -> str:
        """Save image with detections to file"""
        try:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            scan_azimuth, scan_elevation = scan_position

            filename = f"{timestamp}_{scan_azimuth:05.1f}_{scan_elevation:05.1f}.jpg"
            filepath = os.path.join(self.logging_config.images_directory, filename)

            # Draw detections on image
            try:
                import cv2
                annotated_image = self._draw_detections_on_image(image, detections, alarm_detections)
                cv2.imwrite(filepath, annotated_image)
            except ImportError:
                # Save without annotations if OpenCV not available
                cv2.imwrite(filepath, image)
            except Exception as e:
                print(f"⚠️ Error saving annotated image: {e}, saving without annotations")
                cv2.imwrite(filepath, image)

            print(f"💾 Image saved: {filepath}")
            return filename

        except Exception as e:
            print(f"❌ Failed to save image: {e}")
            return ""

    def _draw_detections_on_image(self, image: np.ndarray,
                                   detections: List[DetectionResult],
                                   alarm_detections: Optional[List[DetectionResult]] = None) -> np.ndarray:
        """Draw detection bounding boxes on image with red boxes for alarm detections"""
        try:
            import cv2

            result_image = image.copy()
            height, width = image.shape[:2]

            # Create set of alarm detection IDs for quick lookup
            alarm_detection_ids = set()
            if alarm_detections:
                for alarm_det in alarm_detections:
                    # Create a unique ID for each alarm detection
                    alarm_detection_ids.add(id(alarm_det))

            for detection in detections:
                try:
                    # Convert normalized coordinates to pixel coordinates
                    x1, y1, x2, y2 = detection.bbox
                    if all(isinstance(val, (int, float)) and val is not None for val in [x1, y1, x2, y2]):
                        px1 = int(x1 * width)
                        py1 = int(y1 * height)
                        px2 = int(x2 * width)
                        py2 = int(y2 * height)

                        # Check if this detection is triggering an alarm
                        is_alarm = id(detection) in alarm_detection_ids

                        # Choose color: Red for alarm detections, Green for normal
                        if is_alarm:
                            color = (0, 0, 255)  # BGR: Red
                            label_color = (0, 0, 255)  # BGR: Red
                            thickness = 3  # Thicker line for alarm
                        else:
                            color = (0, 255, 0)  # BGR: Green
                            label_color = (0, 255, 0)  # BGR: Green
                            thickness = 2  # Normal thickness

                        # Draw bounding box
                        cv2.rectangle(result_image, (px1, py1), (px2, py2), color, thickness)

                        # Draw label
                        label = f"{detection.label} {detection.confidence:.2f}"
                        if is_alarm:
                            label += " [ALARM]"
                        cv2.putText(result_image, label, (px1, py1 - 10),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, label_color, 2)
                except (ValueError, TypeError):
                    print(f"⚠️ Skipping detection with invalid bbox: {detection}")
                    continue

            return result_image

        except ImportError:
            print("⚠️ OpenCV not available for drawing detections")
            return image
        except Exception as e:
            print(f"❌ Error drawing detections: {e}")
            return image

    def get_detection_history(self, hours: int = 24) -> List[Dict[str, Any]]:
        """Get detection history from the last N hours"""
        try:
            cutoff_time = time.time() - (hours * 3600)
            history = []

            with open(self.logging_config.detections_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    # Parse timestamp and check if within time window
                    try:
                        row_time = datetime.fromisoformat(row['timestamp']).timestamp()
                        if row_time >= cutoff_time:
                            # Convert string values to appropriate types
                            row['scan_azimuth'] = float(row['scan_azimuth'])
                            row['scan_elevation'] = float(row['scan_elevation'])
                            row['confidence'] = float(row['confidence'])
                            row['bbox_x1'] = float(row['bbox_x1'])
                            row['bbox_y1'] = float(row['bbox_y1'])
                            row['bbox_x2'] = float(row['bbox_x2'])
                            row['bbox_y2'] = float(row['bbox_y2'])
                            row['center_azimuth'] = float(row['center_azimuth'])
                            row['center_elevation'] = float(row['center_elevation'])
                            row['area'] = float(row['area'])

                            history.append(row)
                    except (ValueError, KeyError):
                        continue

            return history

        except FileNotFoundError:
            return []
        except Exception as e:
            print(f"❌ Error reading detection history: {e}")
            return []

    def get_statistics(self, hours: int = 24) -> Dict[str, Any]:
        """Get detection statistics"""
        history = self.get_detection_history(hours)

        if not history:
            return {'total_detections': 0, 'unique_labels': [], 'avg_confidence': 0.0}

        total_detections = len(history)
        labels = [h['detection_label'] for h in history]
        confidences = [h['confidence'] for h in history]

        # Count unique labels
        unique_labels = list(set(labels))
        label_counts = {}
        for label in labels:
            label_counts[label] = label_counts.get(label, 0) + 1

        return {
            'total_detections': total_detections,
            'unique_labels': unique_labels,
            'label_counts': label_counts,
            'avg_confidence': sum(confidences) / len(confidences) if confidences else 0.0,
            'time_period_hours': hours
        }

    def cleanup_old_data(self, days_to_keep: int = 30) -> int:
        """Remove detection data older than specified days"""
        try:
            cutoff_time = time.time() - (days_to_keep * 24 * 3600)
            temp_file = self.logging_config.detections_file + '.tmp'
            removed_count = 0

            with open(self.logging_config.detections_file, 'r') as f_in, \
                 open(temp_file, 'w', newline='') as f_out:

                reader = csv.reader(f_in)
                writer = csv.writer(f_out)

                # Copy header
                header = next(reader)
                writer.writerow(header)

                # Copy recent entries
                for row in reader:
                    try:
                        row_time = datetime.fromisoformat(row[0]).timestamp()
                        if row_time >= cutoff_time:
                            writer.writerow(row)
                        else:
                            removed_count += 1
                    except (ValueError, IndexError):
                        # Keep malformed rows
                        writer.writerow(row)

            # Replace original file
            os.replace(temp_file, self.logging_config.detections_file)

            if removed_count > 0:
                print(f"🧹 Cleaned up {removed_count} old detection records")

            return removed_count

        except Exception as e:
            print(f"❌ Error during cleanup: {e}")
            return 0