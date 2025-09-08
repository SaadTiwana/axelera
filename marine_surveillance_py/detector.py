"""
Object Detection Module using Degirum SDK
"""
import numpy as np
import time
import cv2
from typing import List, Dict, Any, Optional, Tuple
from config import ConfigManager


class DetectionResult:
    """Represents a single detection result"""

    def __init__(self, label: str, confidence: float, bbox: Tuple[float, float, float, float],
                 class_id: int = 0):
        self.label = label
        self.confidence = confidence
        self.bbox = bbox  # (x1, y1, x2, y2) in normalized coordinates
        self.class_id = class_id

    def __repr__(self) -> str:
        try:
            x1, y1, x2, y2 = self.bbox
            if all(isinstance(val, (int, float)) and val is not None for val in [x1, y1, x2, y2]):
                return f"Detection(label={self.label}, conf={self.confidence:.2f}, bbox=({x1:.2f}, {y1:.2f}, {x2:.2f}, {y2:.2f}))"
            else:
                return f"Detection(label={self.label}, conf={self.confidence:.2f}, bbox=invalid)"
        except (ValueError, TypeError):
            return f"Detection(label={self.label}, conf={self.confidence:.2f}, bbox=error)"

    def get_center(self) -> Tuple[float, float]:
        """Get center point of bounding box"""
        try:
            x1, y1, x2, y2 = self.bbox
            if all(isinstance(val, (int, float)) and val is not None for val in [x1, y1, x2, y2]):
                return ((x1 + x2) / 2, (y1 + y2) / 2)
            else:
                return (0.0, 0.0)
        except (ValueError, TypeError):
            return (0.0, 0.0)

    def get_area(self) -> float:
        """Get area of bounding box"""
        try:
            x1, y1, x2, y2 = self.bbox
            if all(isinstance(val, (int, float)) and val is not None for val in [x1, y1, x2, y2]):
                return (x2 - x1) * (y2 - y1)
            else:
                return 0.0
        except (ValueError, TypeError):
            return 0.0


class ObjectDetector:
    """Object detection using Degirum SDK with Axelera Metis"""

    def __init__(self, config: ConfigManager):
        self.config = config
        self.detection_config = config.detection
        self.model = None
        self.initialized = False

    def initialize(self) -> bool:
        """Initialize Degirum model"""
        try:
            # Import Degirum SDK
            import degirum as dg

            print("🔧 Initializing Degirum SDK...")

            print(f"   Model path: {self.detection_config.model_path}")
            print(f"   Model name: {self.detection_config.model_name}")

            # Create model instance
            '''
            self.model = dg.load_model(
                model_path=self.detection_config.model_path,
                model_name = self.detection_config.model_name,
                inference_host_address="@local",
                token="",
                overlay_font_scale=1.0,
                overlay_alpha=0.5,
                overlay_show_probabilities=True
            )'''
            hw_location = "@local"
            model_name = "yolo11m_marine_detector_v4--640x640_quant_axelera_metis_2"
            #zoo_name = "file://../my-zoo/axelera/"
            zoo_name = "../my-zoo/axelera/"
            self.model = dg.load_model(model_name, hw_location, zoo_name, image_backend='pil')

            # Configure model
            self.model.confidence_threshold = self.detection_config.confidence_threshold
            self.model.input_numpy_colorspace = "RGB"

            print("✅ Degirum model initialized")
            print(f"   Model: {self.detection_config.model_path}")
            print(f"   Confidence threshold: {self.detection_config.confidence_threshold}")
            print(f"   Tile size: {self.detection_config.tile_size}")

            self.initialized = True
            return True

        except ImportError:
            print("❌ Degirum SDK not found. Please install: pip install degirum")
            return False
        except Exception as e:
            print(f"❌ Model initialization failed: {e}")
            return False

    def detect_objects(self, image: np.ndarray) -> List[DetectionResult]:
        """Run object detection on image"""
        if not self.initialized or self.model is None:
            print("❌ Detector not initialized")
            return []

        try:
            # Validate image
            if image is None:
                print("❌ Detection failed: image is None")
                return []

            if not hasattr(image, 'shape') or len(image.shape) < 2:
                print(f"❌ Detection failed: invalid image shape: {getattr(image, 'shape', 'no shape')}")
                return []

            height, width = image.shape[:2]
            if height == 0 or width == 0:
                print(f"❌ Detection failed: invalid image dimensions: {width}x{height}")
                return []

            # Convert BGR to RGB if needed
            if len(image.shape) == 3 and image.shape[2] == 3:
                rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                rgb_image = image

            # Run inference with timeout
            print(f"🎯 Starting inference on image shape: {rgb_image.shape}")
            start_time = time.time()

            # Add timeout to prevent hanging
            import threading
            result = [None]
            exception = [None]

            def run_inference():
                try:
                    print(f"************************SAAD: {self.model(rgb_image)}")
                    result[0] = self.model(rgb_image)
                except Exception as e:
                    exception[0] = e

            inference_thread = threading.Thread(target=run_inference)
            inference_thread.start()
            inference_thread.join(timeout=30.0)  # 30 second timeout

            if inference_thread.is_alive():
                print("❌ Inference timed out after 30 seconds")
                return []
            elif exception[0] is not None:
                print(f"❌ Inference failed with exception: {exception[0]}")
                return []

            predictions = result[0]
            inference_time = time.time() - start_time

            # Handle Degirum DetectionResults object
            try:
                if hasattr(predictions, '__iter__'):
                    # Try to iterate directly
                    predictions_list = list(predictions)
                    print(f"🎯 Inference completed in {(inference_time * 1000):.1f}ms, got {len(predictions_list)} predictions")
                elif hasattr(predictions, '__len__'):
                    # Try to get length directly
                    print(f"🎯 Inference completed in {(inference_time * 1000):.1f}ms, got {len(predictions)} predictions")
                else:
                    # Try to access as object attributes
                    print(f"🎯 Inference completed in {(inference_time * 1000):.1f}ms, predictions type: {type(predictions)}")
                    predictions_list = []
                    # Try common Degirum result access patterns
                    print(f"🎯 Prediction object attributes: {dir(predictions)}")
                    if hasattr(predictions, 'results'):
                        predictions_list = predictions.results
                        print(f"🎯 Found results attribute")
                    elif hasattr(predictions, 'detections'):
                        predictions_list = predictions.detections
                        print(f"🎯 Found detections attribute")
                    elif hasattr(predictions, 'predictions'):
                        predictions_list = predictions.predictions
                        print(f"🎯 Found predictions attribute")
                    else:
                        # Try to convert to list
                        try:
                            predictions_list = list(predictions)
                            print(f"🎯 Converted to list successfully")
                        except Exception as list_error:
                            print(f"⚠️ Could not convert to list: {list_error}")
                            predictions_list = [predictions]  # Single result
                            print(f"🎯 Treating as single result")
                    print(f"🎯 Extracted {len(predictions_list) if hasattr(predictions_list, '__len__') else 'unknown'} predictions from results")
                    print(f"🎯 First prediction sample: {predictions_list[0] if predictions_list and len(predictions_list) > 0 else 'None'}")
            except Exception as result_error:
                print(f"⚠️ Error processing prediction results: {result_error}")
                predictions_list = []

            # Process results
            detections = []
            for pred in predictions_list:
                try:
                    # Extract bounding box coordinates (normalized 0-1)
                    if 'bbox' not in pred or pred['bbox'] is None:
                        print(f"⚠️ Skipping detection with missing bbox: {pred}")
                        continue

                    bbox_data = pred['bbox']
                    # Handle different bbox formats from Degirum
                    try:
                        if isinstance(bbox_data, list) and len(bbox_data) == 4:
                            # Format: [x1, y1, x2, y2] (pixel coordinates)
                            x1, y1, x2, y2 = bbox_data
                        elif isinstance(bbox_data, dict):
                            # Format: {'x1': val, 'y1': val, 'x2': val, 'y2': val}
                            x1 = bbox_data.get('x1')
                            y1 = bbox_data.get('y1')
                            x2 = bbox_data.get('x2')
                            y2 = bbox_data.get('y2')
                        else:
                            print(f"⚠️ Unsupported bbox format: {type(bbox_data)}, data: {bbox_data}")
                            continue

                        if any(val is None for val in [x1, y1, x2, y2]):
                            print(f"⚠️ Skipping detection with None bbox values: {bbox_data}")
                            continue

                        # Convert to normalized coordinates (0-1)
                        bbox = (
                            float(x1) / width,  # x1
                            float(y1) / height,  # y1
                            float(x2) / width,  # x2
                            float(y2) / height   # y2
                        )
                    except (ValueError, TypeError, KeyError, IndexError) as bbox_error:
                        print(f"⚠️ Error extracting bbox coordinates: {bbox_error}, bbox_data: {bbox_data}")
                        continue

                    # Validate bbox values
                    if any(not isinstance(val, (int, float)) or val is None for val in bbox):
                        print(f"⚠️ Skipping detection with invalid bbox values: {bbox}")
                        continue

                    # Safely extract other detection parameters
                    try:
                        label = str(pred.get('label', 'unknown')) if pred.get('label') is not None else 'unknown'
                        # Try different confidence key names (Degirum uses 'score' sometimes)
                        confidence_val = pred.get('confidence') or pred.get('score') or 0.0
                        confidence = float(confidence_val)
                        class_id_val = pred.get('class_id', 0)
                        class_id = int(class_id_val) if class_id_val is not None else 0

                        detection = DetectionResult(
                            label=label,
                            confidence=confidence,
                            bbox=bbox,
                            class_id=class_id
                        )
                    except (ValueError, TypeError) as param_error:
                        print(f"⚠️ Error extracting detection parameters: {param_error}, pred: {pred}")
                        continue

                    detections.append(detection)

                except (KeyError, ValueError, TypeError) as e:
                    print(f"⚠️ Error processing detection: {e}, pred: {pred}")
                    continue

            print(f"🎯 Detected {len(detections)} objects in {(inference_time * 1000):.1f}ms")
            return detections

        except Exception as e:
            print(f"❌ Detection failed: {e}")
            return []

    def detect_with_tiling(self, image: np.ndarray) -> List[DetectionResult]:
        """Run detection with proper Degirum tiling for large images"""
        print(f"🔲 Starting detect_with_tiling using Degirum tools, image shape: {image.shape}")
        if not self.initialized or self.model is None:
            print("❌ Detector not initialized")
            return []

        try:
            # Import Degirum tiling tools
            from degirum_tools.tile_compound_models import TileExtractorPseudoModel, BoxFusionLocalGlobalTileModel
            from degirum_tools import NmsOptions, NmsBoxSelectionPolicy

            # Configure tiling parameters
            cols = 3  # Number of columns for tiling
            rows = 2  # Number of rows for tiling
            overlap_percent = 0.1  # 10% overlap

            print(f"🔲 Tiling configuration: {cols}x{rows} grid, {overlap_percent*100}% overlap")

            # Set up NMS options for proper detection fusion
            nms_options = NmsOptions(
                threshold=0.2,
                use_iou=True,
                box_select=NmsBoxSelectionPolicy.MOST_PROBABLE,
            )

            # Create tile extractor
            tile_extractor = TileExtractorPseudoModel(
                cols=cols,
                rows=rows,
                overlap_percent=overlap_percent,
                model2=self.model,
                global_tile=True
            )

            # Create fused tile model for proper detection merging
            tile_model = BoxFusionLocalGlobalTileModel(
                model1=tile_extractor,
                model2=self.model,
                large_object_threshold=0.01,
                edge_threshold=0.02,
                fusion_threshold=0.8,
                nms_options=nms_options
            )

            print(f"🔲 Running tiled detection on image...")

            # Start timing for tiled detection
            tiled_start_time = time.time()

            # Convert numpy array to PIL Image for Degirum
            if len(image.shape) == 3 and image.shape[2] == 3:
                # Convert BGR to RGB
                rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            else:
                rgb_image = image

            # Convert to PIL Image
            from PIL import Image
            pil_image = Image.fromarray(rgb_image)

            # Run tiled detection
            results = tile_model(pil_image)
            print(f"************************SAAD: {results}")

            # Calculate tiled detection time
            tiled_detection_time = time.time() - tiled_start_time

            print(f"🔲 Tiled detection completed in {(tiled_detection_time * 1000):.1f}ms, processing results...")

            # Extract detections from Degirum results
            detections = []
            height, width = image.shape[:2]

            # Access the results from the DetectionResults object
            print(f"🔍 Tiled results object type: {type(results)}")
            print(f"🔍 Tiled results attributes: {dir(results)}")

            if hasattr(results, 'results'):
                predictions = results.results
                print(f"🔍 Found results attribute, type: {type(predictions)}")
                if hasattr(predictions, '__len__'):
                    print(f"🔍 Results length: {len(predictions)}")
                else:
                    print(f"🔍 Results has no length, trying to convert to list")
                    try:
                        predictions = list(predictions)
                        print(f"🔍 Converted results to list, length: {len(predictions)}")
                    except Exception as convert_error:
                        print(f"⚠️ Could not convert results to list: {convert_error}")
                        return []
            elif hasattr(results, '__iter__'):
                predictions = list(results)
                print(f"🔍 Converted iterable results to list, length: {len(predictions)}")
            else:
                print(f"⚠️ Could not extract predictions from tiled results - no results attribute or iterable")
                return []

            print(f"🔲 Extracted {len(predictions)} predictions from tiled detection")

            for pred in predictions:
                try:
                    # Extract bounding box coordinates
                    if 'bbox' not in pred or pred['bbox'] is None:
                        continue

                    bbox_data = pred['bbox']

                    # Handle different bbox formats from Degirum
                    try:
                        if isinstance(bbox_data, list) and len(bbox_data) == 4:
                            # Format: [x1, y1, x2, y2] (pixel coordinates)
                            x1, y1, x2, y2 = bbox_data
                        elif isinstance(bbox_data, dict):
                            # Format: {'x1': val, 'y1': val, 'x2': val, 'y2': val}
                            x1 = bbox_data.get('x1')
                            y1 = bbox_data.get('y1')
                            x2 = bbox_data.get('x2')
                            y2 = bbox_data.get('y2')
                        else:
                            print(f"⚠️ Unsupported bbox format: {type(bbox_data)}, data: {bbox_data}")
                            continue

                        if any(val is None for val in [x1, y1, x2, y2]):
                            print(f"⚠️ Skipping detection with None bbox values: {bbox_data}")
                            continue

                        # Convert to normalized coordinates (0-1)
                        bbox = (
                            float(x1) / width,  # x1
                            float(y1) / height,  # y1
                            float(x2) / width,  # x2
                            float(y2) / height   # y2
                        )
                    except (ValueError, TypeError, KeyError, IndexError) as bbox_error:
                        print(f"⚠️ Error extracting bbox coordinates: {bbox_error}, bbox_data: {bbox_data}")
                        continue

                    # Extract other detection parameters
                    label = str(pred.get('label', 'unknown'))
                    # Try different confidence key names (Degirum uses 'score' sometimes)
                    confidence_val = pred.get('confidence') or pred.get('score') or 0.0
                    confidence = float(confidence_val)
                    class_id = int(pred.get('class_id', 0))

                    detection = DetectionResult(
                        label=label,
                        confidence=confidence,
                        bbox=bbox,
                        class_id=class_id
                    )

                    detections.append(detection)

                except (KeyError, ValueError, TypeError) as e:
                    print(f"⚠️ Error processing tiled detection: {e}")
                    continue

            print(f"🎯 Final detections after proper tiling: {len(detections)} in {(tiled_detection_time * 1000):.1f}ms")
            return detections

        except ImportError as e:
            print(f"❌ Degirum tiling tools not available: {e}")
            return []
        except Exception as e:
            print(f"❌ Tiled detection failed: {e}")
            return []

    def _non_max_suppression(self, detections: List[DetectionResult],
                           iou_threshold: float = 0.5) -> List[DetectionResult]:
        """Apply Non-Maximum Suppression to remove overlapping detections"""
        if not detections:
            return detections

        # Sort by confidence (highest first)
        detections.sort(key=lambda x: x.confidence, reverse=True)

        kept_detections = []

        while detections:
            # Keep the detection with highest confidence
            best = detections.pop(0)
            kept_detections.append(best)

            # Remove detections that overlap too much with the kept detection
            detections = [
                det for det in detections
                if self._calculate_iou(best.bbox, det.bbox) < iou_threshold
            ]

        return kept_detections

    def _calculate_iou(self, bbox1: Tuple[float, float, float, float],
                      bbox2: Tuple[float, float, float, float]) -> float:
        """Calculate Intersection over Union (IoU) of two bounding boxes"""
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

    def draw_detections(self, image: np.ndarray, detections: List[DetectionResult],
                       alarm_detections: Optional[List[DetectionResult]] = None) -> np.ndarray:
        """Draw detection results on image with red boxes for alarm detections"""
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
                    print(f"⚠️ Skipping detection with invalid bbox for drawing: {detection}")
                    continue

            return result_image

        except ImportError:
            print("⚠️ OpenCV not available for drawing detections")
            return image
        except Exception as e:
            print(f"❌ Error drawing detections: {e}")
            return image

    def get_detection_stats(self, detections: List[DetectionResult]) -> Dict[str, Any]:
        """Get statistics about detections"""
        if not detections:
            return {'count': 0}

        labels = [d.label for d in detections]
        confidences = [d.confidence for d in detections]

        # Count by label
        label_counts = {}
        for label in labels:
            label_counts[label] = label_counts.get(label, 0) + 1

        return {
            'count': len(detections),
            'labels': label_counts,
            'avg_confidence': sum(confidences) / len(confidences),
            'max_confidence': max(confidences),
            'min_confidence': min(confidences)
        }