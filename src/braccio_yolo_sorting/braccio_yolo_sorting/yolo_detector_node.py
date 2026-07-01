#!/usr/bin/env python3
"""
YOLOv8 Object Detector for Braccio Sorting - Simulation Only
Uses HSV color detection (works perfectly in simulation)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np


class YOLODetectorNode(Node):
    """HSV-based color detector for Braccio sorting."""
    
    def __init__(self):
        super().__init__('yolo_detector_node')
        
        # Parameters
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('image_topic', '/camera/image_raw')
        
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        image_topic = self.get_parameter('image_topic').value
        
        # HSV color ranges (tuned for Gazebo simulation)
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 120, 70]),
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 120, 70]),
                'upper2': np.array([180, 255, 255])
            },
            'blue': {
                'lower': np.array([100, 120, 70]),
                'upper': np.array([130, 255, 255])
            }
        }
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)
        
        self.annotated_pub = self.create_publisher(
            Image, '/detections/annotated', 10)
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        
        self.frame_count = 0
        self.get_logger().info('Braccio YOLO Detector Ready (HSV Color Detection)')
    
    def detect_objects_hsv(self, image):
        """Detect colored objects using HSV."""
        
        detections = []
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # ============================================================
        # TODO (Part 8 — HSV Mask Generation):
        # Generate binary masks for the red and blue cubes.
        #
        # Red wraps around the ends of OpenCV's HSV hue range, so
        # it must be detected using two separate intervals.
        #
        # Steps:
        #   1. Create red_mask1 using red lower1/upper1.
        #   2. Create red_mask2 using red lower2/upper2.
        #   3. Combine them into red_mask using a bitwise OR.
        #   4. Create blue_mask using the blue lower/upper range.
        #
        # Available data:
        #   hsv
        #   self.color_ranges
        #
        # Required output variables:
        #   red_mask
        #   blue_mask
        #
        # The debugging code immediately below this TODO uses both
        # variables, so preserve those exact names.
        # ============================================================

        # ── YOUR CODE HERE ──────────────────────────────────────────
        raise NotImplementedError('HSV mask generation is not implemented yet')
        # ─────────────────────────────────────────────────────────────

## Student completion condition

The periodic debug output must report non-zero mask-pixel counts when the corresponding cubes are visible.
        # Debug: log mask pixel counts every 60 frames
        if self.frame_count % 60 == 1:
            red_px = int(cv2.countNonZero(red_mask))
            blue_px = int(cv2.countNonZero(blue_mask))
            self.get_logger().info(
                f'HSV debug: red_mask={red_px}px  blue_mask={blue_px}px'
            )
        
        # ============================================================
        # TODO (Part 9 — Contour-Based Object Detection):
        # Convert the red and blue masks into valid cube detections.
        #
        # For each colour:
        #
        # Step A — Clean the mask
        #   1. Create a 5×5 uint8 morphology kernel.
        #   2. Apply MORPH_CLOSE to fill small holes.
        #   3. Apply MORPH_OPEN to remove isolated noise.
        #
        # Step B — Extract contours
        #   1. Find external contours only.
        #   2. Compute each contour's area.
        #   3. Reject contours below the minimum useful area.
        #
        # Step C — Build and filter bounding boxes
        #   1. Compute x, y, width, and height.
        #   2. Compute the vertical centre of the box.
        #   3. Reject very large regions belonging to the coloured
        #      destination containers.
        #   4. Reject boxes with excessive width or height.
        #   5. Reject boxes outside the useful image-height region.
        #   6. Reject implausible aspect ratios.
        #
        # Step D — Append accepted detections
        # Append a dictionary with exactly:
        #
        #   {
        #       'bbox': [x_min, y_min, x_max, y_max],
        #       'color': color,
        #       'confidence': confidence
        #   }
        #
        # Use a high fixed confidence for accepted simulation
        # detections, then return the complete `detections` list.
        #
        # Available variables:
        #   image
        #   red_mask
        #   blue_mask
        #   detections
        #   self.frame_count
        #   self.get_logger()
        # ============================================================

        # ── YOUR CODE HERE ──────────────────────────────────────────
        raise NotImplementedError(
            'Contour filtering and detection creation are not implemented yet'
        )
        # ─────────────────────────────────────────────────────────────
    
    def image_callback(self, msg):
        """Process incoming images."""
        
        self.frame_count += 1
        
        try:
            # ros_gz_image publishes as rgb8.  cv_bridge with
            # desired_encoding='bgr8' should convert, but some
            # versions silently pass through when the source is
            # already 8UC3.  Handle both cases explicitly.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert to BGR if the source was RGB
            if msg.encoding in ('rgb8', 'RGB8'):
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

            # Detect objects
            detections = self.detect_objects_hsv(cv_image)
            
            # Create detection array
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            for det in detections:
                detection = Detection2D()
                detection.header = msg.header
                
                # Bounding box
                x1, y1, x2, y2 = det['bbox']
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)
                
                # Hypothesis
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = f'{det["color"]}_cube'
                hypothesis.hypothesis.score = det['confidence']
                
                detection.results.append(hypothesis)
                detection_array.detections.append(detection)
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
            # Publish annotated image (always, so RViz panel isn't blank)
            annotated = self.draw_detections(cv_image, detection_array)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_pub.publish(annotated_msg)
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    f'Frame {self.frame_count}: {len(detection_array.detections)} objects '
                    f'(encoding={msg.encoding}, shape={cv_image.shape})'
                )
        
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
    
    def draw_detections(self, image, detection_array):
        """Draw bounding boxes on image."""
        
        annotated = image.copy()
        
        for detection in detection_array.detections:
            if len(detection.results) == 0:
                continue
            
            cx = int(detection.bbox.center.position.x)
            cy = int(detection.bbox.center.position.y)
            w = int(detection.bbox.size_x)
            h = int(detection.bbox.size_y)
            
            x1 = cx - w // 2
            y1 = cy - h // 2
            x2 = cx + w // 2
            y2 = cy + h // 2
            
            hypothesis = detection.results[0]
            class_name = hypothesis.hypothesis.class_id
            confidence = hypothesis.hypothesis.score
            
            color = (0, 0, 255) if 'red' in class_name else (255, 0, 0)
            
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            
            label = f'{class_name}: {confidence:.2f}'
            cv2.rectangle(annotated, (x1, y1-25), (x1+150, y1), color, -1)
            cv2.putText(annotated, label, (x1+5, y1-8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            cv2.circle(annotated, (cx, cy), 5, color, -1)
        
        return annotated


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
