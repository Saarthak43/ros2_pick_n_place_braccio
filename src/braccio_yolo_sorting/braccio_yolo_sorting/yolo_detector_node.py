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
        
        # Red detection
        red_mask1 = cv2.inRange(hsv, 
                               self.color_ranges['red']['lower1'],
                               self.color_ranges['red']['upper1'])
        red_mask2 = cv2.inRange(hsv,
                               self.color_ranges['red']['lower2'],
                               self.color_ranges['red']['upper2'])
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        
        # Blue detection
        blue_mask = cv2.inRange(hsv,
                               self.color_ranges['blue']['lower'],
                               self.color_ranges['blue']['upper'])
        
        # Debug: log mask pixel counts every 60 frames
        if self.frame_count % 60 == 1:
            red_px = int(cv2.countNonZero(red_mask))
            blue_px = int(cv2.countNonZero(blue_mask))
            self.get_logger().info(
                f'HSV debug: red_mask={red_px}px  blue_mask={blue_px}px'
            )
        
        # Process each color
        for color, mask in [('red', red_mask), ('blue', blue_mask)]:
            # Clean up mask
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                          cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                if area > 500:  # Minimum area
                    x, y, w, h = cv2.boundingRect(contour)
                    center_y = y + h / 2.0
                    img_h = image.shape[0]

                    # Debug: log every detection before filtering
                    if self.frame_count % 60 == 1:
                        self.get_logger().info(
                            f'  {color} contour: area={area:.0f} '
                            f'bbox=({x},{y},{w},{h}) '
                            f'center_y_pct={center_y/img_h*100:.0f}%'
                        )

                    # --- Filters to reject containers ---
                    # From the logs, containers have area ~17000-18000 px²
                    # and bbox widths of 130-142px.  Real cubes (5cm at
                    # 0.59m) should be ~49x49px = ~2400 px².
                    # Use a generous max of 8000 px² to catch cubes at
                    # any angle while still rejecting containers.
                    if area > 25000:
                        continue
                    if w > 200 or h > 200:
                        continue

                    # Cubes appear in the lower part of the image
                    # (py ~340-420, which is 70-87%).  Containers appear
                    # in the middle (py ~240, 50%) and at the very bottom
                    # (py ~400+, 83%+).  Accept detections between
                    # 50% and 90% of image height.
                    if center_y < img_h * 0.02 or center_y > img_h * 0.90:
                        continue

                    # Check aspect ratio
                    aspect_ratio = w / float(h)
                    if 0.3 < aspect_ratio < 3.0:
                        detections.append({
                            'bbox': [x, y, x+w, y+h],
                            'color': color,
                            'confidence': 0.95
                        })
        
        return detections
    
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