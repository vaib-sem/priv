import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import json
import requests

# Import the inference SDK
from inference_sdk import InferenceHTTPClient

# Initialize the client
CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",
    api_key="alK8EEqJGxv5cm4KiuF0"
)

class ModelInference(Node):
    def __init__(self):
        super().__init__('model_inference')
        self.publisher_ = self.create_publisher(String, '/detection_results', 10)
        self.camera_subscription = self.create_subscription(
            Image,
            '/images_l16',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def infer_on_frame(self, frame):
        # Save the frame to a temporary file
        temp_filename = "temp_frame.jpg"
        cv2.imwrite(temp_filename, frame)
        
        # Infer on the saved image
        result = CLIENT.infer(temp_filename, model_id="summer_project_morphobot/1")
        
        # Process the result
        return result

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Perform inference on the current frame
        result = self.infer_on_frame(frame)
        detections = result.get("predictions", [])
        
        detection_str = json.dumps(detections)
        
        # Publish detection results
        detection_msg = String()
        detection_msg.data = detection_str
        self.publisher_.publish(detection_msg)
        
        # Display the resulting frame
        for detection in detections:
            x, y, w, h = detection["x"], detection["y"], detection["width"], detection["height"]
            class_name = detection["class"]
            confidence = detection["confidence"]

            # Calculate bounding box coordinates
            x1 = int(x - w / 2)
            y1 = int(y - h / 2)
            x2 = int(x + w / 2)
            y2 = int(y + h / 2)
            
            # Draw the bounding box and label on the frame
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{class_name}: {confidence:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('Live Feed', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    model_inference = ModelInference()
    rclpy.spin(model_inference)
    model_inference.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
