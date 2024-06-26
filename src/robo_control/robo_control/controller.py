import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robo_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10)
        self.camera_subscription = self.create_subscription(
            Image,
            '/images_l16',
            self.camera_callback,
            10)
        self.detection_subscription = self.create_subscription(
            String,
            '/detection_results',
            self.detection_callback,
            10)
        self.bridge = CvBridge()
        self.lidar_data = []
        self.camera_data = None
        self.detections = []

    def lidar_callback(self, msg):
        self.lidar_data = msg.ranges
        self.avoid_obstacles()

    def camera_callback(self, msg):
        self.camera_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.avoid_obstacles()

    def detection_callback(self, msg):
        self.detections = json.loads(msg.data)
        self.avoid_obstacles()

    def avoid_obstacles(self):
        if not self.lidar_data or self.camera_data is None:
            return

        data = Twist()
        min_distance = 1.0  # Minimum safe distance to obstacles
        front_left = min(self.lidar_data[0:90])  # Check front left 90 degrees
        front_right = min(self.lidar_data[270:360])  # Check front right 90 degrees
        front_center = min(self.lidar_data[90:270])  # Check front center 180 degrees

        obstacle_detected = False
        if front_center < min_distance:
            obstacle_detected = True
            if front_left < front_right:
                # Obstacle is closer on the left, turn right
                data.linear.x = 0.0
                data.angular.z = -1.0
            else:
                # Obstacle is closer on the right, turn left
                data.linear.x = 0.0
                data.angular.z = 1.0
        elif front_left < min_distance:
            obstacle_detected = True
            data.linear.x = 0.0
            data.angular.z = -1.0
        elif front_right < min_distance:
            obstacle_detected = True
            data.linear.x = 0.0
            data.angular.z = 1.0

        if not obstacle_detected:
            # No obstacle in close range, proceed with camera-based navigation
            obstacle_in_camera_view = self.detect_obstacle_in_camera()
            if obstacle_in_camera_view:
                data.linear.x = 0.0
                data.angular.z = 1.0
            else:
                data.linear.x = 0.5
                data.angular.z = 0.0

        self.publisher_.publish(data)

    def detect_obstacle_in_camera(self):
        # Convert the image to grayscale
        gray = cv2.cvtColor(self.camera_data, cv2.COLOR_BGR2GRAY)
        # Use a simple threshold to detect obstacles (this can be replaced with a more sophisticated method)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        # Check if there are any white pixels (indicating an obstacle) in the central region of the image
        height, width = thresh.shape
        central_region = thresh[height // 2:, width // 3:2 * width // 3]
        return np.any(central_region == 255)

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
