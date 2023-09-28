import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import sys
import time
import numpy as np
import cv2


class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self.point_subscriber = self.create_subscription(Point, '/point_coordinate', self.object_callback, image_qos_profile)
        self.point_subscriber

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer to periodically check for object detection timeout
        self.create_timer(0.1, self.check_detection_timeout)
        self.last_detected_time = None
        self.detection_timeout = 1.0


    def object_callback(self, point):
        robot_velocity = Twist()
        angular_velocity = 0
        width = 320
        
        self.last_detected_time = time.time()  # Update the last detected time
        object_x = point.x
        object_y = point.y

        if width > object_x > (width/2):
            angular_velocity = -0.1
        elif 0 < object_x < (width/2):
            angular_velocity = 0.1
        else:
            angular_velocity = 0.0


        robot_velocity.angular.z = angular_velocity
        self.velocity_publisher.publish(robot_velocity)

    def check_detection_timeout(self):
        # If object hasn't been detected for the specified timeout, stop the robot.
        if self.last_detected_time and time.time() - self.last_detected_time > self.detection_timeout:
            stop_velocity = Twist()
            self.velocity_publisher.publish(stop_velocity)
            self.last_detected_time = None  # Reset


def main(args=None):
    rclpy.init(args=args)
    rotate_robot = RotateRobot()
    rclpy.spin(rotate_robot)
    rotate_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
