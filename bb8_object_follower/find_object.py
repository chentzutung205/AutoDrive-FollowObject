import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge

import sys

import cv2
import numpy as np


class ImageSubscriber(Node):

    def __init__(self):
        # Creates the node.
        super().__init__('image_subscriber')

        # Set Parameters
        self.declare_parameter('show_image_bool', True)
        self.declare_parameter('window_name', "raw_image")

        # Determine Window Showing Based on Input
        self._display_image = bool(self.get_parameter('show_image_bool').value)

        # Declare some variables
        self._titleOriginal = self.get_parameter('window_name').value  # Image Window Title

        # Only create image frames if we are not running headless (_display_image sets this)
        # if (self._display_image):
        #     # Set Up Image Viewing
        #     cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE)  # Viewing Window
        #     cv2.moveWindow(self._titleOriginal, 50, 50)  # Viewing Window Original Location

        # Set up QoS Profiles for passing images over Wi-Fi
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # Declare that the image_subscriber node is subscribing to the /camera/image/compressed topic.
        self._image_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback,
            image_qos_profile)
        self._image_subscriber  # Prevents unused variable warning.

        self._coordinate_publisher = self.create_publisher(Point, '/point_coordinate', 10)


    def publish_point(self, circles):
       if circles is not None:
           for circle in circles[0, :]:
               point = Point()
               point.x = float(circle[0])
               point.y = float(circle[1])
               point.z = 0.0
               self._coordinate_publisher.publish(point)

    def _image_callback(self, CompressedImage):
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        # Image Processing to find circle
        imgGRAY = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2GRAY)
        imgBLUR = cv2.medianBlur(imgGRAY, 5)
        circles = cv2.HoughCircles(imgBLUR, cv2.HOUGH_GRADIENT, 1, 80, param1=110, param2=63, minRadius=0, maxRadius=0)

        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                # Print the pixel location
                xc, yc, r = circle[0], circle[1], circle[2]
                # draw the outer circle (bounding boxes)
                cv2.circle(self._imgBGR, (xc, yc), r, (255, 0, 0), 7)
                # draw the center of the circle
                cv2.circle(self._imgBGR, (xc, yc), 2, (0, 0, 255), 3)

        # if (self._display_image):
        #     # Display the image in a window
        #     self.show_image(self._imgBGR)
        #     cv2.waitKey(1)

        self.publish_point(circles)


    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
        # Cause a slight delay so image is displayed
        self._user_input = cv2.waitKey(10)  # Use OpenCV keystroke grabber for delay.

    def get_user_input(self):
        return self._user_input


def main():
    rclpy.init()  # init routine needed for ROS2.
    image_subscriber = ImageSubscriber()  # Create class object to be used.
    rclpy.spin(image_subscriber)  # Trigger callback processing.
    
    # while rclpy.ok():
    #     rclpy.spin(image_subscriber)  # Trigger callback processing.
    #     if image_subscriber._display_image:
    #         if image_subscriber.get_user_input() == ord('q'):
    #             cv2.destroyAllWindows()
    #             break

    # Clean up and shutdown.
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
