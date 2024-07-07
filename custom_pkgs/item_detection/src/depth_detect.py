#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class DepthImageProcessor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('depth_image_processor', anonymous=True)

        # Create a CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the depth image topic
        self.subscriber = rospy.Subscriber('/red/camera/depth/image_raw', Image, self.callback)

        # Create a publisher to send out the depth at the center of the image
        self.depth_pub = rospy.Publisher('/red/center_depth', Float32, queue_size=10)

    def callback(self, data):
        try:
            # Convert the depth image to a numpy array
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

            # Assuming the depth image is a single-channel float32 image
            # Calculate the distance at the center of the image
            center_x, center_y = cv_image.shape[1] // 2, cv_image.shape[0] // 2
            center_depth = cv_image[center_y, center_x]

            # Publish the depth value at the center of the image
            self.depth_pub.publish(center_depth)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

if __name__ == '__main__':
    try:
        processor = DepthImageProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

