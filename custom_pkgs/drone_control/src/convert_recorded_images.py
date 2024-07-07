#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import random
import os
from pynput.keyboard import Listener, Key

def generate_random_digits(num_digits):
    range_start = 10**(num_digits-1)
    range_end = (10**num_digits)-1
    return random.randint(range_start, range_end)

def image_callback(msg):
    global latest_image
    try:
        bridge = CvBridge()
        latest_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("Could not convert image: %s" % e)
        
def ensure_directory_exists(path):
    if not os.path.exists(path):
        os.makedirs(path)

def fetch_and_show_image():
    try:
        ensure_directory_exists('./images')
        #image_msg = rospy.wait_for_message("/red/camera/color/image_raw", Image, timeout=5)
        #bridge = CvBridge()
        #cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
        rospy.loginfo("Image converted successfully.")
        
        rand_num = generate_random_digits(6)
        filename = f'./images/flower1{rand_num}.jpg'
        cv2.imwrite(filename, latest_image)
        rospy.loginfo(f"Image saved as {filename}.")

        cv2.imshow("Color Image Window", latest_image)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
    except rospy.ROSException as e:
        rospy.logerr(f"Failed to receive image message: {e}")
    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert image: {e}")
    except Exception as e:
        rospy.logerr(f"An error occurred: {e}")

def on_press(key):
    if key == Key.space:
        rospy.loginfo("Space key pressed.")
        fetch_and_show_image()

def record():
    rospy.init_node('convert_recorded_images', anonymous=True)
    rospy.Subscriber("/red/camera/color/image_raw", Image, image_callback)

    rospy.loginfo("Node initialized.")
    listener = Listener(on_press=on_press)
    listener.start()
    rospy.spin()

if __name__ == '__main__':
    try:
        record()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node interrupted and shutting down.")

