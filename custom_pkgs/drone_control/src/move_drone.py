#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from pynput.keyboard import Key, Listener, KeyCode

# Initialize the publisher globally
pub = None

def on_press(key):
    global pub
    # Initialize movement adjustments
    x, y, z = 0, 0, 0

    # Define movement for each key press
    if key == Key.up:
        x = 1.0  # Move forward
    elif key == Key.down:
        x = -1.0  # Move backward
    elif key == Key.left:
        y = 1.0  # Move left
    elif key == Key.right:
        y = -1.0  # Move right
    elif key == KeyCode(char='w'):
        z = 1.0  # Move upward
    elif key == KeyCode(char='s'):
        z = -1.0  # Move downward
    else:
        return  # No movement for other keys
        
    rospy.loginfo(f"Move_drone node delta: {x} {y} {z} ")
    
    try:
        pose_message = rospy.wait_for_message('/red/pose', PoseStamped, timeout=1)
        pose_message.pose.position.x += x
        pose_message.pose.position.y += y
        pose_message.pose.position.z += z
        rospy.loginfo(f"Move_drone node msg: {pose_message.pose.position.x}  {pose_message.pose.position.y} {pose_message.pose.position.z}")
        pub.publish(pose_message)  # Publish the updated pose
    except rospy.ROSException as e:
        rospy.logwarn("Failed to receive pose message: {}".format(e))


def move_drone():
    global pub
    rospy.init_node('move_drone', anonymous=True)
    pub = rospy.Publisher('/red/tracker/input_pose', PoseStamped, queue_size=10)
    listener = Listener(on_press=on_press)
    listener.start()  # Start the listener
    rospy.spin()  # Keep the node alive until it's stopped

if __name__ == '__main__':
    try:
        move_drone()
    except rospy.ROSInterruptException:
        pass

