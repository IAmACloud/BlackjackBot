#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('camera_node', anonymous=True)
        
        # Create a CV Bridge object
        self.bridge = CvBridge()
        
        # Create a subscriber for the camera image topic
        # Change 'camera/image_raw' to match your camera's topic
        self.image_sub = rospy.Subscriber('camera/image_raw', Image, self.image_callback)
        
        # For debugging: create a publisher to verify the processed image
        self.image_pub = rospy.Publisher('camera/processed_image', Image, queue_size=1)
        
    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Here you can add your image processing code
        # For example, display the image
        cv2.imshow("Camera Feed", cv_image)
        cv2.waitKey(1)

        try:
            # Publish processed image (for debugging)
            processed_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(processed_msg)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

def main():
    camera_node = CameraNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down camera_node")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()