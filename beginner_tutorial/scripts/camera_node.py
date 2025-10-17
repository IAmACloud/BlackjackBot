#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('camera_node', anonymous=True)
        
        # Get parameters
        self.debug_mode = False
        
        rospy.loginfo(f"[camera_node] Initialized with debug_mode: {self.debug_mode}")
        
        # Create a CV Bridge object
        self.bridge = CvBridge()
          
        # Create a publisher to verify the processed image
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=100)

        # Initialize camera with error checking
        self.video_source = cv2.VideoCapture(1)
        
        # Check if camera opened successfully
        if not self.video_source.isOpened():
            rospy.logerr("[camera_node] Failed to open camera at index 2")
            # Try other camera indices
            for i in range(5):
                rospy.loginfo(f"[camera_node] Trying camera index {i}")
                self.video_source = cv2.VideoCapture(i)
                if self.video_source.isOpened():
                    rospy.loginfo(f"[camera_node] Successfully opened camera at index {i}")
                    break
                self.video_source.release()
            
        if not self.video_source.isOpened():
            rospy.logerr("[camera_node] No camera found!")
            return
            
        # Get camera properties for debugging
        width = self.video_source.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.video_source.get(cv2.CAP_PROP_FRAME_HEIGHT)
        #fps = self.video_source.get(cv2.CAP_PROP_FPS)
        fps = 1
        
        rospy.loginfo(f"[camera_node] Camera properties - Width: {width}, Height: {height}, FPS: {fps}")
        
        # Try to set MJPEG format (optional, might not work on all cameras)
        try:
            self.video_source.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            rospy.loginfo("[camera_node] Set MJPEG format")
        except Exception as e:
            rospy.logwarn(f"[camera_node] Could not set MJPEG format: {e}")

        # Set reasonable resolution if the default is too high
        self.video_source.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.video_source.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # Initialize debug window if debug mode is enabled
        if self.debug_mode:
            try:
                # Test if we can create a window (check for display availability)
                cv2.namedWindow('Camera Feed', cv2.WINDOW_AUTOSIZE)
                rospy.loginfo("[camera_node] Debug window initialized successfully")
            except Exception as e:
                rospy.logwarn(f"[camera_node] Cannot create debug window (no display?): {e}")
                self.debug_mode = False
                rospy.loginfo("[camera_node] Debug mode disabled due to display issues")

    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz
        frame_count = 0
        while not rospy.is_shutdown():
            frameExists, frame = self.video_source.read()
            rospy.loginfo_once(f"Image shape: {frame.shape if frame is not None else 'None'}")
            frame_count += 1
            
            if frameExists and frame is not None:
                # Additional validation - check if frame has reasonable dimensions
                if frame.shape[0] > 0 and frame.shape[1] > 0 and len(frame.shape) == 3:
                    try:
                        # Convert OpenCV image to ROS Image message
                        img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        
                        # Add timestamp and frame info to the header
                        img_msg.header.stamp = rospy.Time.now()
                        img_msg.header.frame_id = "camera_frame"
                        
                        self.image_pub.publish(img_msg)
                        
                        # Log less frequently to avoid spam
                        if frame_count % 30 == 0:  # Log every 3 seconds at 10Hz
                            rospy.loginfo(f"[camera_node] Published image {frame_count}, shape: {frame.shape}")
                        
                        # Show image in debug window if debug mode is enabled
                        if self.debug_mode:
                            try:
                                cv2.imshow('Camera Feed', frame)
                                key = cv2.waitKey(1) & 0xFF
                                if key == ord('q'):
                                    rospy.loginfo("[camera_node] 'q' pressed, shutting down")
                                    rospy.signal_shutdown("User requested shutdown")
                            except Exception as e:
                                rospy.logwarn(f"[camera_node] Display error: {e}")
                                self.debug_mode = False
                                rospy.loginfo("[camera_node] Debug mode disabled due to display error")
                        
                    except CvBridgeError as e:
                        rospy.logerr("CvBridge Error: {0}".format(e))
                else:
                    rospy.logwarn(f"[camera_node] Invalid frame dimensions: {frame.shape if frame is not None else 'None'}")
            else:
                rospy.logwarn(f"[camera_node] No frame captured from camera (attempt {frame_count})")
                
            rate.sleep()

def main():
    camera_node = CameraNode()
    try:
        camera_node.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down camera_node")
    except Exception as e:
        rospy.logerr(f"Camera node error: {e}")
    finally:
        # Clean up resources
        if hasattr(camera_node, 'video_source') and camera_node.video_source.isOpened():
            camera_node.video_source.release()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass  # Ignore cleanup errors
        rospy.loginfo("[camera_node] Cleanup completed")

if __name__ == '__main__':
    main()