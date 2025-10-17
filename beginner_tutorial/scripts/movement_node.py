#!/usr/bin/env python3

# Movement node for ROS Noetic
# Controls the robotic arm for blackjack game
# Communicates with Arduino for motor control

import rospy
from std_msgs.msg import Bool, String
import serial
import time

class MovementNode:
    def __init__(self):
        rospy.init_node('movement_node', anonymous=False)
        
        # Parameters
        self.serial_port = rospy.get_param('~serial_port', 'COM3')  # Windows default, change to /dev/ttyACM0 for Linux
        self.baud_rate = rospy.get_param('~baud_rate', 9600)
        self.simulation_mode = rospy.get_param('~simulation_mode', True)
        self.movement_timeout = rospy.get_param('~movement_timeout', 5.0)  # seconds to wait for movement
        
        # Serial connection
        if not self.simulation_mode:
            try:
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
                rospy.loginfo(f"Connected to serial port {self.serial_port}")
            except serial.SerialException as e:
                rospy.logerr(f"Failed to connect to serial port: {e}")
                self.simulation_mode = True  # Fall back to simulation
        
        # Publisher
        self.pub_complete = rospy.Publisher('/movement/complete', Bool, queue_size=10)
        
        # Subscriber
        self.sub_control = rospy.Subscriber('/movement/control', String, self.control_callback, queue_size=5)
        
        # State
        self.current_command = None
        
        rospy.loginfo(f"[movement_node] Initialized with simulation_mode: {self.simulation_mode}")
    
    def control_callback(self, msg: String):
        mode = msg.data.strip().upper()
        rospy.loginfo(f"[movement_node] Received command: {mode}")
        
        self.current_command = mode
        
        if self.simulation_mode:
            # Simulate movement
            self.simulate_movement(mode)
            self.publish_complete()
        else:
            # Send to Arduino
            try:
                self.ser.write((mode + '\n').encode())
                # Wait for response
                start_time = time.time()
                while time.time() - start_time < self.movement_timeout:
                    if self.ser.in_waiting > 0:
                        response = self.ser.readline().decode().strip()
                        if response == 'DONE':
                            self.publish_complete()
                            return
                        elif response.startswith('ERROR'):
                            rospy.logerr(f"Arduino error: {response}")
                            self.publish_complete()  # Still complete, even if error
                            return
                # Timeout
                rospy.logwarn(f"Timeout waiting for Arduino response for command: {mode}")
                self.publish_complete()
            except Exception as e:
                rospy.logerr(f"Serial communication error: {e}")
                self.publish_complete()
    
    def simulate_movement(self, mode):
        # Simulate movement time
        if 'TURN' in mode:
            time.sleep(1.0)
        elif 'DEAL' in mode:
            time.sleep(0.5)
        elif 'CAMERA' in mode:
            time.sleep(0.8)
        elif 'CELEBRATE' in mode:
            time.sleep(2.0)
        elif 'TIE' in mode:
            time.sleep(3.0)
        elif 'END_GAME' in mode:
            time.sleep(1.5)
        else:
            time.sleep(0.1)
        rospy.loginfo(f"Simulated movement for: {mode}")
    
    def publish_complete(self):
        msg = Bool()
        msg.data = True
        self.pub_complete.publish(msg)
        rospy.loginfo(f"Published movement complete for: {self.current_command}")
        self.current_command = None
    
    def spin(self):
        # Publish initial complete to signal ready
        self.publish_complete()
        rospy.spin()

if __name__ == '__main__':
    try:
        node = MovementNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        pass