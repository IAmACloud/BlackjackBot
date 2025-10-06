#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header, String, Bool, Int32
from vision_pkg.msg import RpsResult, CardResult, VisionControl
from movement_pkg.msg import MovementControl, MovementComplete
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum

class GameState(Enum):
    INIT = 0
    START_GAME = 1
    DEALING_INITIAL = 2
    PLAYER_TURN = 3
    WAITING_ACTION = 4 # this may be unnecessary, just embed in the other states as a substate fcn
    DEALING_PLAYER = 5
    CHECK_CARDS = 6 # also may be unnecessary, just embed as substate fcns (1. wait for action complete, 2. wait for card results)
    HOUSE_TURN = 7
    GAME_END = 8

class BlackjackFSM:
    def __init__(self):
        rospy.init_node('blackjack_fsm', anonymous=True)
        
        # Game parameters
        self.num_players = rospy.get_param('~num_players', 1)
        self.current_player = 0
        self.current_state = GameState.INIT
        self.camera_ready = False
        self.movement_complete = False
        
        # Player data
        self.player_cards = [[] for _ in range(self.num_players + 1)]  # +1 for house
        self.player_values = [0] * (self.num_players + 1)
        
        # Publishers
        self.movement_control = rospy.Publisher('/movement/control', MovementControl, queue_size=10)
        self.vision_control = rospy.Publisher('/vision/control', VisionControl, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/vision/card_result', CardResult, self.card_callback)
        rospy.Subscriber('/vision/rps_result', RpsResult, self.action_callback)
        rospy.Subscriber('/movement/complete', MovementComplete, self.movement_callback)
        
        # Initialize game values
        self.last_card = None
        self.last_action = None
    
    def card_callback(self, msg):
        self.last_card = msg.data
        
    def action_callback(self, msg):
        self.last_action = msg.data
        
    def movement_callback(self, msg):
        self.movement_complete = msg.data
        
    def camera_callback(self, msg):
        self.camera_ready = msg.data
    
    def calculate_hand_value(self, cards):
        # TODO: Implement card value calculation
        pass
    
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            if self.current_state == GameState.INIT:
                self.current_state = GameState.START_GAME
                
            elif self.current_state == GameState.START_GAME:
                msg = MovementControl()
                msg.mode = "CAMERA UP"
                self.movement_control.publish(msg)
                # TODO: make sure we don't keep sending message by using lockout
                # TODO: wait for movement complete

                if self.camera_ready:
                    msg = MovementControl()
                    msg.mode = "DEAL HOUSE"
                    self.movement_control.publish(msg)  # Deal house card
                    self.current_state = GameState.DEALING_INITIAL
                    
            elif self.current_state == GameState.DEALING_INITIAL:
                if self.movement_complete and self.current_player < self.num_players:

                    self.movement_control.publish(True)
                    self.current_player += 1
                elif self.current_player >= self.num_players:
                    self.current_player = 0
                    self.current_state = GameState.PLAYER_TURN
                    
            elif self.current_state == GameState.PLAYER_TURN:
                # TODO: insert wait fcn here (maybe?)
                if self.movement_complete:
                    msg = MovementControl()
                    msg.mode = "DEAL " + str(self.current_player)
                    self.movement_control.publish(msg)
                    # TODO: insert wait fcn here
                    self.vision_control.publish("RPS_START")
                    # TODO: insert wait for vision fcn here
                    self.current_state = GameState.WAITING_ACTION # TODO: remove once sub-function

            # TODO: wait for movement complete, this should be a sub-action that just checks subscription       
            elif self.current_state == GameState.WAITING_ACTION:
                if self.last_action == "hit":
                    self.current_state = GameState.DEALING_PLAYER
                    self.deal_pub.publish(True)
                elif self.last_action == "stand":
                    self.current_player += 1
                    if self.current_player >= self.num_players:
                        self.current_state = GameState.HOUSE_TURN
                    else:
                        self.current_state = GameState.PLAYER_TURN
                        
            # TODO: Add remaining states here
            
            rate.sleep()

if __name__ == '__main__':
    try:
        fsm = BlackjackFSM()
        fsm.run()
    except rospy.ROSInterruptException:
        pass