#!/usr/bin/env python3

# This function implements the finite state machine for the blackjack game.
# It manages the game flow, including dealing cards, handling commanding movements to the movement node,
# and processing results from the vision node.

import rospy
from std_msgs.msg import Bool, UInt8, String
from enum import Enum

class GameState(Enum):
    INIT = 0
    START_GAME = 1
    DEAL_HOUSE_INITIAL = 2
    CAMERA_DOWN_INITIAL = 3
    DEALING_PLAYERS_INITIAL = 4
    PLAYER_TURN = 5
    HOUSE_TURN = 6
    GAME_END = 7

class BlackjackFSM:
    def __init__(self):
        rospy.init_node('blackjack_fsm', anonymous=True)
        
        # Game parameters
        self.num_players = rospy.get_param('~num_players', 1)
        self.current_player = 1  # Start with player 1
        self.game_state = GameState.INIT
        self.camera_down = False
        self.movement_complete = False
        self.last_card = None
        self.last_rps = None
        self.vision_ready = False  # TBD, assume ready for now
        
        # Player data: index 0 is house
        self.player_cards = [[] for _ in range(self.num_players + 1)]
        self.player_values = [0] * (self.num_players + 1)
        
        # Publishers
        self.movement_control = rospy.Publisher('/movement/control', String, queue_size=10)
        self.vision_control = rospy.Publisher('/vision/control', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/vision/card_result', String, self.card_callback)
        rospy.Subscriber('/vision/rps_result', UInt8, self.rps_callback)
        rospy.Subscriber('/movement/complete', Bool, self.movement_callback)
        
        # Flags for sequencing
        self.turn_done = False
        self.deal1_done = False
        self.deal2_done = False
        self.card_checked = False
        self.house_dealing = False
    
    def card_callback(self, msg: String):
        self.last_card = msg.data
        self.vision_ready = True  # Assume ready on first message
    
    def rps_callback(self, msg: UInt8):
        # Map result to action: 1=R(hit), 2=P(stay), 3=S(stay)
        actions = {1: 'hit', 2: 'stay', 3: 'stay'}
        self.last_rps = actions.get(msg.data, 'stay')
        self.vision_ready = True
    
    def movement_callback(self, msg: Bool):
        self.movement_complete = msg.data
    
    def calculate_hand_value(self, cards):
        value = 0
        aces = 0
        for card in cards:
            rank = card[:-1]  # Remove suit
            if rank == 'A':
                aces += 1
                value += 11
            elif rank in ['J', 'Q', 'K']:
                value += 10
            else:
                value += int(rank)
        while value > 21 and aces:
            value -= 10
            aces -= 1
        return value
    
    def publish_movement(self, mode):
        msg = String()
        msg.data = mode
        self.movement_control.publish(msg)
        self.movement_complete = False  # Reset flag
    
    def publish_vision(self, mode):
        msg = String()
        msg.data = mode
        self.vision_control.publish(msg)
    
    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            if self.game_state == GameState.INIT:
                # Wait for nodes ready: assume ready when first messages received
                if self.vision_ready and self.movement_complete:
                    self.game_state = GameState.START_GAME
            
            elif self.game_state == GameState.START_GAME:
                # Deal house card
                self.publish_movement("DEAL 0")
                self.game_state = GameState.DEAL_HOUSE_INITIAL
            
            elif self.game_state == GameState.DEAL_HOUSE_INITIAL:
                if self.movement_complete:
                    # Camera down
                    self.publish_movement("CAMERA_DOWN")
                    self.camera_down = True
                    self.game_state = GameState.CAMERA_DOWN_INITIAL
            
            elif self.game_state == GameState.CAMERA_DOWN_INITIAL:
                if self.movement_complete:
                    # Start dealing to players
                    self.current_player = 1
                    self.reset_dealing_flags()
                    self.game_state = GameState.DEALING_PLAYERS_INITIAL
            
            elif self.game_state == GameState.DEALING_PLAYERS_INITIAL:
                if self.current_player <= self.num_players:
                    if not self.turn_done:
                        self.publish_movement(f"TURN {self.current_player}")
                        self.turn_done = True
                    elif self.movement_complete and not self.deal1_done:
                        self.publish_movement(f"DEAL {self.current_player}")
                        self.deal1_done = True
                        self.movement_complete = False
                    elif self.movement_complete and not self.deal2_done:
                        self.publish_movement(f"DEAL {self.current_player}")
                        self.deal2_done = True
                        self.movement_complete = False
                    elif self.movement_complete and not self.card_checked:
                        self.publish_vision("CARDS_START")
                        if self.last_card:
                            self.player_cards[self.current_player].append(self.last_card)
                            self.player_values[self.current_player] = self.calculate_hand_value(self.player_cards[self.current_player])
                            self.publish_vision("CARDS_STOP")
                            self.last_card = None
                            self.card_checked = True
                            self.reset_dealing_flags()
                            self.current_player += 1
                else:
                    # All players dealt
                    self.current_player = 1
                    self.game_state = GameState.PLAYER_TURN
                    self.reset_dealing_flags()
            
            elif self.game_state == GameState.PLAYER_TURN:
                if self.current_player <= self.num_players:
                    # Check if player busted
                    if self.player_values[self.current_player] > 21:
                        self.current_player += 1
                        continue
                    
                    if not self.turn_done:
                        self.publish_movement(f"TURN {self.current_player}")
                        self.turn_done = True
                    elif self.movement_complete and not self.deal1_done:
                        self.publish_movement(f"DEAL {self.current_player}")
                        self.deal1_done = True
                        self.movement_complete = False
                    elif self.movement_complete and not self.card_checked:
                        if not self.camera_down:
                            self.publish_movement("CAMERA_DOWN")
                            self.camera_down = True
                        else:
                            self.publish_vision("CARDS_START")
                            if self.last_card:
                                self.player_cards[self.current_player].append(self.last_card)
                                self.player_values[self.current_player] = self.calculate_hand_value(self.player_cards[self.current_player])
                                self.publish_vision("CARDS_STOP")
                                self.last_card = None
                                self.card_checked = True
                    elif self.movement_complete and self.card_checked and not self.deal2_done:  # After card check
                        if self.player_values[self.current_player] > 21:
                            self.reset_dealing_flags()
                            self.current_player += 1
                            continue
                        self.publish_movement("CAMERA_UP")
                        self.camera_down = False
                        self.deal2_done = True  # Reuse flag
                        self.movement_complete = False
                    elif self.movement_complete and self.deal2_done and not self.turn_done:  # Wait for camera up
                        self.publish_vision("RPS_START")
                        self.turn_done = False  # Reuse
                    elif self.last_rps:
                        if self.last_rps == 'hit':
                            # Reset for another deal
                            self.reset_dealing_flags()
                            # Stay in state
                        else:
                            self.publish_vision("RPS_STOP")
                            self.last_rps = None
                            self.reset_dealing_flags()
                            self.current_player += 1
                else:
                    # House turn
                    self.game_state = GameState.HOUSE_TURN
                    self.reset_dealing_flags()
            
            elif self.game_state == GameState.HOUSE_TURN:
                if not self.turn_done:
                    self.publish_movement("TURN 0")
                    self.turn_done = True
                elif self.movement_complete and not self.camera_down:
                    self.publish_movement("CAMERA_DOWN")
                    self.camera_down = True
                    self.movement_complete = False
                elif self.movement_complete and self.camera_down:
                    if self.player_values[0] < 17:
                        if not self.house_dealing:
                            self.publish_movement("DEAL 0")
                            self.house_dealing = True
                            self.movement_complete = False
                        elif self.movement_complete and self.house_dealing:
                            self.publish_vision("CARDS_START")
                            if self.last_card:
                                self.player_cards[0].append(self.last_card)
                                self.player_values[0] = self.calculate_hand_value(self.player_cards[0])
                                self.publish_vision("CARDS_STOP")
                                self.last_card = None
                                self.house_dealing = False
                    else:
                        self.publish_movement("CAMERA_UP")
                        self.camera_down = False
                        self.game_state = GameState.GAME_END
            
            elif self.game_state == GameState.GAME_END:
                # Determine winners
                house_value = self.player_values[0]
                winners = []
                tie = False
                for p in range(1, self.num_players + 1):
                    pv = self.player_values[p]
                    if pv <= 21 and (pv > house_value or house_value > 21):
                        winners.append(p)
                    elif pv == house_value and pv <= 21:
                        tie = True
                
                # Celebrate
                for w in winners:
                    self.publish_movement(f"CELEBRATE {w}")
                if tie and not winners:
                    self.publish_movement("TIE")
                # End game
                self.publish_movement("END_GAME")
                # Exit
                break
            
            rate.sleep()
    
    def reset_dealing_flags(self):
        self.turn_done = False
        self.deal1_done = False
        self.deal2_done = False
        self.card_checked = False
        self.house_dealing = False

if __name__ == '__main__':
    try:
        fsm = BlackjackFSM()
        fsm.run()
    except rospy.ROSInterruptException:
        pass