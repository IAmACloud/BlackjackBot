#!/usr/bin/env python3
"""
This function implements the finite state machine for the blackjack game.
It manages the game flow, including dealing cards, handling commanding movements to the movement node,
and processing results from the vision node.
"""

import rospy
from std_msgs.msg import Bool, UInt8, String
from beginner_tutorial.msg import RpsResult, CardResult, VisionControl
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
        self.seen_cards = set()
        self.waiting_for_cards = False
        self.card_wait_start = None
        self.pending_cards = set()  # unique cards
        self.card_check_pending = False
        self.waiting_for_rps = False
        self.rps_wait_start = None
        self.pending_rps = {1: 0.0, 2: 0.0, 3: 0.0}  # scores for R, P, S
        
        # Player data: index 0 is house
        self.player_cards = [[] for _ in range(self.num_players + 1)]
        self.player_values = [0] * (self.num_players + 1)
        
        # Publishers
        self.movement_control = rospy.Publisher('/movement/control', String, queue_size=10)
        self.vision_control = rospy.Publisher('/vision/control', VisionControl, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/vision/card_result', CardResult, self.card_callback)
        rospy.Subscriber('/vision/rps_result', RpsResult, self.rps_callback)
        rospy.Subscriber('/movement/complete', Bool, self.movement_callback)
        rospy.Subscriber('/vision/ready', Bool, self.vision_callback)
        
        # Flags for sequencing (per player, index 0 for house)
        self.turn_done = [False] * (self.num_players + 1)
        self.deal1_done = [False] * (self.num_players + 1)
        self.deal2_done = [False] * (self.num_players + 1)
        self.card_checked = [False] * (self.num_players + 1)
        self.house_dealing = [False] * (self.num_players + 1)
        self.card_check_pending = [False] * (self.num_players + 1)
        self.camera_down_for_check = [False] * (self.num_players + 1)
        self.cards_verified = [False] * (self.num_players + 1)
        
        rospy.loginfo(f"[blackjack_fsm] Initialized with num_players: {self.num_players}")

    # --- Vision pub/sub ---
    
    def card_callback(self, msg: CardResult):
        card_str = msg.card
        parts = card_str.split()
        if parts and parts[0].isdigit():
            num = int(parts[0])
            card = parts[1]
            if num > 1 and card.endswith('s'):
                card = card[:-1]
        else:
            card = card_str
        if self.waiting_for_cards:
            if card not in self.seen_cards:
                self.pending_cards.add(card)
    
    def rps_callback(self, msg: RpsResult):
        if self.waiting_for_rps:
            self.pending_rps[msg.result] += msg.confidence
        #else:
            # Fallback: map result to action if not waiting
            #actions = {1: 'hit', 2: 'stay', 3: 'stay'}
            #self.last_rps = actions.get(msg.result, 'stay')

    def vision_callback(self, msg: Bool):
        self.vision_ready = msg.data
        rospy.loginfo(f"[blackjack_fsm] Vision ready status: {self.vision_ready}")

    # --- Movement pub/sub ---
    
    def movement_callback(self, msg: Bool):
        self.movement_complete = msg.data
        rospy.loginfo(f"[blackjack_fsm] Movement complete status: {self.movement_complete}")

    def publish_movement(self, mode):
        msg = String()
        msg.data = mode
        self.movement_control.publish(msg)
        self.movement_complete = False  # Reset flag
        rospy.loginfo(f"[blackjack_fsm] Published movement control: {mode}")
    
    def publish_vision(self, mode):
        msg = VisionControl()
        msg.header.stamp = rospy.Time.now()
        msg.mode = mode
        self.vision_control.publish(msg)
        if mode == "CARDS_START":
            self.waiting_for_cards = True
            self.card_wait_start = rospy.Time.now()
            self.pending_cards = set()
        elif mode == "CARDS_STOP":
            self.waiting_for_cards = False
            self.card_wait_start = None
            self.pending_cards = set()
        elif mode == "RPS_START":
            self.waiting_for_rps = True
            self.rps_wait_start = rospy.Time.now()
            self.pending_rps = {1: 0.0, 2: 0.0, 3: 0.0}
        elif mode == "RPS_STOP":
            self.waiting_for_rps = False
            self.rps_wait_start = None
            self.pending_rps = {}
        rospy.loginfo(f"[blackjack_fsm] Published vision control: {mode}")

    #--- Game Logic Helpers ---

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

    def reset_dealing_flags(self, player):
        self.turn_done[player] = False
        self.deal1_done[player] = False
        self.deal2_done[player] = False
        self.card_checked[player] = False
        self.house_dealing[player] = False
        self.card_check_pending[player] = False
        self.camera_down_for_check[player] = False
        self.cards_verified[player] = False
        rospy.loginfo(f"[blackjack_fsm] Reset dealing flags for player {player}")

    # --- Game State Management ---------------------------------------------------------------------------------------------------
    
    def run(self):
        """
        Main game loop that orchestrates the blackjack game finite state machine.
        This method runs continuously at 10Hz until ROS shutdown, managing all game states
        from initialization through game completion. It handles timeouts for card and RPS
        (Rock-Paper-Scissors) detection, coordinates movement and vision systems, and 
        implements the complete blackjack game flow.
        Game States Managed:
        - INIT: Wait for vision and movement nodes to be ready
        - START_GAME: Reset game data and begin dealing
        - DEAL_HOUSE_INITIAL: Deal initial card to house/dealer
        - CAMERA_DOWN_INITIAL: Lower camera to view cards
        - DEALING_PLAYERS_INITIAL: Deal first card to each player
        - PLAYER_TURN: Handle each player's turn (hit/stay decisions via RPS)
        - HOUSE_TURN: House plays according to standard rules (hit until 17+)
        - GAME_END: Determine winners and celebrate
        Timeout Handling:
        - Card collection: 5 second timeout for vision to detect cards
        - RPS collection: 5 second timeout for gesture recognition
        The method coordinates between movement commands (dealing, camera positioning)
        and vision commands (card detection, RPS gesture recognition) while maintaining
        game state and player hand values.
        Returns:
            None: Method runs until game completion or ROS shutdown
        """
        # Set the main loop rate to 10Hz for responsive state management
        rate = rospy.Rate(10)  # 10Hz
        rospy.loginfo(f"[blackjack_fsm] {self.game_state}")
        
        while not rospy.is_shutdown():
            # Check for card collection timeout
            if self.waiting_for_cards and self.card_wait_start and (rospy.Time.now() - self.card_wait_start).to_sec() > 5.0:
                if self.pending_cards:
                    for card in self.pending_cards:
                        self.seen_cards.add(card)
                        self.player_cards[self.current_player].append(card)
                    self.last_card = self.player_cards[self.current_player][-1] if self.player_cards[self.current_player] else None
                self.pending_cards = set()
                self.publish_vision("CARDS_STOP")  # Stop vision after timeout and appending
                rospy.loginfo(f"[blackjack_fsm] Card collection timeout for player {self.current_player}. Hand: {self.player_cards[self.current_player]}")
            
            # Check for RPS collection timeout
            if self.waiting_for_rps and self.rps_wait_start and (rospy.Time.now() - self.rps_wait_start).to_sec() > 5.0:
                if self.pending_rps:
                    max_key = max(self.pending_rps, key=self.pending_rps.get)
                    actions = {1: 'hit', 2: 'stay', 3: 'stay'}
                    # select action based on highest confidence and number of seen frames, defaults to stay
                    self.last_rps = actions.get(max_key, 'stay') 
                else:
                    self.last_rps = 'stay'  # default if no messages
                self.publish_vision("RPS_STOP")
                rospy.loginfo(f"[blackjack_fsm] RPS collection timeout for player {self.current_player}. Selected action: {self.last_rps}")
            
            # State: INIT - Wait for vision and movement nodes to be ready before starting the game
            if self.game_state == GameState.INIT:
                # Check if vision has sent messages and movement is complete
                if self.vision_ready and self.movement_complete:
                    # Nodes are ready, transition to start game
                    self.game_state = GameState.START_GAME
                    rospy.loginfo("[blackjack_fsm] =====STATE CHANGED TO: START_GAME")
            
            # State: START_GAME - Reset game data and deal the initial house card
            elif self.game_state == GameState.START_GAME:
                # Reset seen cards for a new game
                self.seen_cards = set()
                # Command to deal the house's initial card
                self.publish_movement("DEAL 0")
                self.game_state = GameState.DEAL_HOUSE_INITIAL
                rospy.loginfo("[blackjack_fsm] =====STATE CHANGED TO: DEAL_HOUSE_INITIAL")
            
            # State: DEAL_HOUSE_INITIAL - Wait for house card to be dealt, then lower camera
            elif self.game_state == GameState.DEAL_HOUSE_INITIAL:
                if self.movement_complete:
                    # Movement complete, lower camera to check the card
                    self.publish_movement("CAMERA_DOWN")
                    self.camera_down = True
                    self.game_state = GameState.CAMERA_DOWN_INITIAL
                    rospy.loginfo("[blackjack_fsm] =====STATE CHANGED TO: CAMERA_DOWN_INITIAL")
            
            # State: CAMERA_DOWN_INITIAL - Wait for camera to lower, then start dealing to players
            elif self.game_state == GameState.CAMERA_DOWN_INITIAL:
                if self.movement_complete:
                    # Camera down, initialize player dealing
                    self.current_player = 1
                    self.reset_dealing_flags(1)
                    self.game_state = GameState.DEALING_PLAYERS_INITIAL
                    rospy.loginfo("[blackjack_fsm] =====STATE CHANGED TO: DEALING_PLAYERS_INITIAL")
            
            # State: DEALING_PLAYERS_INITIAL - Deal one card to each player initially
            elif self.game_state == GameState.DEALING_PLAYERS_INITIAL:
                if self.current_player <= self.num_players:
                    # Turn to current player if not done
                    if not self.turn_done[self.current_player]:
                        self.publish_movement(f"TURN {self.current_player}")
                        self.turn_done[self.current_player] = True
                    # Deal first card if movement complete and not dealt
                    elif self.movement_complete and not self.deal1_done[self.current_player]:
                        self.publish_movement(f"DEAL {self.current_player}")
                        self.deal1_done[self.current_player] = True
                        self.movement_complete = False
                    # Check card if deal done and not checked
                    elif self.movement_complete and not self.card_checked[self.current_player]:
                        if not self.card_check_pending[self.current_player]:
                            self.publish_vision("CARDS_START")
                            self.card_check_pending[self.current_player] = True
                        elif self.card_check_pending[self.current_player] and not self.waiting_for_cards:
                            # Append collected cards after vision stop
                            if self.pending_cards:
                                for card in self.pending_cards:
                                    if card not in self.seen_cards:
                                        self.seen_cards.add(card)
                                        self.player_cards[self.current_player].append(card)
                                self.player_values[self.current_player] = self.calculate_hand_value(self.player_cards[self.current_player])
                            self.pending_cards = set()
                            self.card_checked[self.current_player] = True
                            self.reset_dealing_flags(self.current_player)
                            self.current_player += 1
                else:
                    # All players dealt initial card, move to player turns
                    self.current_player = 1
                    self.game_state = GameState.PLAYER_TURN
                    self.reset_dealing_flags(1)
                    rospy.loginfo("[blackjack_fsm] =====STATE CHANGED TO: PLAYER_TURN")
            
            # State: PLAYER_TURN - Handle each player's turn: check existing cards, deal second card, check RPS for hit/stay
            elif self.game_state == GameState.PLAYER_TURN:
                if self.current_player <= self.num_players:
                    # Skip if player busted
                    if self.player_values[self.current_player] > 21:
                        rospy.loginfo(f"[blackjack_fsm] Player {self.current_player} busted with value {self.player_values[self.current_player]}")
                        self.reset_dealing_flags(self.current_player)
                        self.current_player += 1
                        continue
                    
                    # Turn to player if not done
                    if not self.turn_done[self.current_player]:
                        self.publish_movement(f"TURN {self.current_player}")
                        self.turn_done[self.current_player] = True
                    # After turning, lower camera to check existing cards
                    elif self.movement_complete and not self.camera_down_for_check[self.current_player]:
                        self.publish_movement("CAMERA_DOWN")
                        self.camera_down_for_check[self.current_player] = True
                        self.camera_down = True
                        self.movement_complete = False
                    # Start vision to check cards
                    #elif self.movement_complete and self.camera_down_for_check[self.current_player] and not self.cards_verified[self.current_player]:
                    #    self.publish_vision("CARDS_START")
                    #    self.cards_verified[self.current_player] = True
                    # Deal second card if cards verified and not dealt
                    elif self.movement_complete and not self.deal1_done[self.current_player]:
                        self.publish_movement(f"DEAL {self.current_player}")
                        self.deal1_done[self.current_player] = True
                        self.movement_complete = False
                    # Check card if deal done and not checked
                    elif self.movement_complete and not self.card_checked[self.current_player] and self.deal1_done[self.current_player]:
                        if not self.card_check_pending[self.current_player]:
                            self.publish_vision("CARDS_START")
                            self.card_check_pending[self.current_player] = True
                        elif self.card_check_pending[self.current_player] and not self.waiting_for_cards:
                            # Assign card
                            if self.pending_cards:
                                for card in self.pending_cards:
                                    if card not in self.seen_cards:
                                        self.seen_cards.add(card)
                                        self.player_cards[self.current_player].append(card)
                                self.player_values[self.current_player] = self.calculate_hand_value(self.player_cards[self.current_player])
                            self.pending_cards = set()
                            self.card_checked[self.current_player] = True
                    # After card check, check if busted or proceed
                    elif self.movement_complete and self.card_checked[self.current_player] and not self.deal2_done[self.current_player]:  # After card check
                        if self.player_values[self.current_player] > 21:
                            self.reset_dealing_flags(self.current_player)
                            self.current_player += 1
                            continue
                        self.publish_movement("CAMERA_UP")
                        self.camera_down = False
                        self.deal2_done[self.current_player] = True
                        self.movement_complete = False
                    # Wait for camera up, then start RPS
                    elif self.movement_complete and self.deal2_done[self.current_player]:
                        self.publish_vision("RPS_START")
                    # Process RPS result
                    elif self.last_rps and not self.waiting_for_rps:
                        if self.last_rps == 'hit':
                            # Reset for another deal
                            self.reset_dealing_flags(self.current_player)
                            # Stay in state
                        else: # Stay (no more cards for this player)
                            self.last_rps = None
                            self.current_player += 1
                else:
                    # All players done, house turn
                    self.game_state = GameState.HOUSE_TURN
                    self.reset_dealing_flags(0)
                    rospy.loginfo("[blackjack_fsm] =====STATE CHANGED TO: HOUSE TURN")
            
            # State: HOUSE_TURN - House plays: deal cards until 17 or higher
            elif self.game_state == GameState.HOUSE_TURN:
                self.current_player = 0  # House
                # Turn to house if not done
                if not self.turn_done[0]:
                    self.publish_movement("TURN 0")
                    self.turn_done[0] = True
                # Lower camera if not done
                elif self.movement_complete and not self.camera_down:
                    self.publish_movement("CAMERA_DOWN")
                    self.camera_down = True
                    self.movement_complete = False
                # Check existing house cards if not verified
                elif self.movement_complete and self.camera_down and not self.cards_verified[0]:
                    if not self.card_check_pending[0]:
                        self.publish_vision("CARDS_START")
                        self.card_check_pending[0] = True
                    elif self.card_check_pending[0] and not self.waiting_for_cards:
                        # Assign existing house cards
                        if self.pending_cards:
                            for card in self.pending_cards:
                                if card not in self.seen_cards:
                                    self.seen_cards.add(card)
                                    self.player_cards[0].append(card)
                            self.player_values[0] = self.calculate_hand_value(self.player_cards[0])
                        self.pending_cards = set()
                        self.cards_verified[0] = True
                        self.card_check_pending[0] = False
                # House dealing logic
                elif self.movement_complete and self.camera_down and self.cards_verified[0]:
                    if self.player_values[0] < 17:
                        if not self.house_dealing[0]:
                            self.publish_movement("DEAL 0")
                            self.house_dealing[0] = True
                            self.movement_complete = False
                        elif self.movement_complete and self.house_dealing[0]:
                            if not self.card_check_pending[0]:
                                self.publish_vision("CARDS_START")
                                self.card_check_pending[0] = True
                            elif self.card_check_pending[0] and not self.waiting_for_cards:
                                # Assign card
                                if self.pending_cards:
                                    for card in self.pending_cards:
                                        if card not in self.seen_cards:
                                            self.seen_cards.add(card)
                                            self.player_cards[0].append(card)
                                    self.player_values[0] = self.calculate_hand_value(self.player_cards[0])
                                self.pending_cards = set()
                                self.house_dealing[0] = False
                                self.card_check_pending[0] = False
                    else:
                        # House stands, raise camera and end game
                        self.publish_movement("CAMERA_UP")
                        self.camera_down = False
                        self.game_state = GameState.GAME_END
                        rospy.loginfo("[blackjack_fsm] =====STATE CHANGED TO: GAME_END")
            
            # State: GAME_END - Determine winners and celebrate
            elif self.game_state == GameState.GAME_END:
                # Calculate house value
                house_value = self.player_values[0]
                winners = []
                tie = False
                for p in range(1, self.num_players + 1):
                    pv = self.player_values[p]
                    if pv <= 21 and (pv > house_value or house_value > 21):
                        winners.append(p)
                    elif pv == house_value and pv <= 21:
                        tie = True
                
                # Celebrate winners
                for w in winners:
                    self.publish_movement(f"CELEBRATE {w}")
                if tie and not winners:
                    self.publish_movement("TIE")
                # End game
                self.publish_movement("END_GAME")
                # Exit
                break
            
            rate.sleep()

if __name__ == '__main__':
    try:
        fsm = BlackjackFSM()
        fsm.run()
    except rospy.ROSInterruptException:
        pass