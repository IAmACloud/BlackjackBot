# Program structure

https://officialgamerules.org/game-rules/blackjack/

## FSM for full program
- Has a subscriber for:
  - Card information
  - Action information
  - movement complete
  - vision ready (TBD if we need this)
- Has a publisher that publishes:
  - deal card
  - turn towards a given player number (player 0 = house, player 1 = 1, etc.)
  - look for rps
  - stop looking for rps
  - look for cards
  - stop looking for cards
  - move camera towards a given player number
  - Celebrate (towards a given player number) movement
  - Tie movement (which is telling movement to celebrate at each player once)
  - start game (for movement to do first deal of cards to players and self)
  - end game (for setting all motor positions back to start), may need to add additional message
- Parameters set in the file:
  - Location of players
  - Set number of players (probably only 1 or 2 players + house)
  - Fake movement input (command line input) (TBD if we need this)
  - Fake card input (command line input)  (TBD if we need this)
  - Fake gesture input (command line input) (TBD if we need this)
- General structure:
  - Wait for nodes ready - see that there has been a movement complete message and a vision message so that we know they are ready.
  - Start game
    - Deal House card (send "DEAL 0" to movement)
    - Wait for movement complete
    - Camera down (send to movement)
    - Wait for movement complete
    - Deal 2 cards per player (send "TURN player_number" to movement, wait for movement complete, send "DEAL player_number" to movement, wait for movement complete, send "DEAL player_number" to movement, wait for movement complete, check card by sending "CARDS_START", wait for CardResult message, then send "CARDS_STOP" to vision, repeat for each player)
  - Player turn
    - Confirm that the player_number is valid (if not, move to house turn)
    - Face player that is next (send "TURN player_number" to movement, wait for movement complete, send "DEAL player_number" to movement, wait for movement complete, send "CAMERA_DOWN" to movement, wait for movement complete, send "CARDS_START" to vision, wait for CardResult message, then send "CARDS_STOP" to vision, check if they are >21 in hand value, if yes then start over player turn with player_number + 1, if no then send "CAMERA_UP" to movement, wait for movement complete, send "RPS_START" to vision, wait for RpsResult message, then send "RPS_STOP" to vision, if RpsResult is hit, repeat dealing card sequence, if stay, move to next player and repeat or house turn if last player)
  - House turn
    - Face house (send "TURN 0" to movement, wait for movement complete, send "CAMERA_DOWN" to movement, wait for movement complete, calculate optimal move based on card values, if hit, send "DEAL 0" to movement, wait for movement complete, send "CARDS_START" to vision, wait for CardResult message, then send "CARDS_STOP" to vision, repeat until optimal move is stay, then send "CAMERA_UP" to movement, wait for movement complete)
  - End game
    - Deliberate winners (compare player card values to house card value, if player value > house value and <= 21, player wins, if player value < house value or > 21, house wins, if equal, tie)
    - Celebrate winners (send "CELEBRATE player_number" to movement for each winning player, wait for movement complete each time, if tie send "TIE" to movement, wait for movement complete)
    - Send "END_GAME" to movement to reset all motors to start position
    - Wait for movement complete
    - End program
- Game structure that we should keep track of:
  - Number of cards each player has (can be used for deciding where to place the card for movement and for calculating hand value)
  - value of cards each player has
  - Current game state
  - Current camera position (up or down, based on command we sent to movement last)


## Vision:
- Has a subscriber for:
  - look for rps (rock, paper, scissors)
  - stop looking for rps
  - look for cards
  - stop looking for cards
- Has a publisher that publishes:
  - Rock paper scissors seen message
    - char R, P, or S
  - Cards seen message
    - string of seen cards (ex. King of clubs = KC, 3 of hearts = 3H). Compilation of values does not happen in vision.
- Parameters set in the file:
  - rate of frame capture
  - number of frames to compile results from
  - rate to publish results
  - which camera
  - confidence for rps
  - confidence for cards
  - percent of compiled frames with result
  - some sort of bounding box parameters
- General structure:
  - At first run, activate camera and then keep active.
  - Send a message that vision is ready (TBD if we need this)
  - While true:
    - Check subscriptions for commands
    - set whether we should collect frames - if not, throw out the frames - but keep the camera connection live.
    - if collecting, run through desired model
    - compile results
    - send messages


## Movement
- Initialize
- Has a subscriber for a string message with modes:
  - turn to player x (0 = house, 1 = player 1, 2 = player 2)
  - Move Camera up/down
  - Deal Card to player x
  - Celebrate player x
  - Tie (celebrate each player once) (TBD if we need this, I can just send celebrate for each player in main)
  - End Game (move all motors to relaxed position)
  
- Has a publisher for
  - Movement Complete (used to tell main that movement is done for both camera and dealing/turning/celebrating and for main to know that movement node is ready at start)
- Parameters set in the file:
  - Motor pins
  - Motor positions for each action (turn to each player, camera up/down, deal card to each player, celebrate each player, end game position)
  - Motor speeds (TBD if we need this)
- Listen (while true)
- switch(action):
-   case "TURN":
-     turn_to_player(action.player)
-   case "CAMERA_UP":
-     move_camera("UP")
-   case "CAMERA_DOWN":
-     move_camera("DOWN")
-   case "DEAL":
-     deal_card(action.player)
-   case "CELEBRATE":
-     celebrate_player(action.player)
-   case "END_GAME":
-     end_game()
  
