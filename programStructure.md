# Program structure

https://officialgamerules.org/game-rules/blackjack/

## FSM for full program
- Has a subscriber for:
  - Card information
  - Action information
  - movement complete
  - camera movement complete
- Has a publisher that publishes:
  - deal card
  - turn x direction
  - look for rps
  - stop looking for rps
  - look for cards
  - stop looking for cards
  - move camera
  - Celebrate (myself, the players) movement
  - Tie movement
  - Point (this might just be turning)
  - start game (for movement to do first deal of cards to players and self)
  - end game (for setting all motor positions back to start)
- Parameters set in the file:
  - Location of players
  - Set number of players (probably only 1/2, 3 may be difficult)
  - Fake movement input (command line input)
  - Fake card input (command line input)
  - Fake gesture input (command line input)
  - Location of deck grab
- General structure:
  - Start game (this is done by movement, just wait for movement complete)
    - Camera up
    - Deal House card
    - Wait for deal
    - Deal one card per player
    - wait for deal
  - Wait for movement complete
  - Face player that is next
  - Camera up (movement)
  - Wait for camera movement complete
  - Look for action (vision)
  - Waiting for action from vision
  - Either deal or nothing. 
    - If deal, check cards (camera down), if first time for player, check cards. 
    - If not over, then return to camera up
  - If more players, return to face next player
  - Turn to house
  - Camera down
  - Wait for camera movement complete
  - Select house optimal move (deal at least once)
  - Check new card (vision)
  - Wait for new card information
  - Return to house optimal move
  - Deliberate winners
  - Celebrate winners somehow
- Game structure that we should keep track of:
  - Number of cards each player has (can be used for deciding where to place the cards)
  - value of cards
  - Deck load (so we know where to grab)
  - Current game state
  - Current camera position


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
  - Check subscriptions
  - set whether we should collect frames - if not, throw out the frames - but keep the camera connection live.
  - if collecting, run through desired model
  - compile results
  - send messages


## Movement
- Initialize
- Has a subscriber for
  - turn x direction
  - Move Camera up
  - Move Camera down
  - Deal Card
  - Celebrate
  - Deck position
  
- Has a publisher for
  - Movement Complete
  - Camera Up
  - Camera Down
    
- Publish a request for deck position
- Deal 2 cards to every player (including house, house gets 1 card though) [We are assuming card reads while dealing]
- Turn direction
- Publish Movement Complete
- Move Camera up
- Publish Camera up
- Listen (while true)
- switch(action):
  - Deal card -> Movement complete
  - Turn x direction -> Movement complete
  - Move Camera up -> Publish Camera up
  - Move Camera down -> Publish Camera down
  - Celebrate -> Movement comnplete
  - Return to start pos -> Movement complete
