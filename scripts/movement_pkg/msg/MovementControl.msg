std_msgs/Header header
string mode # e.g. "DEAL <PLAYER NUMBER>","TURN <PLAYER NUMBER>","CELEBRATE <PLAYER NUMBER>", "CAMERA <UP/DOWN>, END_GAME"
# player number is 0, 1, or 2 in our setup. House should be 0?
# END_GAME is for setting all motors back to relaxed position.