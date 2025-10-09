/*
  Blackjack Robot Movement Control

  This sketch controls the robotic arm for blackjack game.
  Receives commands from ROS movement node via serial.
  Commands: TURN <player>, DEAL <player>, CAMERA_UP/DOWN, CELEBRATE <player>, TIE, END_GAME

  created based on Hubert robot
  modified for blackjack game
*/

#include <Arduino.h>
#include <Servo.h>

// Servos
Servo body;
Servo headPan;
Servo headTilt;
Servo shoulder;
Servo elbow;
Servo gripper;
Servo wrist;

// Enum for servo indices
enum ServoIndex {
  BODY = 0,
  HEAD_PAN = 1,
  HEAD_TILT = 2,
  SHOULDER = 3,
  ELBOW = 4,
  GRIPPER = 5,
  WRIST = 6,
  SERVO_COUNT = 7
};

// Servo pins
const int servo_pins[SERVO_COUNT] = {3, 5, 6, 9, 10, 11, 8};

// Current positions
int curr_pos[SERVO_COUNT];

// Initial position (relaxed)
const int pos_init[SERVO_COUNT] = {1700, 1500, 2000, 2300, 900, 1000, 450};

// Player positions (body servo)
const int pos_house[SERVO_COUNT] = {1700, 1500, 2000, 2300, 900, 1000, 450}; // Same as init
const int pos_player1[SERVO_COUNT] = {2100, 1500, 2000, 2300, 900, 1000, 450}; // Right
const int pos_player2[SERVO_COUNT] = {1300, 1500, 2000, 2300, 900, 1000, 450}; // Left

// Camera positions (headTilt servo)
const int pos_camera_up[SERVO_COUNT] = {1700, 1500, 2000, 2300, 900, 1000, 450}; // Higher
const int pos_camera_down[SERVO_COUNT] = {1700, 1500, 1500, 2300, 900, 1000, 450}; // Lower

// Deck position for picking cards
const int pos_deck[SERVO_COUNT] = {1700, 1700, 2000, 2300, 1750, 1000, 450};

// Place positions for each player (adjust elbow/wrist for card placement)
const int pos_place_house[SERVO_COUNT] = {1700, 2000, 2000, 2300, 1500, 1500, 1850};
const int pos_place_player1[SERVO_COUNT] = {2100, 2000, 2000, 2300, 1500, 1500, 1850};
const int pos_place_player2[SERVO_COUNT] = {1300, 2000, 2000, 2300, 1500, 1500, 1850};

// Celebrate positions (wave with headPan)
const int pos_celebrate[SERVO_COUNT] = {1700, 1200, 2000, 2300, 900, 1000, 450}; // Left wave
const int pos_celebrate_back[SERVO_COUNT] = {1700, 1800, 2000, 2300, 900, 1000, 450}; // Right wave

// Command buffer
String command = "";

void actuate_servo(Servo &servo, int servo_index, const int new_pos) {
  int diff, steps, now, CurrPwm, NewPwm, delta = 6;

  now = curr_pos[servo_index];
  CurrPwm = now;
  NewPwm = new_pos;

  if (CurrPwm == NewPwm) return;

  diff = (NewPwm - CurrPwm) / abs(NewPwm - CurrPwm);
  steps = abs(NewPwm - CurrPwm);

  for (int i = 0; i < steps; i += delta) {
    now = now + delta * diff;
    servo.writeMicroseconds(now);
    delay(20);
  }
  curr_pos[servo_index] = now;
  delay(10);
}

void move_to_position(const int target_pos[SERVO_COUNT]) {
  actuate_servo(body, BODY, target_pos[BODY]);
  actuate_servo(headPan, HEAD_PAN, target_pos[HEAD_PAN]);
  actuate_servo(headTilt, HEAD_TILT, target_pos[HEAD_TILT]);
  actuate_servo(shoulder, SHOULDER, target_pos[SHOULDER]);
  actuate_servo(elbow, ELBOW, target_pos[ELBOW]);
  actuate_servo(gripper, GRIPPER, target_pos[GRIPPER]);
  actuate_servo(wrist, WRIST, target_pos[WRIST]);
}

void turn_to_player(int player) {
  if (player == 0) {
    move_to_position(pos_house);
  } else if (player == 1) {
    move_to_position(pos_player1);
  } else if (player == 2) {
    move_to_position(pos_player2);
  }
}

void camera_up() {
  actuate_servo(headTilt, HEAD_TILT, pos_camera_up[HEAD_TILT]);
}

void camera_down() {
  actuate_servo(headTilt, HEAD_TILT, pos_camera_down[HEAD_TILT]);
}

void deal_card(int player) {
  // Go to deck
  move_to_position(pos_deck);
  delay(500);

  // Close gripper to pick
  actuate_servo(gripper, GRIPPER, 1500); // Closed
  delay(500);

  // Lift slightly
  actuate_servo(elbow, ELBOW, 1900);
  delay(500);

  // Go to player position
  if (player == 0) {
    move_to_position(pos_place_house);
  } else if (player == 1) {
    move_to_position(pos_place_player1);
  } else if (player == 2) {
    move_to_position(pos_place_player2);
  }
  delay(500);

  // Open gripper to place
  actuate_servo(gripper, GRIPPER, 1000); // Open
  delay(500);

  // Back to hover
  actuate_servo(elbow, ELBOW, 2100);
  delay(500);
}

void celebrate_player(int player) {
  // Turn to player
  turn_to_player(player);
  delay(500);

  // Wave
  for (int i = 0; i < 3; i++) {
    move_to_position(pos_celebrate);
    delay(300);
    move_to_position(pos_celebrate_back);
    delay(300);
  }
}

void tie_celebrate() {
  // Celebrate each player
  celebrate_player(1);
  delay(1000);
  celebrate_player(2);
  delay(1000);
  celebrate_player(0);
}

void end_game() {
  move_to_position(pos_init);
}

void process_command(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd.startsWith("TURN ")) {
    int player = cmd.substring(5).toInt();
    turn_to_player(player);
    Serial.println("DONE");
  } else if (cmd == "CAMERA_UP") {
    camera_up();
    Serial.println("DONE");
  } else if (cmd == "CAMERA_DOWN") {
    camera_down();
    Serial.println("DONE");
  } else if (cmd.startsWith("DEAL ")) {
    int player = cmd.substring(5).toInt();
    deal_card(player);
    Serial.println("DONE");
  } else if (cmd.startsWith("CELEBRATE ")) {
    int player = cmd.substring(10).toInt();
    celebrate_player(player);
    Serial.println("DONE");
  } else if (cmd == "TIE") {
    tie_celebrate();
    Serial.println("DONE");
  } else if (cmd == "END_GAME") {
    end_game();
    Serial.println("DONE");
  } else {
    Serial.println("ERROR: Unknown command");
  }
}

void setup() {
  Serial.begin(9600); // Match movement node baud rate

  // Attach servos
  body.attach(servo_pins[BODY]);
  headPan.attach(servo_pins[HEAD_PAN]);
  headTilt.attach(servo_pins[HEAD_TILT]);
  shoulder.attach(servo_pins[SHOULDER]);
  elbow.attach(servo_pins[ELBOW]);
  gripper.attach(servo_pins[GRIPPER]);
  wrist.attach(servo_pins[WRIST]);

  // Set initial positions
  move_to_position(pos_init);

  Serial.println("Blackjack movement ready.");
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      process_command(command);
      command = "";
    } else {
      command += c;
    }
  }
}