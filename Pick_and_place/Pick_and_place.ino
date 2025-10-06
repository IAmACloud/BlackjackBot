/*
  Hubert robot test program.

  This sketch control Hubert's servos.
  A write.microseconds value in approx. [500,2350], mid pos.: 1350
  NOTE: Go back to NeutralPos. before terminating.

  created 20 Jul 2020
  by Krister Wolff
  modified 11 Sep 2025
  by Vivien Lacorre

  This example code is in the public domain.

  http://www.arduino.cc/en/
*/

#include <Arduino.h>
#include <Servo.h>

//Servos
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

//Init position of all servos
const int servo_pins[SERVO_COUNT] = {3, 5, 6, 9, 10, 11, 8};

int curr_pos[SERVO_COUNT];

const int pos_init[SERVO_COUNT] = {1700, 1500, 2000, 2300, 900, 1000, 450};
const int pos_dodge[SERVO_COUNT] = {900, 2000, 2000, 2300, 2100, 1000, 450};
const int pos_hover[SERVO_COUNT] = {2100, 2000, 2000, 2300, 2100, 1000, 450};
const int pos_pick[SERVO_COUNT] = {2100, 1700, 2000, 2300, 1750, 1000, 450};
const int pos_place[SERVO_COUNT] = {1700, 2000, 2000, 2300, 1500, 1500, 1850};

int count = 0;

void actuate_servo(Servo &servo, int servo_index, const int new_pos) {
  int diff, steps, now, CurrPwm, NewPwm, delta = 6;

  // current servo value
  now = curr_pos[servo_index];
  CurrPwm = now;
  NewPwm = new_pos;

  if (CurrPwm == NewPwm) return; // No movement needed

  // determine direction (+1 or -1)
  diff = (NewPwm - CurrPwm) / abs(NewPwm - CurrPwm);
  steps = abs(NewPwm - CurrPwm);
  delay(10);

  for (int i = 0; i < steps; i += delta) {
    now = now + delta * diff;
    servo.writeMicroseconds(now);
    delay(20);
  }
  curr_pos[servo_index] = now;
  delay(10);
}

// Move to dodge pos
void actuate_dodge() {
  actuate_servo(headPan, HEAD_PAN, pos_dodge[HEAD_PAN]);
  actuate_servo(body, BODY, pos_dodge[BODY]);
  delay(500);
  actuate_servo(elbow, ELBOW, pos_dodge[ELBOW]);
} 

// Move to hover pos
void actuate_hover() {
  actuate_servo(headPan, HEAD_PAN, pos_hover[HEAD_PAN]);
  actuate_servo(wrist, WRIST, pos_hover[WRIST]);
  delay(500);
  actuate_servo(gripper, GRIPPER, pos_hover[GRIPPER]);
  delay(500);
  actuate_servo(elbow, ELBOW, pos_hover[ELBOW]);
  delay(500);
  actuate_servo(body, BODY, pos_hover[BODY]);
} 

// Move to pick pos
void actuate_pick() {
  actuate_servo(headPan, HEAD_PAN, pos_pick[HEAD_PAN]);
  actuate_servo(elbow, ELBOW, pos_pick[ELBOW]);
  delay(500);
} 

// Move to place pos
void actuate_place() {
  actuate_servo(body, BODY, pos_place[BODY]);
  actuate_servo(headPan, HEAD_PAN, pos_place[HEAD_PAN]);
  delay(500);
  actuate_servo(wrist, WRIST, pos_place[WRIST]);
  delay(500);
  actuate_servo(gripper, GRIPPER, pos_place[GRIPPER]);
  delay(500);
  actuate_servo(elbow, ELBOW, pos_place[ELBOW]);
  delay(500);
} 

void setup() {
  Serial.begin(57600); // Starts the serial communication

	//Attach each joint servo
	//and write each init position
  body.attach(servo_pins[0]);
  body.writeMicroseconds(pos_init[0]);

  headPan.attach(servo_pins[1]);
  headPan.writeMicroseconds(pos_init[1]);

  headTilt.attach(servo_pins[2]);
  headTilt.writeMicroseconds(pos_init[2]);

  shoulder.attach(servo_pins[3]);
	shoulder.writeMicroseconds(pos_init[3]);

	elbow.attach(servo_pins[4]);
	elbow.writeMicroseconds(pos_init[4]);

	gripper.attach(servo_pins[5]);
  gripper.writeMicroseconds(pos_init[5]);

  wrist.attach(servo_pins[6]);
  wrist.writeMicroseconds(pos_init[6]);

  Serial.println("Hubert test program started.");

  // We keep track of the current poses in the curr_pos array
  byte i;
  for (i=0; i<(sizeof(pos_init)/sizeof(int)); i++){
    curr_pos[i] = pos_init[i];
  }

	delay(2000);
}

void loop() {

  if(count == 0) {
    Serial.println("Moving Hubert to dodge position...");
    actuate_dodge();
    count++;
  } 
  actuate_hover();
  actuate_pick();
  actuate_hover();
  actuate_place();
}
