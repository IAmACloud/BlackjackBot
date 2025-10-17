#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16.h>

//#include "MovementControl.h"
//#include "MovementComplete.h"

ros::NodeHandle hubert;

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

//Init position of all servos {7, 5, 6, 9, 10, 11, 8};
const int servo_pins[SERVO_COUNT] = {7, 2, 6, 9, 10, 11, 8};

int curr_pos[SERVO_COUNT];
const int pos_init[SERVO_COUNT] =  {1700, 1500, 1460, 2300, 900, 1000, 450};

// to be updated
const int bodyDeck = 1000;
const int bodyDodge = 1250;
const int bodyP1 = 1400;
const int bodyP2 = 1700;
const int bodyHouse = 2300;

const int headUp = 2000;
const int headDown = 1700;

const int shoulderCelebrate = 1500;

const int elbowRaised = 2100;
const int elbowDeck = 1750;
const int elbowPlace = 1400;

const int gripperOpen = 1000;
const int gripperClosed = 1500;

const int wristNeutral = 450;
const int wristPlace = 1850;

int count = 0;

std_msgs::Bool complete;
//Publisher
ros::Publisher movement_complete("movement_complete", &complete);
ros::Publisher camera_up("camera_up", &complete);
ros::Publisher camera_down("camera_down", &complete);

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
  hubert.spinOnce();
}

// Used as a startup-sequence
void actuate_dodge() {
  actuate_servo(headPan, HEAD_PAN, headUp);
  actuate_servo(body, BODY, bodyDodge);
  actuate_servo(elbow, ELBOW, elbowRaised);
  actuate_servo(gripper, GRIPPER, gripperClosed);
} 

// Help functions for atomic robot behaviour
void actuate_headUp() {
  actuate_servo(headPan, HEAD_PAN, headUp);
}

void actuate_headDown() {
  actuate_servo(headPan, HEAD_PAN, headDown);
}

void actuate_turnP1() {
  actuate_servo(body, BODY, bodyP1);
} 

void actuate_turnP2() {
  actuate_servo(body, BODY, bodyP2);
} 

void actuate_turnHouse() {
  actuate_servo(body, BODY, bodyHouse);
} 

// Pick card from deck
void actuate_deck() {
  actuate_servo(body, BODY, bodyDeck);
  actuate_servo(elbow, ELBOW, elbowDeck);
  actuate_servo(elbow, ELBOW, elbowRaised);
}

// Deal card to Player 1
void actuate_dealP1() {
  actuate_servo(body, BODY, bodyP1);
  actuate_servo(wrist, WRIST, wristPlace);
  actuate_servo(elbow, ELBOW, elbowPlace);
  actuate_servo(wrist, WRIST, wristNeutral);
  actuate_servo(gripper, GRIPPER, gripperOpen);
  actuate_servo(elbow, ELBOW, elbowRaised);
  actuate_servo(gripper, GRIPPER, gripperClosed);
  actuate_servo(headPan, HEAD_PAN, headDown);
} 

// Deal card to Player 2
void actuate_dealP2() {
  actuate_servo(body, BODY, bodyP2);
  actuate_servo(wrist, WRIST, wristPlace);
  actuate_servo(elbow, ELBOW, elbowPlace);
  actuate_servo(wrist, WRIST, wristNeutral);
  actuate_servo(gripper, GRIPPER, gripperOpen);
  actuate_servo(elbow, ELBOW, elbowRaised);
  actuate_servo(gripper, GRIPPER, gripperClosed);
  actuate_servo(headPan, HEAD_PAN, headDown);
} 

// Deal card to House
void actuate_dealHouse() {
  actuate_servo(body, BODY, bodyHouse);
  actuate_servo(wrist, WRIST, wristPlace);
  actuate_servo(elbow, ELBOW, elbowPlace);
  actuate_servo(wrist, WRIST, wristNeutral);
  actuate_servo(gripper, GRIPPER, gripperOpen);
  actuate_servo(elbow, ELBOW, elbowRaised);
  actuate_servo(gripper, GRIPPER, gripperClosed);
  actuate_servo(headPan, HEAD_PAN, headDown);
} 

// Celebrate Player 1
void actuate_celebrateP1() {
  actuate_servo(body, BODY, bodyP1);
  actuate_servo(wrist, WRIST, wristPlace);
  actuate_servo(wrist, WRIST, wristNeutral);
  actuate_servo(wrist, WRIST, wristPlace);
  actuate_servo(wrist, WRIST, wristNeutral);
}

// Celebrate Player 2
void actuate_celebrateP2() {
  actuate_servo(body, BODY, bodyP2);
  actuate_servo(wrist, WRIST, wristPlace);
  actuate_servo(wrist, WRIST, wristNeutral);
  actuate_servo(wrist, WRIST, wristPlace);
  actuate_servo(wrist, WRIST, wristNeutral);
}

// Celebrate House
void actuate_celebrateHouse() {
  actuate_servo(body, BODY, bodyHouse);
  actuate_servo(shoulder, SHOULDER, shoulderCelebrate);
  actuate_servo(elbow, ELBOW, elbowDeck);
  actuate_servo(elbow, ELBOW, elbowRaised);
  actuate_servo(elbow, ELBOW, elbowDeck);
  actuate_servo(elbow, ELBOW, elbowRaised);
  actuate_servo(shoulder, SHOULDER, pos_init[SHOULDER]);
}

//Functions for the messages we subscribe for:
void start_sequence(const std_msgs::String& start_msg) {
  const String& msg = start_msg.data;
  if(msg == "START GAME"){
    //Raise arm in safe way
    actuate_dodge();
    complete.data = true;
    //complete.header = ros::Time::now();
    movement_complete.publish(&complete);
  }
  else {
    // bad string
    complete.data = false;
    //complete.header = ros::Time::now();
    movement_complete.publish(&complete);
  }
}

void deal_card(const std_msgs::String& deal_msg){
  const String msg = deal_msg.data;
  //get player (digit)
  char last_char = msg.charAt(msg.length() - 1);
  if (last_char < '0' || last_char > '9') return;
  int player = last_char - '0';
  switch(player){
    case 0:
      //deal to house
      actuate_deck();
      actuate_dealHouse();
      complete.data = true;
      //complete.header.stamp = ros::Time::now();
      movement_complete.publish(&complete);
      break;
    
    case 1:
      actuate_deck();
      actuate_dealP1();
      complete.data = true;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;

    case 2:
      actuate_deck();
      actuate_dealP2();
      complete.data = true;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;
    
    default:
      //bad string
      complete.data = false;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;
  }
}

void turn_direction(const std_msgs::String& turn_msg){
  const String& msg = turn_msg.data;
  //get player (digit)
  char last_char = msg.charAt(msg.length() - 1);
  if (last_char < '0' || last_char > '9') return;
  int player = last_char - '0';
  switch(player){
    case 0:
      //turn to House
      actuate_turnHouse();
      complete.data = true;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;
    case 1:
      //turn to p1
      actuate_turnP1();
      complete.data = true;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;
    case 2:
      //turn to P2
      actuate_turnP2();
      complete.data = true;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;
    default:
      //bad message
      complete.data = false;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;
  }
}

void celebrate(const std_msgs::String& celeb_msg){  
  const String& msg = celeb_msg.data;
  //get player (digit)
  char last_char = msg.charAt(msg.length() - 1);
  if (last_char < '0' || last_char > '9') return;
  int player = last_char - '0';
  switch(player){
    case 0:
      //celebrate House
      actuate_celebrateHouse();
      complete.data = true;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;
    case 1:
      //celebrate p1
      actuate_celebrateP1();
      complete.data = true;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      
      break;
    case 2:
      //celebrate P2
      actuate_celebrateP2();
      complete.data = true;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;
    default:
      //bad message
      complete.data = false;
      //complete.header = ros::Time::now();
      movement_complete.publish(&complete);
      break;
  }
}

void move_camera(const std_msgs::String& cam_msg){
  const String& msg = cam_msg.data;
  if(msg == "CAMERA UP"){
      //move camera up
      actuate_headUp();
      complete.data = true;
      //complete.header = ros::Time::now();
      camera_up.publish(&complete);
    }
    else if(msg == "CAMERA DOWN"){
      //move camera down
      actuate_headDown();
      complete.data = true;
      //complete.header = ros::Time::now();
      camera_down.publish(&complete);
    }
}

//For testing ROS-connection
void messageCb(const std_msgs::UInt16& msg){
  hubert.loginfo("GOT MESSAGE");
  actuate_servo(body, BODY, msg.data);
}

//Subscriber
ros::Subscriber<std_msgs::String> subStart("start", &start_sequence);
ros::Subscriber<std_msgs::String> subDeal("deal_card", &deal_card);
ros::Subscriber<std_msgs::String> subTurn("turn_direction", &turn_direction);
ros::Subscriber<std_msgs::String> subCeleb("celebrate", &celebrate);
ros::Subscriber<std_msgs::String> subMoveCam("move_camera", &move_camera);
ros::Subscriber<std_msgs::UInt16> subEcho("chatter", messageCb);
void setup() {
  Serial.begin(57600); // Starts the serial communication
  //hubert.getHardware()->setPort(&Serial);
  hubert.getHardware()->setPort(&SerialUSB);
  hubert.initNode();
  hubert.subscribe(subStart);
  hubert.subscribe(subDeal);
  hubert.subscribe(subTurn);
  hubert.subscribe(subCeleb);
  hubert.subscribe(subMoveCam);

  hubert.advertise(movement_complete);
  //For testing ROS-connection
  //hubert.subscribe(subEcho);

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

  // We keep track of the current poses in the curr_pos array
  byte i;
  for (i=0; i<(sizeof(pos_init)/sizeof(int)); i++){
    curr_pos[i] = pos_init[i];
  }

	delay(2000);
}

void loop() {
  if(count == 0) {
    actuate_dodge();
    count++;
  }
  hubert.spinOnce();
  delay(1);
}
