
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

//Publisher
std_msgs::bool complete;
ros::Publisher movement_complete("movement_complete", complete);
ros::Publisher camera_up("camera_up", complete);
ros::Publisher camera_down("camera_down", complete);

//Subscriber
ros::Subscriber<std_msgs::string> sub("/deal_card", deal_card);
ros::Subscriber<std_msgs::string> sub("/turn_direction", turn_direction);
ros::Subscriber<std_msgs::string> sub("/celebrate", celebrate);
ros::Subscriber<std_msgs::string> sub("/move_camera", move_camera);

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

// to be updated
const int bodyDodge = 550;
const int bodyDeck = 1300;
const int bodyP1 = 1700;
const int bodyP2 = 2000;
const int bodyHouse = 2200;

const int headUp = 2000;
const int headDown = 1700;

const int shoulderCelebrate = 1500;

const int elbowRaised = 2100;
const int elbowDeck = 1750;
const int elbowPlace = 1500;

const int gripperOpen = 1000;
const int gripperClosed = 1500;

const int wristNeutral = 450;
const int wristPlace = 1850;


const int pos_init[SERVO_COUNT] =  {1700, 1500, 2000, 2300, 900, 1000, 450};
const int pos_dodge[SERVO_COUNT] = {900, 2000, 2000, 2300, 2100, 1000, 450};
const int pos_deck[SERVO_COUNT] =  {2100, 2000, 2000, 2300, 2100, 1000, 450};
const int pos_pick[SERVO_COUNT] =  {2100, 1700, 2000, 2300, 1750, 1000, 450};
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
  actuate_servo(headPan, HEAD_PAN, headUp);
  actuate_servo(body, BODY, bodyDodge);
  actuate_servo(elbow, ELBOW, elbowRaised);
} 

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
void actuate_celebrateP1() {
  actuate_servo(body, BODY, bodyHouse);
  actuate_servo(shoulder, SHOULDER, shoulderCelebrate);
  actuate_servo(elbow, ELBOW, elbowDeck);
  actuate_servo(elbow, ELBOW, elbowRaised);
  actuate_servo(shoulder, SHOULDER, pos_init[SHOULDER]);
}

//Functions for the messages we subscribe for:
void deal_card(std_msgs::string deal_msg){
  switch(deal_msg){
    case "DEAL 0":
      //deal to p1
      movement_complete.publish(true);
      break;
    
    case "DEAL 1":
      //deal to p2
      movement_complete.publish(true);
      break;

    case "DEAL 2":
      //deal to house
      movement_complete.publish(true);
      break;
    
    default:
      //bad string
      movement_complete.publish(false);
      break;
  }
}

void turn_direction(std_msgs::string turn_msg){
  switch(turn_msg){
    case "TURN 0":
      //turn to p1
      movement_complete.publish(true);
      break;
    case "TURN 1";
      //turn to p2
      movement_complete.publish(true);
      break;
    case "TURN 2";
      //turn to house
      movement_complete.publish(true);
      break;
    default:
      //bad message
      movement_complete.publish(false);
      break;
  }
}

void celebrate(std_msgs::string celeb_msg){  
  switch(celeb_msg){
    case "CELEBRATE 0":
      //celebrate p1
      movement_complete.publish(true);
      break;
    case "CELEBRATE 1";
      movement_complete.publish(true);
      //celebrate p2
      break;
    case "CELEBRATE 2";
      //celebrate house
      movement_complete.publish(true);
      break;
    default:
      //bad message
      movement_complete.publish(false);
      break;
  }
}

void move_camera(std_msgs::string cam_msg){
  switch(cam_msg){
    case "CAMERA UP":
      //move camera up
      actuate_headUp();
      camera_up.publish(true);
      break;
    case "CAMERA DOWN";
      //move camera down
      actuate_headDown();
      camera_down.publish(true);
      break;
    default:
      break;
  }
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
  hubert.spinOnce();
  

}
