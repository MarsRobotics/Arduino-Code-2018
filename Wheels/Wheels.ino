

/* 
 / This handles all of the device drivers for the Transporter.
 /
 / SABERTOOTH SETUP:
 /  If ON is "up", then DIPs 1-3 should be DOWN. DIPs 4-6 are address DIPs,
 / and will vary by ther sabertooth.
 / Sabertooth[s] should be connected on S1 to TX3 on an Arduino Mega 2560.
 / This allows it to read 9600-baud commands from the Arduino Mega.
 / Be careful to use TX3: TX0 should be reserved for Arduino-Computer
 / communications.
 */
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <command2ros/ManualCommand.h>
#include <EncoderFL.h>
#include <EncoderML.h>
#include <EncoderFR.h>
#include <EncoderMR.h>
#include <EncoderRL.h>
#include <EncoderRR.h>
#include "SabertoothDriverROS.h"


//The encoders we will use to monitor the angle of each articulation joint
EncoderFL efl;
EncoderML eml;
EncoderFR efr;
EncoderMR emr;
EncoderRL erl;
EncoderRR err;


//Control logic

//All proccesses killed, don't ever leave this state.
const int E_STOPPED = -1;

//Not driving any motors
const int STOPPED = 0;

//Articulating the wheels
const int ARTICULATING = 1;

//Done articulating, drive 
const int START_DRIVING = 2;

//Started articulating, drive
const int IS_DRIVING = 3;

//The state the robot is currently in
int currentStatus = STOPPED;

//time robot will drive before stopping
long driveTime = 0;

//
// Articualtion Constants
//

//We want to get articulation join within this range (degrees) from the target
const int DELTA_START_RANGE = 8;  //TODO ranges need to be diffrent
const int DELTA_STOP_RANGE = 3;

//When we are passing around the direction in which to articulate the wheels, we pass this value
const int ARTICULATION_DIRECTION_CLOCKWISE = -1;
const int ARTICULATION_DIRECTION_NONE = 0;
const int ARTICULATION_DIRECTION_COUNTER_CLOCKWISE = 1;

const int ENCODER_POSITIONS = 400; //TODO, change moto id's

//The speed 
const double ARTICULATION_DRIVE_SPEED = 6260.0 / 7521.0;
// This corresponds to the "maximum speed" available to the robot.
const int MAX_DRIVE_SPEED = 30;
bool motorInMotion[15];

//
// ROS initialization
//

// This node handle represents this arduino. 
ros::NodeHandle sabertoothDriverNode; //TODO maybie?

//
// ROS Publishers
// 

//Publisher to present the rotation of each of the articulation joints
command2ros::ManualCommand wheelOffset;
command2ros::ManualCommand wheelStatus;
ros::Publisher pubwheelStatus("current_rotation", &wheelStatus);// current rotation data for each wheel. 
// This will publish when the angle is updated.

//Publisher to print debug statements
std_msgs::String debugMsg;
ros::Publisher pubDebug("sabertooth_debugger", &debugMsg);

//
// ROS Subsribers
//

//Inbound wheel command
command2ros::ManualCommand wheelTarget;

// Subscribers for intended drive velocity and direction
void newManualCommandCallback(const command2ros::ManualCommand& newManualCommand) {      
  //TODO: Do we need to stop the robot at this point, Sherry and Matt H say no
  print("start cb");

  //Set articulation target values
  wheelTarget.fl_articulation_angle = newManualCommand.fl_articulation_angle;
  wheelTarget.ml_articulation_angle = newManualCommand.ml_articulation_angle;
  wheelTarget.rl_articulation_angle = newManualCommand.rl_articulation_angle;

  wheelTarget.fr_articulation_angle = newManualCommand.fr_articulation_angle;
  wheelTarget.mr_articulation_angle = newManualCommand.mr_articulation_angle;
  wheelTarget.rr_articulation_angle = newManualCommand.rr_articulation_angle;

  //Set wheel speeds
  wheelTarget.fl_drive_speed = newManualCommand.fl_drive_speed;
  wheelTarget.ml_drive_speed = newManualCommand.ml_drive_speed;
  wheelTarget.rl_drive_speed = newManualCommand.rl_drive_speed;

  wheelTarget.fr_drive_speed = newManualCommand.fr_drive_speed;
  wheelTarget.mr_drive_speed = newManualCommand.mr_drive_speed;
  wheelTarget.rr_drive_speed = newManualCommand.rr_drive_speed;

  //How long we will be driving from (in seconds)
  wheelTarget.drive_duration = newManualCommand.drive_duration;

  //Set E-Stop Command
  wheelTarget.e_stop = newManualCommand.e_stop;

  //Set the state of 
  if (wheelTarget.e_stop == true) {
    //TODO: Make this E_STOPPED for final product
      currentStatus = STOPPED;
    stopAllMotors(true);
  } 
  else if (needsToArticulate() == true) {
    currentStatus = ARTICULATING;
  } 
  else {
    currentStatus = START_DRIVING;
  }

}

void setActualArticulationValues(const command2ros::ManualCommand& articulationValues){
  //Set articulation values //TODO weird code Ryan Kane is confused 
  wheelOffset.fl_articulation_angle = (int)(articulationValues.fl_articulation_angle - wheelStatus.fl_articulation_angle - wheelOffset.fl_articulation_angle);
  wheelOffset.ml_articulation_angle = (int)(articulationValues.ml_articulation_angle - wheelStatus.ml_articulation_angle - wheelOffset.ml_articulation_angle);
  wheelOffset.rl_articulation_angle = (int)(articulationValues.rl_articulation_angle - wheelStatus.rl_articulation_angle - wheelOffset.rl_articulation_angle);

  wheelOffset.fr_articulation_angle = (int)(articulationValues.fr_articulation_angle - wheelStatus.fr_articulation_angle - wheelOffset.fr_articulation_angle);
  wheelOffset.mr_articulation_angle = (int)(articulationValues.mr_articulation_angle - wheelStatus.mr_articulation_angle - wheelOffset.mr_articulation_angle);
  wheelOffset.rr_articulation_angle = (int)(articulationValues.rr_articulation_angle - wheelStatus.rr_articulation_angle - wheelOffset.rr_articulation_angle);

  //pubwheelStatus.publish(&wheelOffset);// current rotation data for each wheel. 
}


ros::Subscriber<command2ros::ManualCommand> commandSubscriber("ManualCommand", &newManualCommandCallback);

ros::Subscriber<command2ros::ManualCommand> setActualArticulationSubscriber("SetActualArticulationValues", &setActualArticulationValues);


// Print error message to "sabertooth_debugger" topic
void print(char* errorMsg){
  debugMsg.data = errorMsg;
  pubDebug.publish(&debugMsg);
}

void setupWheelStatus() {
  //current wheel location: 0 is right of robot
  //left side of robot starts under robot at 0 degrees, rolls out to 180
  //right side of robot starts uner robot at 180 degrees, rolls out to 0
  wheelStatus.fl_articulation_angle = 0;
  wheelStatus.fr_articulation_angle = 180;
  wheelStatus.ml_articulation_angle = 0;
  wheelStatus.mr_articulation_angle = 180;
  wheelStatus.rl_articulation_angle = 0;
  wheelStatus.rr_articulation_angle = 180;

  //Amount turned from last movement (start)?
  wheelOffset.fl_articulation_angle = 0;
  wheelOffset.fr_articulation_angle = 0;
  wheelOffset.ml_articulation_angle = 0;
  wheelOffset.mr_articulation_angle = 0;
  wheelOffset.rl_articulation_angle = 0;
  wheelOffset.rr_articulation_angle = 0;  

  wheelStatus.fl_drive_speed = 0;
  wheelStatus.fr_drive_speed = 0;
  wheelStatus.ml_drive_speed = 0;
  wheelStatus.mr_drive_speed = 0;
  wheelStatus.rl_drive_speed = 0;
  wheelStatus.rr_drive_speed = 0;
}

/**
 * Checks the current angle of each of the articulation joints against their target value
 *    if the difference betwen any of them exceed the range given by DELTA_RANGE then we need 
 *    to articulate "return true"
 */
bool needsToArticulate() {
  int delta;
  int moveRange;
  //Check if the difference between target and actual articulation exceeds our given range
  delta = wheelStatus.fl_articulation_angle - wheelTarget.fl_articulation_angle;  //FL
  moveRange = (motorInMotion[FRONT_LEFT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  delta = wheelStatus.ml_articulation_angle - wheelTarget.ml_articulation_angle;  //ML
  moveRange = (motorInMotion[MIDDLE_LEFT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  delta = wheelStatus.rl_articulation_angle - wheelTarget.rl_articulation_angle;  //RL
  moveRange = (motorInMotion[REAR_LEFT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  delta = wheelStatus.fr_articulation_angle - wheelTarget.fr_articulation_angle;  //FR
  moveRange = (motorInMotion[FRONT_RIGHT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  delta = wheelStatus.mr_articulation_angle - wheelTarget.mr_articulation_angle;  //MR
  moveRange = (motorInMotion[MIDDLE_RIGHT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  delta = wheelStatus.rr_articulation_angle - wheelTarget.rr_articulation_angle;  //RR
  moveRange = (motorInMotion[REAR_RIGHT_ARTICULATION_MOTOR_ID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta < -moveRange || delta > moveRange) { 
    return true; 
  }

  return false;
}

void articulateAllWheels() {

  int wheelIds[6] = {
    FRONT_LEFT_ARTICULATION_MOTOR_ID,
    FRONT_RIGHT_ARTICULATION_MOTOR_ID, 
    MIDDLE_LEFT_ARTICULATION_MOTOR_ID, 
    MIDDLE_RIGHT_ARTICULATION_MOTOR_ID, 
    REAR_LEFT_ARTICULATION_MOTOR_ID,
    REAR_RIGHT_ARTICULATION_MOTOR_ID };
  int wheelArticulations[6];

  // Note: wheelTarget says it is the angle, but since we 
  // hacked at the last minute, it is actually time.
  wheelArticulations[0] = wheelTarget.fl_articulation_time;   //TODO change all _angle => _time
  wheelArticulations[1] = wheelTarget.fr_articulation_time;
  wheelArticulations[2] = wheelTarget.ml_articulation_time;
  wheelArticulations[3] = wheelTarget.mr_articulation_time;
  wheelArticulations[4] = wheelTarget.rl_articulation_time;
  wheelArticulations[5] = wheelTarget.rr_articulation_time;

  // Algorithm: "A nondeterministic-deterministic mergesort for Jaimiey" <3 
  boolean sorted = false;
  // Insertion sort
  for (int j = 0; j < 6; j++){
    int key = wheelArticulations[j];
    int key2 = wheelIds[j];
    int i = j - 1; 
    while (i >= 0 && abs(wheelArticulations[i]) > abs(key)){
      wheelArticulations[i + 1] = wheelArticulations[i];
      wheelIds[i + 1] = wheelIds[i];
      i = i - 1;
    }

    wheelArticulations[i + 1] = key;
    wheelIds[i + 1] = key2;
  }
  // End Algorithm

  // Iterate through all of the wheels, and 
  // start them if their times are greater than 0.
  for(int i = 0; i < 6; i++ ) {
    if(wheelArticulations[i] != 0) {
      // Drive clockwise when positive time is given
      if(wheelArticulations[i] > 0) {
        // Articulate
        driveClockwise(wheelIds[i], ARTICULATION_SPEED);
        // drive the wheel
        driveClockwise(wheelIds[i] - 6, ARTICULATION_SPEED*ARTICULATION_DRIVE_SPEED);
      }
      else {
        // Articulate
        driveCounterclockwise(wheelIds[i], ARTICULATION_SPEED);
        // Drive the wheel
        driveCounterclockwise(wheelIds[i] - 6, ARTICULATION_SPEED*ARTICULATION_DRIVE_SPEED);
      }
    }
  }

  // Delay the difference for each pair
  int last = wheelArticulations[0];
  delaySeconds((double)wheelArticulations[0]);
  for(int i = 1; i < 6; i++ ) {
    int diff = abs(wheelArticulations[i]) - abs(last);
    delaySeconds((double)diff);

    // Stop that motor
    driveCounterclockwise(wheelArticulations[last],0);  
    driveCounterclockwise(wheelArticulations[last] - 6, 0);
    last = wheelArticulations[i];
  }


  /*
  // Test all articulation motors:
  // 1.5s clockwise, then 1.5s counterclockwise.
  if(wheelTarget.fl_articulation_angle > 0){
    driveClockwise(FRONT_LEFT_ARTICULATION_MOTOR_ID, aSpeed);
    driveClockwise(FRONT_LEFT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)(int)wheelTarget.fl_articulation_angle); 
    driveClockwise(FRONT_LEFT_ARTICULATION_MOTOR_ID,0);
    driveClockwise(FRONT_LEFT_DRIVE_MOTOR_ID, 0);
  }
  else if(wheelTarget.fl_articulation_angle < 0){
    driveCounterclockwise(FRONT_LEFT_ARTICULATION_MOTOR_ID, aSpeed);
    driveCounterclockwise(FRONT_LEFT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)-(int)wheelTarget.fl_articulation_angle); 
    driveCounterclockwise(FRONT_LEFT_ARTICULATION_MOTOR_ID,0); 
    driveCounterclockwise(FRONT_LEFT_DRIVE_MOTOR_ID, 0);
  }
  else{
    driveCounterclockwise(FRONT_LEFT_ARTICULATION_MOTOR_ID,0);  
    driveCounterclockwise(FRONT_LEFT_DRIVE_MOTOR_ID, 0);

  }

  if(wheelTarget.fr_articulation_angle > 0){
    driveClockwise(FRONT_RIGHT_ARTICULATION_MOTOR_ID, aSpeed);
    driveClockwise(FRONT_RIGHT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)(int)wheelTarget.fr_articulation_angle); 
    driveClockwise(FRONT_RIGHT_ARTICULATION_MOTOR_ID,0);
    driveClockwise(FRONT_RIGHT_DRIVE_MOTOR_ID, 0);

  }
  else if(wheelTarget.fr_articulation_angle < 0){
    driveCounterclockwise(FRONT_RIGHT_ARTICULATION_MOTOR_ID, aSpeed);
    driveCounterclockwise(FRONT_RIGHT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)-(int)wheelTarget.fr_articulation_angle); 
    driveCounterclockwise(FRONT_RIGHT_ARTICULATION_MOTOR_ID,0); 
    driveCounterclockwise(FRONT_RIGHT_DRIVE_MOTOR_ID, 0);

  }
  else{
    driveCounterclockwise(FRONT_RIGHT_ARTICULATION_MOTOR_ID,0);  
    driveCounterclockwise(FRONT_RIGHT_DRIVE_MOTOR_ID, 0);

  }  

  if(wheelTarget.ml_articulation_angle > 0){
    driveClockwise(MIDDLE_LEFT_ARTICULATION_MOTOR_ID, aSpeed);
    driveClockwise(MIDDLE_LEFT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)(int)wheelTarget.ml_articulation_angle); 

    driveClockwise(MIDDLE_LEFT_ARTICULATION_MOTOR_ID,0);
    driveClockwise(MIDDLE_LEFT_DRIVE_MOTOR_ID, 0);

  }
  else if(wheelTarget.ml_articulation_angle < 0){
    driveCounterclockwise(MIDDLE_LEFT_ARTICULATION_MOTOR_ID, aSpeed);
    driveCounterclockwise(MIDDLE_LEFT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)-(int)wheelTarget.ml_articulation_angle); 

    driveCounterclockwise(MIDDLE_LEFT_ARTICULATION_MOTOR_ID,0);
    driveCounterclockwise(MIDDLE_LEFT_DRIVE_MOTOR_ID, 0);

  }
  else{
    driveCounterclockwise(MIDDLE_LEFT_ARTICULATION_MOTOR_ID,0);  
    driveCounterclockwise(MIDDLE_LEFT_DRIVE_MOTOR_ID, 0);

  }

  if(wheelTarget.mr_articulation_angle > 0){
    driveClockwise(MIDDLE_RIGHT_ARTICULATION_MOTOR_ID, aSpeed);
    driveClockwise(MIDDLE_RIGHT_DRIVE_MOTOR_ID, dSpeed);


    delaySeconds((double)(int)wheelTarget.mr_articulation_angle); 
    driveClockwise(MIDDLE_RIGHT_ARTICULATION_MOTOR_ID,0);
    driveClockwise(MIDDLE_RIGHT_DRIVE_MOTOR_ID, 0);

  }
  else if(wheelTarget.mr_articulation_angle < 0){
    driveCounterclockwise(MIDDLE_RIGHT_ARTICULATION_MOTOR_ID, aSpeed);
    driveCounterclockwise(MIDDLE_RIGHT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)-(int)wheelTarget.mr_articulation_angle); 
    driveCounterclockwise(MIDDLE_RIGHT_ARTICULATION_MOTOR_ID,0);  
    driveCounterclockwise(MIDDLE_RIGHT_DRIVE_MOTOR_ID, 0);

  }
  else{
    driveCounterclockwise(MIDDLE_RIGHT_ARTICULATION_MOTOR_ID,0); 
    driveCounterclockwise(MIDDLE_RIGHT_DRIVE_MOTOR_ID, 0);

  }

  if(wheelTarget.rl_articulation_angle > 0){
    driveClockwise(REAR_LEFT_ARTICULATION_MOTOR_ID, aSpeed);
    driveClockwise(REAR_LEFT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)(int)wheelTarget.rl_articulation_angle); 
    driveClockwise(REAR_LEFT_ARTICULATION_MOTOR_ID,0);
    driveClockwise(REAR_LEFT_DRIVE_MOTOR_ID, 0);

  }
  else if(wheelTarget.rl_articulation_angle < 0){
    driveCounterclockwise(REAR_LEFT_ARTICULATION_MOTOR_ID, aSpeed);
    driveCounterclockwise(REAR_LEFT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)-(int)wheelTarget.rl_articulation_angle); 
    driveCounterclockwise(REAR_LEFT_ARTICULATION_MOTOR_ID,0); 
    driveCounterclockwise(REAR_LEFT_DRIVE_MOTOR_ID, 0);

  }
  else{
    driveCounterclockwise(REAR_LEFT_ARTICULATION_MOTOR_ID,0);  
    driveCounterclockwise(REAR_LEFT_DRIVE_MOTOR_ID, 0);

  }

  if(wheelTarget.rr_articulation_angle > 0){
    driveClockwise(REAR_RIGHT_ARTICULATION_MOTOR_ID, aSpeed);
    driveClockwise(REAR_RIGHT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)(int)wheelTarget.rr_articulation_angle); 
    driveClockwise(REAR_RIGHT_ARTICULATION_MOTOR_ID,0);
    driveClockwise(REAR_RIGHT_DRIVE_MOTOR_ID, 0);

  }
  else if(wheelTarget.rr_articulation_angle < 0){
    driveCounterclockwise(REAR_RIGHT_ARTICULATION_MOTOR_ID, aSpeed);
    driveCounterclockwise(REAR_RIGHT_DRIVE_MOTOR_ID, dSpeed);

    delaySeconds((double)-(int)wheelTarget.rr_articulation_angle); 
    driveCounterclockwise(REAR_RIGHT_ARTICULATION_MOTOR_ID,0);  
    driveCounterclockwise(REAR_RIGHT_DRIVE_MOTOR_ID, 0);

  }
  else{
    driveCounterclockwise(REAR_RIGHT_ARTICULATION_MOTOR_ID,0);  
    driveCounterclockwise(REAR_RIGHT_DRIVE_MOTOR_ID, 0);

  }*/
  
  
  /*int delta;
   int direction;
   
   direction = getArticulationDirection(FRONT_LEFT_DRIVE_MOTOR_ID, wheelStatus.fl_articulation_angle, wheelTarget.fl_articulation_angle); //FL
   articulateWheel(FRONT_LEFT_DRIVE_MOTOR_ID, direction);
   
   direction = getArticulationDirection(MIDDLE_LEFT_DRIVE_MOTOR_ID, wheelStatus.ml_articulation_angle, wheelTarget.ml_articulation_angle); //ML
   articulateWheel(MIDDLE_LEFT_DRIVE_MOTOR_ID, direction);
   
   direction = getArticulationDirection(REAR_LEFT_DRIVE_MOTOR_ID, wheelStatus.rl_articulation_angle, wheelTarget.rl_articulation_angle); //RL
   articulateWheel(REAR_LEFT_DRIVE_MOTOR_ID, direction);
   
   direction = getArticulationDirection(FRONT_RIGHT_DRIVE_MOTOR_ID, wheelStatus.fr_articulation_angle, wheelTarget.fr_articulation_angle); //FR
   articulateWheel(FRONT_RIGHT_DRIVE_MOTOR_ID, direction);
   
   direction = getArticulationDirection(MIDDLE_RIGHT_DRIVE_MOTOR_ID, wheelStatus.mr_articulation_angle, wheelTarget.mr_articulation_angle); //MR
   articulateWheel(MIDDLE_RIGHT_DRIVE_MOTOR_ID, direction);
   
   direction = getArticulationDirection(REAR_RIGHT_DRIVE_MOTOR_ID, wheelStatus.rr_articulation_angle, wheelTarget.rr_articulation_angle); //RR
   articulateWheel(REAR_RIGHT_DRIVE_MOTOR_ID, direction);*/
}

/**
 *
 */
int getArticulationDirection(int motorID, int from, int to) {

  int delta = from-to;

  int moveRange = (motorInMotion[motorID]) ? DELTA_STOP_RANGE : DELTA_START_RANGE; 
  if (delta >= -moveRange && delta <= moveRange) {
    return ARTICULATION_DIRECTION_NONE;
  }

  // Calculations are made by "normalizing" the data so that we are always starting from "0" 
  // and heading towards the target

  // If the current articulation angle is between 0 and 180, then we want to shift the 
  // whole frame of reference clockwise

  //Shift "to" and "from" so that "to" is the angle that we need to turn and "from" is 0
  if(from >= 0 && from <= 180){

    // shift "to" by however much "from" shifted (counter clockwise)
    to -= from;
    if(to < 0){
      to += 360;
    }

    // shift "from" to 0 degrees
    from = 0;
  }
  else{
    delta = 360 - from;

    // shift "from" to 0 degrees
    from = 0;

    // shift "to" by however much "from" shifted (clockwise)
    to += delta;
    if(to > 360){
      to -= 360;
    }
  }

  //Now we compare the two distances and move in the diection that is fastest
  int counterClockwiseDistance = to;
  int clockwiseDistance = 360 - to;

  if(counterClockwiseDistance > clockwiseDistance){
    return ARTICULATION_DIRECTION_CLOCKWISE;
  }
  else{
    return ARTICULATION_DIRECTION_COUNTER_CLOCKWISE;
  }
}

/**
 *
 */
void articulateWheel(int motorID, int direction) {

  int articulationSpeed = 30*ARTICULATION_DRIVE_SPEED;
  int wheelSpeed = 30;

  //TODO: Make a constant
  int articulationID = motorID + 6;

  if (direction == ARTICULATION_DIRECTION_CLOCKWISE) {
    driveClockwise(articulationID, articulationSpeed);
    driveClockwise(motorID, wheelSpeed);
  } 
  else if (direction == ARTICULATION_DIRECTION_COUNTER_CLOCKWISE) {
    driveCounterclockwise(articulationID, articulationSpeed);
    driveCounterclockwise(motorID, wheelSpeed);
  }
  else if(direction == ARTICULATION_DIRECTION_NONE){
    driveCounterclockwise(articulationID, 0);
    driveCounterclockwise(motorID, 0);
  }
}

void driveAllWheels() {

  //Set wheel speeds
  driveWheel(FRONT_LEFT_DRIVE_MOTOR_ID, wheelTarget.fl_drive_speed*-1);
  driveWheel(MIDDLE_LEFT_DRIVE_MOTOR_ID, wheelTarget.ml_drive_speed*-1);
  driveWheel(REAR_LEFT_DRIVE_MOTOR_ID, wheelTarget.rl_drive_speed*-1);

  driveWheel(FRONT_RIGHT_DRIVE_MOTOR_ID, wheelTarget.fr_drive_speed);
  driveWheel(MIDDLE_RIGHT_DRIVE_MOTOR_ID, wheelTarget.mr_drive_speed);
  driveWheel(REAR_RIGHT_DRIVE_MOTOR_ID, wheelTarget.rr_drive_speed);

}

void driveWheel(int motorID, int speed) {

  if (speed < 0) {
    speed = speed * -1;
    driveClockwise(motorID, speed);
  } 
  else {
    speed = speed;
    driveCounterclockwise(motorID, speed);
  }

}

/**
 * Stops all the motors (Articulation, conveyor, winch, and Wheels) and chages the current state to reflect the stop
 *
 * Parameters:
 *  EStop - Whether or not this is an E-Stop or a normal end of command stop
 */
void stopAllMotors(bool EStop) {

  if (EStop) {
    currentStatus = STOPPED;
  } 
  else {
    currentStatus = E_STOPPED;
  }

  //Stop all motors 
  const int speed = 0;
  for(int motorID = 0; motorID <= 14; ++motorID) {
    driveClockwise(motorID, speed);
  }
}

/**
 * Stops the movement motors (Articulation and Wheels) and chages the current state to reflect the stop.
 */
void stopArticulationAndDriveMotors() {
  currentStatus = STOPPED;

  //Stop all movement (drive and articulation) motors 
  const int speed = 0;
  for(int motorID = 0; motorID <= 11; ++motorID) {
    driveClockwise(motorID, speed);
  }
}


/**
 * Drives a given motor at a given speed in a clockwise direction
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveClockwise(int motorID, int speed){
  if(0 == speed) {
    motorInMotion[motorID] = false; 
  }
  else {
    motorInMotion[motorID] = true;
  }

  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
  // Build the data packet:
  // Get the address and motor command ID from a predefined array.
  unsigned char address = MOTOR_ADDRESS[motorID];
  unsigned char command = MOTOR_COMMAND[motorID];
  // If the motor is connected backwards, we need to flip the command from 0/4 to 1/5:
  if(MOTOR_FLIPPED[motorID]) {
    command += 1;
  }
  unsigned char checksum = (address + command + ((char)speed)) & 0b01111111;

  // Write the packet.
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(((char)speed));
  Serial3.write(checksum);
  //TODO: Move the delay time to a constant
  delayMicroseconds(1000); 
}

/**
 * Drives a given motor at a given speed in a counterclockwise direction
 *
 * Paramteters:
 *  motorID - the id of the motor to spin (0-11)
 *  speed - the speed at which to spin the motor.
 */
void driveCounterclockwise(char motorID, char speed){ 
  if(0 == speed) {
    motorInMotion[motorID] = false; 
  }
  else {
    motorInMotion[motorID] = true;
  }
  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
  unsigned char address = MOTOR_ADDRESS[motorID];
  unsigned char command = MOTOR_COMMAND[motorID] + 1;
  // If the motor is connected backwards, we need to flip the command from 1/5 to 0/4:
  if(MOTOR_FLIPPED[motorID]) {
    command -= 1;
  }
  unsigned char checksum = (address + command + speed) & 0b01111111;
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(speed);
  Serial3.write(checksum);
  //TODO: Move the delay time to a constant
  delayMicroseconds(1000);
}

/**
 * Gets the last known position of each articulation joint, and updates acordingly.
 */
void updateArticulationValues()  {

  /*
  // Code for command line testing purposes
   wheelStatus.ml_articulation_angle = wheelTarget.ml_articulation_angle;
   wheelStatus.rl_articulation_angle = wheelTarget.rl_articulation_angle;
   wheelStatus.fl_articulation_angle = wheelTarget.fl_articulation_angle;
   wheelStatus.mr_articulation_angle = (int)wheelTarget.mr_articulation_angle;
   wheelStatus.rr_articulation_angle = (int)wheelTarget.rr_articulation_angle;
   wheelStatus.fr_articulation_angle = (int)wheelTarget.fr_articulation_angle;
   */

  int encoderPostition = EncoderFL::getPosition() + (int)(wheelOffset.fl_articulation_angle / 360 * 400);
  while (encoderPostition < 0) {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.fl_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderML::getPosition() + (int)(wheelOffset.ml_articulation_angle / 360 * 400);
  while (encoderPostition < 0) {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.ml_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderRL::getPosition() + (int)(wheelOffset.rl_articulation_angle / 360 * 400);
  while (encoderPostition < 0) {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.rl_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderFR::getPosition() + (int)(wheelOffset.fr_articulation_angle / 360 * 400) - 200; // Subtract 200 b/c the right side is flipped.
  while (encoderPostition < 0) {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.fr_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderMR::getPosition() + (int)(wheelOffset.mr_articulation_angle / 360 * 400) - 200;
  while (encoderPostition < 0) {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.mr_articulation_angle = (int)((encoderPostition * 9L) / 10L);

  encoderPostition = EncoderRR::getPosition() + (int)(wheelOffset.rr_articulation_angle / 360 * 400) - 200;
  while (encoderPostition < 0) {
    encoderPostition += ENCODER_POSITIONS;
  }
  wheelStatus.rr_articulation_angle = (int)((encoderPostition * 9L) / 10L);
}

void unitTest(){
  // delay after set up
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000); //TODO: check for setup with actual values not time
  }

  // loop through and test all drive motors:
  // 1.5s clockwise, then 1.5s counterclockwise.
  for(int i = 0; i < 6; i++){
    // clockwise
    driveClockwise(i, 20);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }

    // counterclockwise
    driveCounterclockwise(i, 20);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }   

    // stop
    driveCounterclockwise(i,0);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }
  }

  // Test all articulation motors:
  // 1.5s clockwise, then 1.5s counterclockwise.
  for(int i = 6; i < 12; ++i){
    // clockwise
    driveClockwise(i, 30);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }   

    // counterclockwise
    driveCounterclockwise(i, 30);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    } 

    // stop 
    driveClockwise(i,0);
    for(int j = 0; j < 100; j++){
      delayMicroseconds(15000);
    }
  }
  // Test the winch.
  //unitTestWinch();
  // Test the conveyor.
  //unitTestConveyor(); 
}  


void unitTest2(){
  // delay after startup
  for(int j = 0; j < 300; j++){
    delayMicroseconds(15000);
  }


  digitalWrite(13, HIGH);    // turn the LED off by making the voltage LOW
  driveCounterclockwise(0, 20);
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000);
  }

  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
  driveCounterclockwise(0,0);
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000);
  }

  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  driveClockwise(0, 20);
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000);
  }  

  digitalWrite(13, LOW);   // turn the LED on (HIGH is the voltage level)
  driveClockwise(0,0);
  for(int j = 0; j < 100; j++){
    delayMicroseconds(15000);
  }
}

void proofOfLife() {
  // Proof of life newspaper (5s)
  delaySeconds((double)5);
  // Spin conveyor backwards (10s)
  driveCounterclockwise(RIGHT_CONVEYOR_MOTOR_ID, CONVEYOR_SPEED);
  delaySeconds((double)10);
  driveCounterclockwise(RIGHT_CONVEYOR_MOTOR_ID, 0);
  // Lift the winch (25s)
  driveClockwise(WINCH_MOTOR_ID, WINCH_SPEED_UP);
  delaySeconds((double)25);
  driveClockwise(WINCH_MOTOR_ID, 0);
  // Drive forwards for 10s
  char driveSpeed = 20;
  driveClockwise(FRONT_LEFT_DRIVE_MOTOR_ID, driveSpeed);
  driveCounterclockwise(FRONT_RIGHT_DRIVE_MOTOR_ID, driveSpeed);
  driveClockwise(MIDDLE_LEFT_DRIVE_MOTOR_ID, driveSpeed);
  driveCounterclockwise(MIDDLE_RIGHT_DRIVE_MOTOR_ID, driveSpeed);
  driveClockwise(REAR_LEFT_DRIVE_MOTOR_ID, driveSpeed);
  driveCounterclockwise(REAR_RIGHT_DRIVE_MOTOR_ID, driveSpeed);
  delaySeconds((double)10);
  driveClockwise(FRONT_LEFT_DRIVE_MOTOR_ID, 0);
  driveCounterclockwise(FRONT_RIGHT_DRIVE_MOTOR_ID, 0);
  driveClockwise(MIDDLE_LEFT_DRIVE_MOTOR_ID, 0);
  driveCounterclockwise(MIDDLE_RIGHT_DRIVE_MOTOR_ID, 0);
  driveClockwise(REAR_LEFT_DRIVE_MOTOR_ID, 0);
  driveCounterclockwise(REAR_RIGHT_DRIVE_MOTOR_ID, 0);
}


void delaySeconds(double n){
  unsigned int delayTime = 10000;
  long desiredMicroDelay = (long)(n * 1000000L);
  long numCycles = desiredMicroDelay / (long)delayTime;
  for(long j = 0L; j < numCycles; j++){
    delayMicroseconds(delayTime);
  }
}


/**
 * Is called on starting the arduino.
 * 
 */
void setup(){
  // To start, no motor is moving. //TODO: input correct number of motors
  for(int i = 0; i < 15; ++i) {
    motorInMotion[i] = false; 
  }

  // Setup the encoders
  EncoderFL::setupEncoderFL();
  EncoderML::setupEncoderML();
  EncoderRL::setupEncoderRL();
  EncoderFR::setupEncoderFR();
  EncoderMR::setupEncoderMR();
  EncoderRR::setupEncoderRR();

  //Initialize the ROS Node
  sabertoothDriverNode.initNode();
  sabertoothDriverNode.subscribe(commandSubscriber);
  sabertoothDriverNode.subscribe(setActualArticulationSubscriber);
  sabertoothDriverNode.advertise(pubwheelStatus);
  sabertoothDriverNode.advertise(pubDebug);

  // Open communication with Saberteeth
  Serial3.begin(9600);

  stopAllMotors(true);
  //unitTest();
  //unitTestConveyor();
  //unitTestWinch();
  //proofOfLife();

  // Initialize the current wheel status message
  setupWheelStatus();
}

// This program does whatever ROS directs it to do.
// So far, it drives the robot forwards and backwards.
void loop(){
  updateArticulationValues(); 

  //We are E-Stopped, don't respond to future commands.
  if (currentStatus == E_STOPPED) {
    //TODO: This will never happen right now, may want for competition though
    return;
  } 

  
  //Currently stopped, don't do anything.
  // ASSUMES the robot is stopped when currentStatus is set to STOPPED.
  if (currentStatus == STOPPED) {
    // do nothing
  }

  if (currentStatus == ARTICULATING) {
    print("needs to articulate: true");
    //if (needsToArticulate() == true) {
    articulateAllWheels();
    //} 
    //else {
    // print("needs to articulate: false");
    stopArticulationAndDriveMotors();
    currentStatus = START_DRIVING;
    //}
  }

  if (currentStatus == START_DRIVING) {
    //TODO: Check with MEs about possibility of losing correct articulation (by going over an obstacle or something)

    //Check if we need to articulate
    // if (needsToArticulate()) {
    //   //Set state to articulate and don't drive
    //   currentStatus = ARTICULATING;
    //   // TODO: Does this need a rosnode.spin() ?
    //   return;
    // }

    print("current status: start_driving");
    //Set stop drive time
    driveTime = millis() + (long)wheelTarget.drive_duration*1000;

    //Send drive start command
    driveAllWheels();

    currentStatus = IS_DRIVING;
  }

  if (currentStatus == IS_DRIVING) {
    if (millis() >= driveTime) {
      print("over time limit: stop driving");
      stopArticulationAndDriveMotors();
    }
  }

  pubwheelStatus.publish(&wheelStatus);// current rotation data for each wheel. 

  //Sync with ROS
  sabertoothDriverNode.spinOnce(); // Check for subscriber update/update timestamp

  //Delay so we don't overload any serial buffers
  for(int i = 0; i < 7; i++){
    delayMicroseconds(15000);
  }
}









