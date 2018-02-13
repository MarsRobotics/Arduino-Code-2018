#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <command2ros/ManualCommand.h>
#include "SabertoothDriverROS.h"

NUM_MOTORS = 6

//node handle to represent this arduino
ros::NodeHandle sabertoothDriverNode;

//Publisher to present the rotation of each of the articulation joints
command2ros::ManualCommand wheelOffset; //how far has rotated
command2ros::ManualCommand wheelStatus; //current angle
command2ros::ManualCommand wheelTarget; //target angle

//publish current rotation data for each wheel when angle is updated
ros::Publisher pubwheelStatus("currentRotation", &wheelStatus);

//Publisher to print debug statements
std_msgs::String debugMsg;
ros::Publisher pubDebug("sabertoothDebugger", &debugMsg);

//Subscribers for intended drive velocity and direction of articulation motors
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

  //How long we will be driving (in seconds)
  wheelTarget.drive_duration = newManualCommand.drive_duration;

  //Set E-Stop Command
  wheelTarget.e_stop = newManualCommand.e_stop;

  //Set the state of the articulation motors
  if (wheelTarget.e_stop) {
    //TODO: Make this E_STOPPED for final product
    currentStatus = STOPPED;
    stopAllMotors(true);
  } 
  else if (needsToArticulate()) {
    currentStatus = ARTICULATING;
  } 
  else {
    currentStatus = START_DRIVING;
  }
}

//Update to actual (current) articulation angles and publish how far each wheel has rotated
void setActualArticulationValues(const command2ros::ManualCommand& articulationValues){
  //Set articulation values //TODO weird code Ryan Kane is confused 
  wheelOffset.fl_articulation_angle = (int)(articulationValues.fl_articulation_angle - wheelStatus.fl_articulation_angle - wheelOffset.fl_articulation_angle);
  wheelOffset.ml_articulation_angle = (int)(articulationValues.ml_articulation_angle - wheelStatus.ml_articulation_angle - wheelOffset.ml_articulation_angle);
  wheelOffset.rl_articulation_angle = (int)(articulationValues.rl_articulation_angle - wheelStatus.rl_articulation_angle - wheelOffset.rl_articulation_angle);

  wheelOffset.fr_articulation_angle = (int)(articulationValues.fr_articulation_angle - wheelStatus.fr_articulation_angle - wheelOffset.fr_articulation_angle);
  wheelOffset.mr_articulation_angle = (int)(articulationValues.mr_articulation_angle - wheelStatus.mr_articulation_angle - wheelOffset.mr_articulation_angle);
  wheelOffset.rr_articulation_angle = (int)(articulationValues.rr_articulation_angle - wheelStatus.rr_articulation_angle - wheelOffset.rr_articulation_angle);

  pubwheelStatus.publish(&wheelOffset);// current rotation data for each wheel. 
}

ros::Subscriber<command2ros::ManualCommand> commandSubscriber("ManualCommand", &newManualCommandCallback);
ros::Subscriber<command2ros::ManualCommand> setActualArticulationSubscriber("SetActualArticulationValues", &setActualArticulationValues);

//roll out wheels for starting position
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

  //Amount turned towards target angle
  wheelOffset.fl_articulation_angle = 0;
  wheelOffset.fr_articulation_angle = 0;
  wheelOffset.ml_articulation_angle = 0;
  wheelOffset.mr_articulation_angle = 0;
  wheelOffset.rl_articulation_angle = 0;
  wheelOffset.rr_articulation_angle = 0;  

  //current wheel speed
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

// Print error message to "sabertoothDebugger" topic
void print(char* errorMsg){
  debugMsg.data = errorMsg;
  pubDebug.publish(&debugMsg);
}

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
  for(int motorID = 0; motorID < NUM_MOTORS; ++motorID) {
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
  Serial1.write(address);
  Serial1.write(command);
  Serial1.write(((char)speed));
  Serial1.write(checksum);
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
  Serial1.write(address);
  Serial1.write(command);
  Serial1.write(speed);
  Serial1.write(checksum);
  //TODO: Move the delay time to a constant
  delayMicroseconds(1000);
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
  for(int i = 0; i < NUM_MOTORS; ++i) {
    motorInMotion[i] = false; 
  }

  // Setup the encoders

  //Initialize the ROS Node
  sabertoothDriverNode.initNode();
  sabertoothDriverNode.subscribe(commandSubscriber);
  sabertoothDriverNode.subscribe(setActualArticulationSubscriber);
  sabertoothDriverNode.advertise(pubwheelStatus);
  sabertoothDriverNode.advertise(pubDebug);

  // Open communication with Saberteeth
  Serial1.begin(9600);

  stopAllMotors(true);

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