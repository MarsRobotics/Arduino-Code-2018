// Header file for the SabertoothDriverROS class.
#include <command2ros/ManualCommand.h>
#include "Conveyor.h"


// DATA STRUCTURES
// Wheel information data structure
typedef struct wheel_status_tag {
  int orientation;
  long velocity;
} Wheel_status;

// CONSTANTS 
// We'll track which direction we're currently moving in this demo
// So we don't have to write more packets than we have to.
const char FORWARD_LABEL = 1;
const char BACKWARD_LABEL = 2;
// Motor constants (used to communicate with the onboard computer).
const unsigned char FRONT_LEFT_DRIVE_MOTOR_ID = 0;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_ID = 1; 
const unsigned char MIDDLE_LEFT_DRIVE_MOTOR_ID = 2;  // went clockwise. positive on purple
const unsigned char MIDDLE_RIGHT_DRIVE_MOTOR_ID = 3; // went clockwise. positive on purple
const unsigned char REAR_LEFT_DRIVE_MOTOR_ID = 4;    // Went clockwise. positive on white
const unsigned char REAR_RIGHT_DRIVE_MOTOR_ID = 5;
// Articulation motors
const unsigned char FRONT_LEFT_ARTICULATION_MOTOR_ID = 6;
const unsigned char FRONT_RIGHT_ARTICULATION_MOTOR_ID = 7; 
const unsigned char MIDDLE_LEFT_ARTICULATION_MOTOR_ID = 8;  // went clockwise. 
const unsigned char MIDDLE_RIGHT_ARTICULATION_MOTOR_ID = 9; // went clockwise. 
const unsigned char REAR_LEFT_ARTICULATION_MOTOR_ID = 10;   // Went clockwise. 
const unsigned char REAR_RIGHT_ARTICULATION_MOTOR_ID = 11;  // Sounds terrible

// Motor addressing (used to link the computer-constants to
// commands sent to the saberteeth).
const unsigned char FRONT_LEFT_DRIVE_MOTOR_ADDRESS = 130;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_ADDRESS = 133;
const unsigned char MIDDLE_LEFT_DRIVE_MOTOR_ADDRESS = 129;
const unsigned char MIDDLE_RIGHT_DRIVE_MOTOR_ADDRESS = 134;
const unsigned char REAR_LEFT_DRIVE_MOTOR_ADDRESS = 128;
const unsigned char REAR_RIGHT_DRIVE_MOTOR_ADDRESS = 132;
// Articulation motor addressing
const unsigned char FRONT_LEFT_ARTICULATION_MOTOR_ADDRESS = 130;
const unsigned char FRONT_RIGHT_ARTICULATION_MOTOR_ADDRESS = 133;
const unsigned char MIDDLE_LEFT_ARTICULATION_MOTOR_ADDRESS = 129;
const unsigned char MIDDLE_RIGHT_ARTICULATION_MOTOR_ADDRESS = 134;
const unsigned char REAR_LEFT_ARTICULATION_MOTOR_ADDRESS = 128;
const unsigned char REAR_RIGHT_ARTICULATION_MOTOR_ADDRESS = 132;


// Motor addressing offset: If a motor is connected to M1, 
// This will be 0. If the motor is connected to M2, this should
// be 4.
// A value of 0 or 4 corresponds to 'clockwise'. A value of 1 or 5
// corresponds to 'counterclockwise'.
const unsigned char FRONT_LEFT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char MIDDLE_LEFT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char MIDDLE_RIGHT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char REAR_LEFT_DRIVE_MOTOR_COMMAND = 0;
const unsigned char REAR_RIGHT_DRIVE_MOTOR_COMMAND = 4;
// Articulation motor offsets. 0 or 4 corresponds to "counterclockwise".
// a value of 1 or 5 corresponds to "clockwise".
const unsigned char FRONT_LEFT_ARTICULATION_MOTOR_COMMAND = 4;
const unsigned char FRONT_RIGHT_ARTICULATION_MOTOR_COMMAND = 4;
const unsigned char MIDDLE_LEFT_ARTICULATION_MOTOR_COMMAND = 4;
const unsigned char MIDDLE_RIGHT_ARTICULATION_MOTOR_COMMAND = 4;
const unsigned char REAR_LEFT_ARTICULATION_MOTOR_COMMAND = 4;
const unsigned char REAR_RIGHT_ARTICULATION_MOTOR_COMMAND = 0;

// Direction indicator. 0 means "correct" (no software flip necessary). 1 means "backwards" (software flip necessary).
const unsigned char FRONT_LEFT_DRIVE_MOTOR_FLIPPED = 1;
const unsigned char FRONT_RIGHT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char MIDDLE_LEFT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char MIDDLE_RIGHT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char REAR_LEFT_DRIVE_MOTOR_FLIPPED = 0;
const unsigned char REAR_RIGHT_DRIVE_MOTOR_FLIPPED = 0;
// Articulation motor offsets. 0 or 4 corresponds to "counterclockwise".
// a value of 1 or 5 corresponds to "clockwise".
const unsigned char FRONT_LEFT_ARTICULATION_MOTOR_FLIPPED = 0;
const unsigned char FRONT_RIGHT_ARTICULATION_MOTOR_FLIPPED = 0;
const unsigned char MIDDLE_LEFT_ARTICULATION_MOTOR_FLIPPED = 0;
const unsigned char MIDDLE_RIGHT_ARTICULATION_MOTOR_FLIPPED = 0;
const unsigned char REAR_LEFT_ARTICULATION_MOTOR_FLIPPED = 1;
const unsigned char REAR_RIGHT_ARTICULATION_MOTOR_FLIPPED = 0;

// shorthand that will allow us to use the motor ID as an array index to access
// the relevant constants.
const unsigned char MOTOR_ADDRESS[15] = {
  FRONT_LEFT_DRIVE_MOTOR_ADDRESS,
  FRONT_RIGHT_DRIVE_MOTOR_ADDRESS,
  MIDDLE_LEFT_DRIVE_MOTOR_ADDRESS,
  MIDDLE_RIGHT_DRIVE_MOTOR_ADDRESS,
  REAR_LEFT_DRIVE_MOTOR_ADDRESS,
  REAR_RIGHT_DRIVE_MOTOR_ADDRESS,
  FRONT_LEFT_ARTICULATION_MOTOR_ADDRESS,
  FRONT_RIGHT_ARTICULATION_MOTOR_ADDRESS,
  MIDDLE_LEFT_ARTICULATION_MOTOR_ADDRESS,
  MIDDLE_RIGHT_ARTICULATION_MOTOR_ADDRESS,
  REAR_LEFT_ARTICULATION_MOTOR_ADDRESS,
  REAR_RIGHT_ARTICULATION_MOTOR_ADDRESS,
  LEFT_CONVEYOR_MOTOR_ADDRESS, 
  RIGHT_CONVEYOR_MOTOR_ADDRESS, 
  WINCH_MOTOR_ADDRESS};
const unsigned char MOTOR_COMMAND[15] = {
  FRONT_LEFT_DRIVE_MOTOR_COMMAND,
  FRONT_RIGHT_DRIVE_MOTOR_COMMAND,
  MIDDLE_LEFT_DRIVE_MOTOR_COMMAND,
  MIDDLE_RIGHT_DRIVE_MOTOR_COMMAND,
  REAR_LEFT_DRIVE_MOTOR_COMMAND,
  REAR_RIGHT_DRIVE_MOTOR_COMMAND,
  FRONT_LEFT_ARTICULATION_MOTOR_COMMAND,
  FRONT_RIGHT_ARTICULATION_MOTOR_COMMAND,
  MIDDLE_LEFT_ARTICULATION_MOTOR_COMMAND,
  MIDDLE_RIGHT_ARTICULATION_MOTOR_COMMAND,
  REAR_LEFT_ARTICULATION_MOTOR_COMMAND,
  REAR_RIGHT_ARTICULATION_MOTOR_COMMAND,
  LEFT_CONVEYOR_MOTOR_COMMAND, 
  RIGHT_CONVEYOR_MOTOR_COMMAND, 
  WINCH_MOTOR_COMMAND};
  const unsigned char MOTOR_FLIPPED[15] = {
  FRONT_LEFT_DRIVE_MOTOR_FLIPPED,
  FRONT_RIGHT_DRIVE_MOTOR_FLIPPED,
  MIDDLE_LEFT_DRIVE_MOTOR_FLIPPED,
  MIDDLE_RIGHT_DRIVE_MOTOR_FLIPPED,
  REAR_LEFT_DRIVE_MOTOR_FLIPPED,
  REAR_RIGHT_DRIVE_MOTOR_FLIPPED,
  FRONT_LEFT_ARTICULATION_MOTOR_FLIPPED,
  FRONT_RIGHT_ARTICULATION_MOTOR_FLIPPED,
  MIDDLE_LEFT_ARTICULATION_MOTOR_FLIPPED,
  MIDDLE_RIGHT_ARTICULATION_MOTOR_FLIPPED,
  REAR_LEFT_ARTICULATION_MOTOR_FLIPPED,
  REAR_RIGHT_ARTICULATION_MOTOR_FLIPPED,
  LEFT_CONVEYOR_MOTOR_FLIPPED, 
  RIGHT_CONVEYOR_MOTOR_FLIPPED, 
  WINCH_MOTOR_FLIPPED};
  
  
  // Motor speed constants
  const unsigned int ARTICULATION_SPEED = 50;
  // The articulation drive speed is 6260/7521 timmes the articulation speed.
  // Use longs for the math so we don't overflow (Arduino int is 16-bit, long is 32-bit).
  //const unsigned int ARTICULATION_DRIVE_SPEED = (int)((50L * 6260L) / 7521L);
  
  // FUNCTIONS

//bool updateTargetWheelStatus(const command2ros::ManualCommand& nmc);
//void continueDriving();
//void driveClockwise(char speed, char motor);
//void driveCounterclockwise(char speed, char motor);
//void stopAllMotors();
//void articulate();
//void updateArticulationValues();


