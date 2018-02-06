#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <command2ros/ManualCommand.h>
#include "SabertoothDriverROS.h"

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

// Print error message to "sabertoothDebugger" topic
void print(char* errorMsg){
  debugMsg.data = errorMsg;
  pubDebug.publish(&debugMsg);
}