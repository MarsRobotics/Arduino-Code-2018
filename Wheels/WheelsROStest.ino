#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <command2ros/ManualCommand.h>
#include "SabertoothDriverROS.h"

//node handle to represent this arduino
ros::NodeHandle sabertoothDriverNode;

//Publisher to print debug statements
std_msgs::String debugMsg;
ros::Publisher pubDebug("sabertoothDebugger", &debugMsg);

// Print error message to "sabertooth_debugger" topic
void print(char* errorMsg){
  debugMsg.data = errorMsg;
  pubDebug.publish(&debugMsg);
}
