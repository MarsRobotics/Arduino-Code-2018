//#include <ros.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Int16.h>
//#include <std_msgs/Float32.h>
//#include <command2ros/ManualCommand.h>
//#include <EncoderFL.h>

//EncoderFL efl;

//All proccesses killed, don't ever leave this state.
const int E_STOPPED = -1;
//Not driving any motors
const int STOPPED = 0;
//Articulating the wheels
//const int ARTICULATING = 1;
//Done articulating, drive 
const int START_DRIVING = 2;
//Started articulating, drive
const int IS_DRIVING = 3;
//The state the robot is currently in
int currentStatus = START_DRIVING;

//time robot will drive before stopping
long driveTime = 1000;

int FRONT_LEFT_DRIVE_MOTOR_ID = 0; //TODO
int MOTOR_ADDRESS = 0; //TODO
int MOTOR_COMMAND = 0; //TODO

//The speed 
const double ARTICULATION_DRIVE_SPEED = 6260.0 / 7521.0;
// This corresponds to the "maximum speed" available to the robot.
const int MAX_DRIVE_SPEED = 30;

bool motorInMotion;

// This node handle represents this arduino. 
//ros::NodeHandle sabertoothDriverNode; //TODO maybie?

//==================================================================================================//
void driveAllWheels() {
  //Set wheel speed
  driveClockwise(FRONT_LEFT_DRIVE_MOTOR_ID, MAX_DRIVE_SPEED*-1);
}


void stopOneMotor(int motorID, bool EStop) { //TODO 
  if (EStop) {
    currentStatus = STOPPED;
  } 
  else {
    currentStatus = E_STOPPED;
  }
  //Stop all motors 
  const int speed = 0;
  //for(int motorID = 0; motorID <= 14; ++motorID) {
  driveClockwise(motorID, speed);
  //}
}

//=============================================================================================================//
void driveClockwise(int motorID, int speed){
//  if(0 == speed)
//  {
//    motorInMotion[motorID] = false; 
//  }
//  else
//  {
//    motorInMotion[motorID] = true;
//  }

  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
  // Build the data packet:
  // Get the address and motor command ID from a predefined array.
  unsigned char address = MOTOR_ADDRESS;
  unsigned char command = MOTOR_COMMAND;
  unsigned char checksum = (address + command + ((char)speed)) & 0b01111111;
  // Write the packet.
  Serial3.write(address);
  Serial3.write(command);
  Serial3.write(((char)speed));
  Serial3.write(checksum);
  //TODO: Move the delay time to a constant
  delayMicroseconds(1000); 
}


void setup() {
  // put your setup code here, to run once:
}

void loop() {
//  //updateArticulationValues(); 
//  //We are E-Stopped, don't respond to future commands.
//  if (currentStatus == E_STOPPED) {
//    //TODO: This will never happen right now, may want for competition though
//    return;
//  } 
  if (currentStatus == START_DRIVING) {
    //print("current status: start_driving");
    //Set stop drive time
    driveTime = millis() + 1000000;//(long)wheelTarget.drive_duration*1000;
    //Send drive start command
    driveAllWheels();
    currentStatus = IS_DRIVING;
  }
//  if (currentStatus == IS_DRIVING) {
//    if (millis() >= driveTime) {
//      //print("over time limit: stop driving");
//      //stopArticulationAndDriveMotors();
//    }
//  }

  //pubwheelStatus.publish(&wheelStatus);// current rotation data for each wheel. 
  //Sync with ROS
//  sabertoothDriverNode.spinOnce(); // Check for subscriber update/update timestamp
  //Delay so we don't overload any serial buffers
  for(int i = 0; i < 7; i++){
    delayMicroseconds(15000);
  }

}




