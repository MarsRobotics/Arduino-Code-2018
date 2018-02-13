////#include <ros.h>
////#include <std_msgs/String.h>
////#include <std_msgs/Int16.h>
////#include <std_msgs/Float32.h>
////#include <command2ros/ManualCommand.h>
////#include <EncoderFL.h>
//
////EncoderFL efl;
//
////All proccesses killed, don't ever leave this state.
//const int E_STOPPED = -1;
////Not driving any motors
//const int STOPPED = 0;
////Articulating the wheels
////const int ARTICULATING = 1;
////Done articulating, drive 
//const int START_DRIVING = 2;
////Started articulating, drive
//const int IS_DRIVING = 3;
////The state the robot is currently in
//int currentStatus = START_DRIVING;
//
////time robot will drive before stopping
//long driveTime = 1000;
//
//int FRONT_LEFT_DRIVE_MOTOR_ID = 0; //TODO
//int MOTOR_ADDRESS = 0; //TODO
//int MOTOR_COMMAND = 0; //TODO
//
////The speed 
//const double ARTICULATION_DRIVE_SPEED = 6260.0 / 7521.0;
//// This corresponds to the "maximum speed" available to the robot.
//const int MAX_DRIVE_SPEED = 30;
//
//bool motorInMotion;
//
//// This node handle represents this arduino. 
////ros::NodeHandle sabertoothDriverNode; //TODO maybie?
//
////==================================================================================================//
//void driveAllWheels() {
//  //Set wheel speed
//  driveClockwise(FRONT_LEFT_DRIVE_MOTOR_ID, MAX_DRIVE_SPEED*-1);
//}
//
//
//void stopOneMotor(int motorID, bool EStop) { //TODO 
//  if (EStop) {
//    currentStatus = STOPPED;
//  } 
//  else {
//    currentStatus = E_STOPPED;
//  }
//  //Stop all motors 
//  const int speed = 0;
//  //for(int motorID = 0; motorID <= 14; ++motorID) {
//  driveClockwise(motorID, speed);
//  //}
//}
//
////=============================================================================================================//
//void driveClockwise(int motorID, int speed){
////  if(0 == speed)
////  {
////    motorInMotion[motorID] = false; 
////  }
////  else
////  {
////    motorInMotion[motorID] = true;
////  }
//
//  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
//  // Build the data packet:
//  // Get the address and motor command ID from a predefined array.
//  unsigned char address = MOTOR_ADDRESS;
//  unsigned char command = MOTOR_COMMAND;
//  unsigned char checksum = (address + command + ((char)speed)) & 0b01111111;
//  // Write the packet.
//  Serial3.write(address);
//  Serial3.write(command);
//  Serial3.write(((char)speed));
//  Serial3.write(checksum);
//  //TODO: Move the delay time to a constant
//  delayMicroseconds(1000); 
//}

//==============================================================================================================
//==============================================================================================================
//==============================================================================================================



//===Program Varables===
int currentTime = 0;
int runTime = 50;
int baseSpeed = 60;

//===Wheel Varables==
int numberOfWheels = 15;
unsigned char wheelID[15] =  {129, 130, 131, 132, 133, 134,         //Drive motors
                              135, 136, 137, 138, 139, 140,         //articulation motors
                              141, 142, 143};                       //Misc motors
boolean wheelDirection[15] = {true, true, true, true, true, true,   //Drive motors
                              true, true, true, true, true, true,   //articulation motors
                              true, true, true};                    //Misc motors
int wheelArticulation[15] =  {0, 0, 0, 0, 0, 0,                     //Drive motors
                              0, 0, 0, 0, 0, 0,                     //articulation motors
                              0, 0, 0};                             //Misc motors
                              
//Serial1 on pins 19 (RX) and 18 (TX), Serial2 on pins 17 (RX) and 16 (TX), Serial3 on pins 15 (RX) and 14 (TX)
int wheelSerial[15] =        {1, 1, 1, 1, 1, 1,
                              1, 1, 1, 1, 1, 1,
                              1, 1, 1};
String wheelName[15] =       {"fr", "mr", "rr", "fl", "ml", "mr",   //Drive motors
                              "fr", "mr", "rr", "fl", "ml", "mr",   //articulation motors
                              "dumper", "digger", "lidarCon"};      //Misc motors


/**
 * runs a motor given motorID, direction, and speed
 * 
 * ID: the wheelID[] position
 * commandInt:  the direction we want to rotate the wheel
 *              0: full foward, 1: full reverse
 * speed2: the speed you want it to rotate
 */
void runMotor(int ID, int commandInt, int speed2){
  unsigned char address = wheelID[ID];
  unsigned char command;
  //reverse the wheels direction if need be
  if(wheelDirection[ID] == true){
    //wheel is facing correct direction
    command = (char)commandInt;
  }
  else{
    //wheel is reversed, reverse the command
    command = (char)(1-commandInt);
  }
  
  unsigned char checksum = (address + command + ((char)speed2)) & 0b01111111;
  // Write to the correct serial packet.
  if(wheelSerial[ID] == 1){
    Serial1.write(address);
    Serial1.write(command);
    Serial1.write(((char)speed2));
    Serial1.write(checksum);
  }
  else if(wheelSerial[ID] == 2){
    Serial2.write(address);
    Serial2.write(command);
    Serial2.write(((char)speed2));
    Serial2.write(checksum);
  }
  else if(wheelSerial[ID] == 3){
    Serial3.write(address);
    Serial3.write(command);
    Serial3.write(((char)speed2));
    Serial3.write(checksum);
  }
}

//Stop all Motors
void fullStop(){
  for(int i = 0; i < numberOfWheels; i++){
    runMotor(i, 0, 0);
  }
}


//run all Drive Motors
void runAllDrive(int command, int speed2){
  for(int i = 0; i < 6; i++){
    runMotor(i, command, speed2);
  }
}

//run all Articulation Motors
void runAllArticulation(int command, int speed2){
  for(int i = 6; i < 12; i++){
    runMotor(i, command, speed2);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
}

void loop() {
  //If we are over our time, stop motor
  if(currentTime > runTime){
    fullStop();
  }
  else{
    //runMotor(0, 0, baseSpeed);
    runAllDrive(0, baseSpeed);
    currentTime++;
    //print("hi");
  }


  
  //Delay so we don't overload any serial buffers
  for(int i = 0; i < 7; i++){
    delayMicroseconds(15000);
  }

  
}




