//#include <ros.h>
//#include <std_msgs/String.h>
//#include <std_msgs/Int16.h>
//#include <std_msgs/Float32.h>
//#include <command2ros/ManualCommand.h>
//#include <EncoderFL.h>
//void stopOneMotor(int motorID, bool EStop) { //TODO 

//  //Stop all motors 
//  const int speed = 0;
//  //for(int motorID = 0; motorID <= 14; ++motorID) {
//  driveClockwise(motorID, speed);
//  //}
//}

//=============================================================================================================//
//void driveClockwise(int motorID, int speed){
//  // Packet format: Address Byte, Command Byte, Value Byte, Checksum.
//  // Build the data packet:
//  // Get the address and motor command ID from a predefined array.
////  unsigned char address = MOTOR_ADDRESS;
////  unsigned char command = MOTOR_COMMAND;
////  unsigned char checksum = (address + command + ((char)speed)) & 0b01111111;
//  // Write the packet.
//  Serial1.write(130);//(address)
//  Serial1.write(0);//(command) 0 = 50% foward
//  Serial1.write(64);//(speed) 
//  Serial1.write(66);//(checksum) = (address + command + speed) &(mod) 127 
//  //TODO: Move the delay time to a constant
//  delayMicroseconds(1000); 
//}


void setup() {
  // put your setup code here, to run once:
  Serial1.begin(9600);
  //Serial 1: pins 18-19
  //Serial 2: pins 16-17
  //Serial 3: pins 14-15
}

void loop() {
  unsigned char address = 129;
  unsigned char command = 0;
  int speed2 = 30;    //TODO This is IMPORTANT to have it as an int to char later
  unsigned char checksum = (address + command + (char)speed2) & 0b01111111;
  Serial1.write(address);//(address)
  Serial1.write(command);//(command) 0 = 50% foward
  Serial1.write(speed2);//(speed) 
  Serial1.write(checksum);
  
  
  delayMicroseconds(10000);
  

}




