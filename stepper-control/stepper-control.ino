/**
 * [Genuino Uno]
 * 
 * We got the code for running the Stepper from:     
 *      https://www.dfrobot.com/wiki/index.php/TB6600_Stepper_Motor_Driver_SKU:_DRI0043
 * 
 * 
 * msg.raise; -1 lower, 0 stop, 1 raise
 * 
 * @author: Ryan Kane ryanjkane911@yahoo.com
 */

//ROS includes
#include <ros.h>
#include <std_msgs/Int32.h>
#include <command2ros/Digger.h>
ros::NodeHandle nh; //our node handle

//storage vale for saving command from R-Pi
int rpiDigger = 0;

/**
 * Updates the arduino with the current information from R-Pi, and writes it to our program varables
 */
void messageCb(const command2ros::Digger &msg){
  rpiDigger = msg.raise;
}

//Our Subscriber
ros::Subscriber<command2ros::Digger> sub("Digger", &messageCb); //Subscribe to "Digger" (Msg we get from R-Pi)  

//for stepper motors
int PUL=7; //define Pulse pin
int DIR=6; //define Direction pin
int ENA=5; //define Enable Pin


void setup() {
  Serial.begin(9600);//for printing mesages to the user
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.spinOnce(); //ROS updates comunication
}

//pulse rates, for stepper to go Up and Down
int pulseUp = 800;//for going up
int pulseDw = 200;//for going down


int RPiActive = 1;    //If we want the Arduino to listen for commands from the R-Pi set "mode = RPiActive"
int csTesting = 2;    //Arduino will run test code only (if "mode = csTesting" R-Pi will not connect to Arduino)
int mode = RPiActive;  

void loop() {
  if(mode == RPiActive){
    //updates comunication from R-Pi, this IS needed for R-Pi to connect with Arduino
    nh.spinOnce();
      
    if(rpiDigger == -1){
      //lower Digger
      for (int i=0; i<100; i++){
        digitalWrite(DIR,LOW);
        digitalWrite(ENA,HIGH);
        digitalWrite(PUL,HIGH);
        delayMicroseconds(pulseDw);
        digitalWrite(PUL,LOW);
        delayMicroseconds(pulseDw);
      }
    }
    else if(rpiDigger == 1){
      //raise Digger
      for (int i=0; i<100; i++){
        digitalWrite(DIR,HIGH);
        digitalWrite(ENA,HIGH);
        digitalWrite(PUL,HIGH);
        delayMicroseconds(pulseUp);
        digitalWrite(PUL,LOW);
        delayMicroseconds(pulseUp);
      }
    }
  }
  else if (mode == csTesting){
    //Test code, used to try things out, 
    //NOTE: while in "csTesting" mode R-Pi will not connect to Arduino
    
    int tempUp = pulseUp;
    for(int i = 1; i < 30; i++){
      tempUp = pulseUp*i*2;
      Serial.print("UP: "+ (String)tempUp + "\n");
      for (int i=0; i<4800; i++)   //Backward 5000 steps
      {
        digitalWrite(DIR,HIGH);
        digitalWrite(ENA,HIGH);
        digitalWrite(PUL,HIGH);
        delayMicroseconds(tempUp);
        digitalWrite(PUL,LOW);
        delayMicroseconds(tempUp);
      }
    }
    
  }
  
//  Serial.print("DOWN\n");
//  for (int i=0; i<9600; i++)    //Forward 5000 steps
//  {
//    digitalWrite(DIR,LOW);
//    digitalWrite(ENA,HIGH);
//    digitalWrite(PUL,HIGH);
//    delayMicroseconds(pulseDw);
//    digitalWrite(PUL,LOW);
//    delayMicroseconds(pulseDw);
//  }
}
