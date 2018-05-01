/**
 * [Genuino Uno]
 * This is TEST CODE for the stepper motor (the one who lowers the digger down and raises the digger/bucket chain back up)
 * We got this code from:     https://www.dfrobot.com/wiki/index.php/TB6600_Stepper_Motor_Driver_SKU:_DRI0043
 * 
 * currently it only runs one way for x seconds then back the other way for x seconds
 * 
 * topic name/msgType: "Digger"
 * msg.raise; -1 lower, 0 stop, 1 raise
 * 
 * @author: Ryan Kane
 */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <command2ros/Digger.h>
ros::NodeHandle nh;

int rpiDigger = 0;

/**
 * Updates the arduino with the current information from R-Pi, and writes it to our program varables
 */
void messageCb(const command2ros::Digger &msg){
  rpiDigger = msg.raise;
}

ros::Subscriber<command2ros::Digger> sub("Digger", &messageCb); //Subscribe to "MovementCommand" (Msg we get from R-Pi)

//old stuff
//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//char hello[13] = "hello world!";
  

int PUL=7; //define Pulse pin
int DIR=6; //define Direction pin
int ENA=5; //define Enable Pin
void setup() {
  Serial.begin(9600);//57600 for printing to user
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.spinOnce(); //ROS updates comunication
}

int pulseUp = 100;//for going up
int pulseDw = 700;//for going down

void loop() {
  //nh.spinOnce();
  rpiDigger = -1;

  if(rpiDigger == -1){
    //lower Digger
    for (int i=0; i<9600; i++){
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
    for (int i=0; i<9600; i++){
      digitalWrite(DIR,HIGH);
      digitalWrite(ENA,HIGH);
      digitalWrite(PUL,HIGH);
      delayMicroseconds(pulseUp);
      digitalWrite(PUL,LOW);
      delayMicroseconds(pulseUp);
    }
  }
  
//  Serial.print("UP\n");
//  for (int i=0; i<9600; i++)   //Backward 5000 steps
//  {
//    digitalWrite(DIR,HIGH);
//    digitalWrite(ENA,HIGH);
//    digitalWrite(PUL,HIGH);
//    delayMicroseconds(pulseUp);
//    digitalWrite(PUL,LOW);
//    delayMicroseconds(pulseUp);
//  }
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
