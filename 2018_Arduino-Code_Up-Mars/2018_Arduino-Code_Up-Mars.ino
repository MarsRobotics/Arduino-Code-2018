/**
 * Code for the [Arduino Mega 2560]         Tools->Board->Arduino Mega/Genuino Mega or Mega 2560
 * 
 * Used to accept a "MovementCommand" from R-Pi (Rasberry Pi), 
 * interperts thous commands to meeningfull motor movements such as driving and dumping
 * 
 * TODO if digging change the speed in reverse for 1 second, ()
 * 
 * @author: Ryan Kane
 */

//====================================================================================================================================================================
// Wheel-Variables, Varables for each individual wheel
//====================================================================================================================================================================
int numberOfWheels = 15;       //0/6  1/7  2/8  3/9  4/10 5/11

//String wheelName[15] =       {"fr-0", "mr-1", "br-2", "fl-3", "ml-4",  "bl-5",  //Drive motors  //"fr" means Fount-Right, ext
//                              "fr-6", "mr-7", "br-8", "fl-9", "ml-10", "bl-11", //articulation motors //"bl" means Rear-Left
//                              "dumper-12", "wench-13", "digger-14"};            //Misc motors

//unsigned char wheelID[15] =  {133, 132, 131, 130, 129, 128,         //Drive motors
//                              133, 132, 131, 130, 129, 128,         //articulation motors
//                              100, 100, 100};                       //Misc motors

//Test (disabled some wheels by seting their ID to Zero)
unsigned char wheelID[15] =  {133, 132, 131, 130, 129, 128,         //Drive motors
                              133, 132, 131, 130, 129, 128,         //articulation motors
                              0, 0, 0};                         //Misc motors

//If mecanical/electrical make a mistake and put a motor on backwards, set Direction to false to reverse rotations
boolean wheelDirection[15] = {false, true, false, true, false, false,   //Drive motors
                              false, true, false, true, true, false, //articulation motors
                              true, true, true};                    //Misc motors

//articulation sensors's true zero relative to our ideal true zero on rover (+ CounterClock, - ClockWise)
const int wheelTrueZero[6] = {-5, 51, -150, -15, -5, 0};

//Serial1 on pins 19 (RX) and 18 (TX), Serial2 on pins 17 (RX) and 16 (TX), Serial3 on pins 15 (RX) and 14 (TX)
//Depending on which wheel type it is, we need to give it a different type of command to move the motor
const int wheelPinType[15] = {1, 1, 1, 1, 1, 1,
                              2, 2, 2, 2, 2, 2,
                              1, 1, 1};

//Speed modifier for wheels, (because wheels turn at diffrent speeds for unknown reasons), so they turn at same speed
double wheelCorectFactor[15] =  {1.13, 1.18, 1.14, 1.04, 1.00, 1.00, //Drive motors
                                 2.00, 1.15, 2.30, 1.15, 1.10, 1.20,  //articulation motors
                                 1.0, 1.0, 1.0};                      //Misc motors

//====================================================================================================================================================================
// Program Variables
//====================================================================================================================================================================

//127 is Max Speed => our motors will not run above this
//15  is Min Speed => any less and motors will not move
int driveSpeed = 50;    //Speed of driveMotors going forward 
int turnSpeed = 30;     //Speed of articulationMotors turning into positions
int turnDriveSpeed = 40; //Speed of driveMotors turning while rover is turning left/right

double turnError = 2;         //The allowed room for error when turning the articulation motors to X angle
double artHelperSpeed = 1.1;  //Speed modifier. Used so the drive wheels turn during wheel articulation smoothly (so the wheels don't drag when steering)
double innerTurnSpeed = 0.8;  //Speed modifier. Used so the inner 2 drive wheels turn slower when the rover is turning in place

//Booleans passed to runMotor to tell it what direction to turn
boolean FORWARD = true;   //Clockwise rotation
boolean BACKWARD = false; //Counter-Clockwise rotation
//These need special ints to control moter rotations
const int FORWARD_DRIVE = 1;   //Clockwise rotation for drive wheels
const int BACKWARD_DRIVE =0;   //Counter-Clockwise rotation for drive wheels
const int FORWARD_ART = 4;     //Clockwise rotation for articulation wheels
const int BACKWARD_ART =5;     //Counter-Clockwise rotation for articulation wheels


//List of "States" the 6 wheels can be facing, 1024 is 360*, 0 is 0* 
int stateStorage[6] =   {768, 256, 256, 256, 256, 768};  //orientation of wheels for the storage/collapsed position, (the position our robot will start RMC at)
int stateTurning[6] =   {419, 768, 605, 605, 768, 419};  //orientation of wheels needed to turn the robot

//int stateDrive[6] =   {256, 768, 768, 768, 768, 256};  //orientation of wheels needed to drive straight-ish
int stateDriveForward[6] =  {256-30, 768+0, 768-10, 768-20, 768+20, 256-15};  //orientation of wheels needed to drive straight->forward
int stateDriveBackward[6] = {256-10, 768+15, 768-10, 768+0, 768+15, 256+0};  //orientation of wheels needed to drive straight->backward

//Plus is C-CW, Minus is CW

int newDegrees[6] = {0,0,0,0,0,0};  //storage value used to pass an array of Ints between methods //(Should be a better way of doing this, but IDK)

//Storage to remember if manual buttons are being pressed, used to detect a change in manual commands from R-Pi
int manualForwardCur = 0;
int manualTurnCur = 0;

//If Arduino is currently paused or is trying to cancel it's current command
boolean ardCancel = false;  //If we have been set the "cancel" command, we stop all, and all new methods not start
int tempSerialID = -999;    //our temporary storage for the last SerialID we got from R-Pi (so we now when we got a fresh message)

//====================================================================================================================================================================
// Digger and Dumper Varables
//====================================================================================================================================================================

//The amount of time and speed the digger and dumper will take to dig and dumping
int diggerTime   = 1000; 
int diggerSpeed = 30;
int diggerDriveSpeed = 30;

int wenchDistance= 1000;
int wenchSpeed  = 30;

int dumperTime  = 1000;
int dumperSpeed = 30;

//====================================================================================================================================================================
// ROS Includes, used to understand passed mesages between the arduino and R-Pi
//====================================================================================================================================================================

//Example I followed to set up ros publisher:   http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World

#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <command2ros/MovementCommand.h>  //message type we get from R-Pi
#include <command2ros/ArduinoMessage.h>   //primary message we send to R-Pi, reports errors, data
#include <command2ros/Feedback.h>         //secondary message we send to R-Pi, simply a String for a Human to read what Arduino is thinking/doing
ros::NodeHandle nh;   //Our Ros Node

//================================================================================================================================================
// Mesage Varables, (rpi)varables are Receved from R-Pi, (ard)varables are feedback Arduino sends to the R-Pi
//================================================================================================================================================

/*
 * R-Pi Varables that Arduion recieved, (commands)
 */
int rpiDriveDistance= 0;      //how far R-Pi wants Arduino to Drive Forward (negative for Drive Backwards)
int rpiTurnDegrees  = 0;      //how far R-Pi wants Arduino to Turn Right    (negative for Turn Left)
boolean rpiDigger = false;    //true, to dig
boolean rpiDumper = false;    //true, to dump
boolean rpiPackIn = false;    //true, to pack in our wheels (too the starting/storage state)
boolean rpiPause =  false;    //true, to pause, stop all motors and acept no new command till [rpiPause =  false;] is sent
boolean rpiEStop =  false;    //true, to Stop Forever
boolean rpiCancel = false;    //true, to stop and cancel the current command
boolean rpiManual = false;    //true, to accept the "manual" commands
int rpiManualDrive = 0;       //(only activates if rpiManual == true;) 1: go Forward, -1: go backward, 0: stop going forward/backward
int rpiManualTurn = 0;        //(only activates if rpiManual == true;) 1: turn Right, -1: turn Left,   0: stop going right/left
int rpiSerialID = -1;         //what message number R-Pi is on (so Arduino knows this is a fresh message)

/*
 * Ardino Varables, Sent to R-Pi (feedback)
 */
boolean ardReady =  true;     //This should be true if we are ready for a new command
int ardSerialID = -1;         //What RPi msg Number arduino is currently working on
int ardMsgID = 0;             //What msg number we are on (so R-Pi knows when a fresh mesage is sent)
String ardProgress = "";      //String that reports the Arduino's current state (for human user)

//Types of errors Arduino can experence, (NOT CURENTLY IN USE)
boolean ardErrorDriving = false;    //experenced error while driving forward or backwards
boolean ardErrorDigging = false;    //experenced error while digging
boolean ardErrorDumping = false;    //experenced error while dumping
boolean ardErrorTurning = false;    //experenced error while turing left or right

//================================================================================================================================================
// Message Command
//================================================================================================================================================

/**
 * Updates the arduino with the current information from R-Pi, and writes it to our program varables
 */
void messageCb(const command2ros::MovementCommand &msg){
  //Information from our msg extracted and copied to our varables
  rpiDriveDistance= (int)(msg.driveDist); //Int32
  rpiTurnDegrees  = (int)(msg.turn);      //Int32
  rpiDigger = msg.dig;      //bool //Int32 -1: backward, 0: stop, 1: forward
  rpiDumper = msg.dump;     //bool
  rpiPackIn = msg.packin;   //bool
  rpiEStop =  msg.eStop;    //bool
  rpiPause =  msg.pause;    //bool
  rpiCancel = msg.cancel;   //bool
  rpiSerialID = msg.serialID;  //Int32
  rpiManual   = msg.manual;    //bool
  rpiManualDrive = msg.manualDrive; //Int32
  rpiManualTurn = msg.manualTurn;   //Int32
}


//For comunicating with Ros subscibers and pubishers have to be bellow "messageCb", and above "updateMessage"
command2ros::ArduinoMessage ardMsgCb;
command2ros::Feedback ardHumanMsgCb;
ros::Subscriber<command2ros::MovementCommand> sub("MovementCommand", &messageCb); //Subscribe to "MovementCommand" (Msg we get from R-Pi)
ros::Publisher ard_msg ("ArduinoFeedback", &ardMsgCb); //Publish to "ArduinoMessage" (Msg we send to R-Pi) 
ros::Publisher ard_msg_hr ("HumanReadable", &ardHumanMsgCb); //Publish to "HumanReadable" (Another Msg we send to R-Pi) <= only containes a String

/**
 * Sends all Arduino's data to R-Pi.
 * uses 2 diffrent messages: "ArduinoMessage" -> All Data
 *                           "HumanReadable"  -> Single String for Human User to read
 */
void updateMessage(){
  //sending all our data to R-Pi useing "ArduinoMessage"
  ardMsgCb.ready        = ardReady;           //bool
  ardMsgCb.serialID     = ardSerialID;        //Int32
  ardMsgCb.errorDriving = ardErrorDriving;    //bool
  ardMsgCb.errorDigging = ardErrorDigging;    //bool
  ardMsgCb.errorDumping = ardErrorDumping;    //bool
  ardMsgCb.errorTurning = ardErrorTurning;    //bool
  ardMsgCb.messageID    = ardMsgID;           //Int32
  ardMsgCb.progress = ardProgress.c_str();    //char*

  //sending our String to R-Pi/User "HumanReadable"
  ardHumanMsgCb.msg = ardProgress.c_str();    //char* 

  //publish both our feed back messages
  ard_msg.publish(&ardMsgCb);
  ard_msg_hr.publish(&ardHumanMsgCb);         //char*
  //update our node with the new published mesages
  nh.spinOnce();
}

//================================================================================================================================================
// Setup and Main Loop, "Setup" is called once at start, "Loop" loops infinitly
//================================================================================================================================================
/**
 * Special Arduino Methode: Runs once at start.
 */
void setup() {
  //Begin all Serial Writers
  Serial.begin(9600);//57600
  Serial1.begin(9600);
  Serial2.begin(9600);//TODO may not be needed
  Serial3.begin(9600);//TODO may not be needed
  
  //initialize our Ros Node, and Subscibe to R-Pi's Message
  //nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  //"advertise" our 2 mesages
  nh.advertise(ard_msg);
  nh.advertise(ard_msg_hr);
  //checks to make sure all articultion sensors giving an understanable reading
//  systemCheck();
  msg2user("\n===Ready-To-Start===\n");
  nh.spinOnce(); //ROS updates comunication
}

int RPiActive = 1;
int csTesting = 2;
int mecTesting = 3;
int mode = csTesting;// csTesting; //if(mode == csTesting){we will use Serial.print to print msg2user}

/*
 * Special Arduino Methode: Main loop, loops infinitly.
 */
void loop() {
  if(mode == RPiActive){
    //This should be our real looping method call, (can be disabled for testing)
    //tempSerialID = rpiSerialID;
    nh.spinOnce();
    
//    //Finds the newest messge from R-Pi
//    while(tempSerialID != rpiSerialID){
//      tempSerialID = rpiSerialID;
//      nh.spinOnce(); //ROS updates comunication
//      simpleDelay();
//    }

    //If we have a new rpiSerialID, we have a fresh mesage. So we have to execute the new command
    if(ardSerialID != rpiSerialID){
      ardSerialID = rpiSerialID;  //Update new SerialID
      
      //Only executes one command, per new rpiSerialID (prevents potential conflicts)
      if(rpiEStop){
        commandEStop(rpiEStop);               //permently stop rover, and disables Arduino
      }else if(rpiPause/* != ardPause*/){
        commandPause(rpiPause);               //Stops motors and waits till rpiPause==false
      }else if(rpiDriveDistance != 0){
        commandDriveForward(rpiDriveDistance);//Automated drive forward
      }else if(rpiTurnDegrees != 0){
        commandTurnRover(rpiTurnDegrees);     //Automated turn Rover X distance
      }else if(rpiDigger){
        commandDigger(rpiDigger);             //Automated run Digger X distance
      }else if(rpiDumper){
        commandDumper(rpiDumper);             //Automated run Dumper
      }else if(rpiPackIn){
        commandPackIn(rpiPackIn);             //Automated "PackIn" to our starting state
      }else if(rpiCancel){
        //R-Pi asked for cancel, but where not running a command
        manualTurnCur = 0;  
        manualForwardCur = 0;
        msg2user("/t <<<Command Canceled>>> (Outside Known Method=>Just Stoping Motors)\n");
        stopAllMotor();
      }

      //Manual Commands
      else if(rpiManual && rpiManualDrive != manualForwardCur){
        commandManualForward(rpiManualDrive); //Manual Command, will remain driving until mesage rpiManualDrive=0 is given
      }else if(rpiManual && rpiManualTurn != manualTurnCur){
        commandManualTurn(-rpiManualTurn);     //Manual Command, will remain turning until mesage rpiManualTurn=0 is given 
      }
      
      //If we are outside methods we no longer need to cancel current command
      ardCancel = false;
      
      //Delay so we don't overload any serial buffers
      simpleDelay();
    }
  }

  else if (mode == csTesting){
    ardSerialID = rpiSerialID;
    //for testing
    printArticulation();
    commandPackIn(true);
//    setAllArticulation(stateDriveForward);

//    setAllArticulation(stateStorage); //stateTurning    stateStorage    stateDriveForward
//    setAllArticulation(stateTurning);
//    setAllArticulation(stateDriveForward);
//    setAllArticulation(stateDriveForward);

//    commandPackIn(true);
//    commandTurnRover(200);
//    chainDriveForward(70);
//    commandDriveForward(300);
//    commandDriveForward(-300);
//    driveForward(100);

//    driveOneForward(5, 100,40);
//    driveOneForward(0, 100,40);
//
    testArticulation(stateDriveForward);
//    testArticulation(stateTurning);
//    testArticulation(stateStorage);

//    runMotor(6, BACKWARD, 30);
//    delay(700); //1000 is a small turn
//    stopAllMotor();

//    runMotor(8, FORWARD, 30); //BACKWARD(CC)   FORWARD(CW) 
//    for(int i = 6;i<12; i++){
//      runMotor(i, FORWARD, 50);
//      delay(1000); //1000 is a small turn
//      stopAllMotor();
//  
//      runMotor(i, BACKWARD, 50);
//      delay(1000); //1000 is a small turn
//      stopAllMotor();
//    }
    

    /*
     * int stateStorage[6] =   {768, 256, 256, 256, 768, 768};  //orientation of wheels for the storage/collapsed position, (the position our robot will start RMC at)
     * int stateForward[6] =   {256, 768, 768, 768, 256, 256};  //orientation of wheels needed to drive straight/forward
     * int stateTurning[6] =   {384, 768, 640, 640, 256, 384};  //orientation of wheels needed to turn the robot
     */

    /*
     * List of "States" the 6 wheels can be facing, 1024 is 360*, 0 is 0* 
     * FORWARDS = ClockWise
     * FR(A3-133), MR(A4-132), BR(A5-131), FL(A0-128), ML(A1-129), BL(A2-130)
     */
    
    printArticulation();
    msg2user("test done\n");
    eternalSleep(); //stopAllMotor and sleep forever
  }//end test code
  
  else if (mode == mecTesting){
    if(systemCheck()){
      //if we are facing forward, go to storage
      setAllArticulation(stateStorage);
    }
    else{
      setAllArticulation(stateDriveForward); 
    }
  }
  else /*if (mode == justStop)*/{
    eternalSleep(); //stopAllMotor and sleep forever
  }
}

//void loop() {
//  nh.spinOnce();
//  delay(1);
//}

//====================================================================================================================================================================
// Print Methods, these methods are used for testing and gathering data, (Not called by R-Pi)
//====================================================================================================================================================================

/**
 * Prints the commands R-Pi sent to the Arduino
 */
void printCommandLines(){
  String commandsCur = "Commands: drive: " + (String)(rpiDriveDistance) +
                      " Turn: " + (String)(rpiTurnDegrees) +
                      " Dig: " + (String)(rpiDigger) +
                      " Dump: " + (String)(rpiDumper) +
                      " PackIn: " + (String)(rpiPackIn) +
                      " EStop: " + (String)(rpiEStop) +
                      " Pause: " + (String)(rpiPause) + "\n";
  msg2user(commandsCur);
}

/**
 * Prints the current Articulation reads from all 6 articulation sensors
 */
void printArticulation(){
  String articulation = "Relative to wheelTrueZero:   FR: "+
                           (String)(analogRead(A3)-wheelTrueZero[0])+
                ",\t MR: "+(String)(analogRead(A4)-wheelTrueZero[1])+
                ",\t BR: "+(String)(analogRead(A5)-wheelTrueZero[2])+
            ",\t\t\t FL: "+(String)(analogRead(A0)-wheelTrueZero[3])+
                ",\t ML: "+(String)(analogRead(A1)-wheelTrueZero[4])+
                ",\t BL: "+(String)(analogRead(A2)-wheelTrueZero[5])+ "\n";

  //sends the final msg
  msg2user(articulation);
}

/**
 * prints the current articulation and prints how far it is from a specific stateCur[6]
 */
void testArticulation(int stateCur[]){
  //prints the basic data the sensors are giving
  String rawData= "Raw Data given by sensors:   FR: "+
                          (String)(analogRead(A3))+
               ",\t MR: "+(String)(analogRead(A4))+  //TODO<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
               ",\t BR: "+(String)(analogRead(A5))+
           ",\t\t\t FL: "+(String)(analogRead(A0))+
               ",\t ML: "+(String)(analogRead(A1))+
               ",\t BL: "+(String)(analogRead(A2))+ "\n";
  msg2user(rawData);

  //Prints the distances each wheel needs to go (assuming the phisical wheels are pointing in "sateCur[]")
  String dis2Correct = "Dis to correct Articulation: FR: "+
                            (String)(analogRead(A3)-wheelTrueZero[0]-stateCur[0])+
               ",\t MR:   "+(String)(analogRead(A4)-wheelTrueZero[1]-stateCur[1])+
               ",\t BR:   "+(String)(analogRead(A5)-wheelTrueZero[2]-stateCur[2])+
           ",\t\t\t FL:   "+(String)(analogRead(A0)-wheelTrueZero[3]-stateCur[3])+
               ",\t ML:   "+(String)(analogRead(A1)-wheelTrueZero[4]-stateCur[4])+
               ",\t BL:   "+(String)(analogRead(A2)-wheelTrueZero[5]-stateCur[5])+ "\n";
  msg2user(dis2Correct);
  

  String newTrueZero = "\n[New] const int wheelTrueZero[6] = {"+
                  (String)(analogRead(A3)-stateCur[0])+", "+
                  (String)(analogRead(A4)-stateCur[1])+", "+
                  (String)(analogRead(A5)-stateCur[2])+", "+
                  (String)(analogRead(A0)-stateCur[3])+", "+
                  (String)(analogRead(A1)-stateCur[4])+", "+
                  (String)(analogRead(A2)-stateCur[5])+"};\n\n";
  msg2user(newTrueZero);
}

/**
 * Tests the system (specificaly the articulation sensors) to see if the rover is giving correct readings for a known state
 * known states: stateTurning, stateStorage, stateForward 
 *    ^^^ we test all these states to see if our readings match any of them, else prints error
 * @return: if the wheels are in the forward state, return true, else return false    
 */
bool systemCheck(){
  msg2user("===Starting System Check====\nChecking Articulation Sensors:\n");
  //prints the basic data the sensors are giving
  String rawData= "Raw Data given by sensors:   FR: "+
                          (String)(analogRead(A3))+
               ",\t MR: "+(String)(analogRead(A4))+
               ",\t BR: "+(String)(analogRead(A5))+
           ",\t\t\t FL: "+(String)(analogRead(A0))+
               ",\t ML: "+(String)(analogRead(A1))+
               ",\t BL: "+(String)(analogRead(A2))+ "\n";
  msg2user(rawData);

  int testTurn = testStateValid(stateTurning);
  int testStore= testStateValid(stateStorage);
  int testForwd= testStateValid(stateDriveForward);
  int testBackwd=testStateValid(stateDriveBackward);

  msg2user("[stateTurning] margin of error: "+ (String)testTurn +"\n");
  msg2user("[stateStorage] margin of error: "+ (String)testStore +"\n");
  msg2user("[stateDriveForward]  margin of error: "+ (String)testForwd +"\n");
  msg2user("[stateDriveBackward] margin of error: "+ (String)testBackwd +"\n");

  //finds if any state is within a known State
  if(testTurn < 100){
    msg2user("Articulation Test Passed for: [stateTurning]\n\n");
  }
  else if (testStore < 100){
    msg2user("Articulation Test Passed for: [stateStorage]\n\n");
  }
  else if (testForwd < 100){
    msg2user("Articulation Test Passed for: [stateDriveForward]\n\n");
    return true;
  }
  else if (testBackwd < 100){
    msg2user("Articulation Test Passed for: [stateDriveBackward]\n\n");
    return true;
  }
  else{
    msg2user("ERROR: Articulation Test <FAILED> no known articulation sensed\n\tOne or more wheels giving unexpected articulation readings\n\n");
  }
  //If wheels are not in the forward state, return false
  return false;
}

/**
 * Tests the validity of a Test State compaired what our Current Articulation sensors are reading
 * 
 * @return: the biggest dffrence between our testState and what we are reading
 */
int testStateValid(int testState[6]){
  int biggestOffSet = -1;
  
  //checks all wheel's sensors
  for(int i = 0; i < 6; i++){
    int curOffSet = abs(articulationRead(i)-wheelTrueZero[i]-testState[i]);
    //if our current wheel is the farthest from our testState, save it
    if(curOffSet > biggestOffSet){
      biggestOffSet = curOffSet;
    }
  }

  return biggestOffSet;
}

//====================================================================================================================================================================
// command Methods, these are the only ones the R-Pi is expected to call
//====================================================================================================================================================================

/**
 * Sets the Rover's wheels to point straight and drives FORWARD
 * 
 * @param distance: Distance we want to drive FORWARD (enter negitive number for backwards)
 */
void commandDriveForward(int distance){
  //if our value is zero our R-Pi didn't actualy want to call this method, also we want to turn the rover first
  if(distance == 0 || rpiPause){return;} 
  ardSerialID = rpiSerialID;  //Update new SerialID
  ardReady = false;
  
  msg2user("C#" + (String)ardSerialID + ", Driving Rover: "+ (String)distance +"\n");

  if(distance > 0){
    setAllArticulation(stateDriveForward);
    setAllArticulation(stateDriveForward); //double check to correct any jittering or wigling 
  }
  else{
    setAllArticulation(stateDriveBackward);
    setAllArticulation(stateDriveBackward); //double check to correct any jittering or wigling 
  }
  chainDriveForward(distance);

  //Tells R-Pi we are ready
  ardReady = true;
  msg2user("\tDone\n");
}

/**
 * Sets the Rover's wheels to point in a circle and Rotates Rover in place
 * 
 * @param degrees: Degrees we want to turn Rover (enter negitive number for Left)
 */
void commandTurnRover(int degrees){
  //if our value is zero our R-Pi didn't actualy want to call this method
  if(degrees == 0 || rpiPause){return;}
  ardSerialID = rpiSerialID;  //Update new SerialID
  ardReady = false;
  
  msg2user("C#" + (String)ardSerialID + ", Turning Rover: "+ (String)degrees +"\n");
  
  setAllArticulation(stateTurning);
  setAllArticulation(stateTurning);//double check to correct any jittering or wigling
  turnRover(-degrees);

  //Tells R-Pi we are ready
  ardReady = true;
  msg2user("\tDone\n");
}

/**
 * Cause the rover to lower wench and runs the digger for "diggerTime"
 * 
 * @param startDigger: true if we want to dig now
 */
void commandDigger(boolean startDigger){
  //If digger in is false we didn't actualy want to call this method
  if(startDigger == false || rpiPause){ return;}
  ardReady = false;
  ardSerialID = rpiSerialID;  //Update new SerialID
  
  msg2user("C#" + (String)ardSerialID + ", Digging\n");

  //make sure wheels will not block the digger
  setAllArticulation(stateDriveForward);
  setAllArticulation(stateDriveForward);
  //move wench down, run digger, move wench back up
  driveOneForward(13, wenchDistance, wenchSpeed); //move wench down into dirt
  
  //drive forward while we dig down
  activeDriveForward(FORWARD, diggerDriveSpeed);
  driveOneForward(12, diggerTime, diggerSpeed);   //run digger
  stopAllMotor(); //stops all digging and driving motors
  
  driveOneForward(13, -wenchDistance, wenchSpeed);//move wench back up
  
  ardReady = true;
  msg2user("\tDone Digging\n");
}

/**
 * Cause the rover run the dumper for "dumperTime"
 * 
 * @param startDumper: true if we want to dump right now
 */
void commandDumper(boolean startDumper){
  //If dumper in is false we didn't actualy want to call this method
  if(startDumper == false || rpiPause){ return;}
  ardReady = false;
  ardSerialID = rpiSerialID;  //Update new SerialID
  
  msg2user("C#" + (String)ardSerialID + ", Dumping\n");

  //run the dumper for dumperTime
  driveOneForward(14, dumperTime, dumperSpeed); //run dumper
  
  ardReady = true;
  msg2user("\tDone Dumping\n");
}

/**
 * Sets the Rover's wheels to the resting/storage position
 * 
 * @param packIn: true, if we actualy want to pack our wheels in
 *                false, if we don't actualy want to pack wheels in
 */
void commandPackIn(boolean packIn){ 
  //If pack in is false we didn't actualy want to call this method
  if(packIn == false || rpiPause){ return;}
  ardReady = false;
  ardSerialID = rpiSerialID;  //Update new SerialID
  
  msg2user("C#" + (String)ardSerialID + ", Packing In\n");
  
  setAllArticulation(stateStorage);
  setAllArticulation(stateStorage);//double check to correct any jittering or wigling
  
  ardReady = true;
  msg2user("\tDone\n");
}

/**
 * If we need to Pause use this command, Stops motors, banns new commands, and causes delays to last forever, untill un-paused
 * 
 * @param newPause: the new Pause, true for stop, false for resume
 *     => true:    Stop all motors and ban motors from turning
 *     => false:   Allow motors to move again (unPaused)
 */
void commandPause(boolean newPause){
  if(newPause){
    //if we need to pause, stop all, wait for unpause
    ardSerialID = rpiSerialID;  //Update new SerialID
    
    msg2user("C#" + (String)ardSerialID + ", Paused, stopping all motors\n");
    
    //we need to Pause: stop motors from spining
    stopAllMotor();
  }
  else{
    //we don't want to pause
    return;
  }

  //while we are paused, stay in while loop
  while(rpiPause){
    //while arduino is "paused", check for rpiPaused to be changed
    delay(5);
    nh.spinOnce();
    
    if(rpiEStop){
      ardSerialID = rpiSerialID;
      //If we need to emergency stop, EStop
      commandEStop(rpiEStop);
    }

    if(rpiCancel == true){
      //let the whole program know we wish to cancel
      msg2user("/t <<<Command Canceled>>> \n");
      stopAllMotor();
      ardCancel = true;
    }
    
    //if rpiPause changes, re-allow motors to move
    if(rpiPause == false){
      ardSerialID = rpiSerialID;
      msg2user("C#" + (String)ardSerialID + ", Pause Ended\n");
      return;
    }
  } 
}

/**
 * Irreversably stop all motors and sleep forever
 */
void commandEStop(boolean tempEStop){
  if(tempEStop){
    ardSerialID = rpiSerialID;
    msg2user("C#" + (String)ardSerialID + ", EStop\n");
    ardReady = false;
    //Stops all Motors and sleeps forever
    eternalSleep();
  }
}


//====================================================================================================================================================================
// Manual command Methods, these are special commands only the User sitting in a chair controlling the R-Pi is expected to call
//====================================================================================================================================================================

/**
 * Sets the rover's wheels facing straight and dirves forward untill the commandManualForward gets a Zero passed into it
 * Used for a manual user keyboard, the user would be pressing(and holding) a button to make the value 1 (or -1), 
 *      and then when the user releases the button => rover stops driving
 * 
 * @param direction: the direction we want the rover to drive
 *      1: Forward
 *      0: Stop(if driving)
 *     -1: Backward
 */
void commandManualForward(int direction){
  //if this is the same command as before, do nothing
  if(direction == manualForwardCur){return;}
  ardSerialID = rpiSerialID;
  ardReady = false;
  
  //if we want to go forward and are not already going forward
  if(direction == 1){
    msg2user("C#" + (String)ardSerialID + ", MANUAL: Drive Forward\n");
    //get the Rover's wheels facing straight
    setAllArticulation(stateDriveForward);
    setAllArticulation(stateDriveForward);
    manualForwardCur = 1;
    //drive forward
    activeDriveForward(FORWARD, driveSpeed);
  } 
  else if(direction == -1){
    msg2user("C#" + (String)ardSerialID + ", MANUAL: Drive Backward\n");
    //get the Rover's wheels facing straight
    setAllArticulation(stateDriveBackward);
    setAllArticulation(stateDriveBackward);
    manualForwardCur = -1;
    //drive backward
    activeDriveForward(BACKWARD, driveSpeed);
  }
  else if(direction == 0){
    manualForwardCur = 0;
    //we want to stop the motors now that we aren't holding the forward button
    stopAllMotor();
    
    //Tells R-Pi we are ready
    ardReady = true;
    msg2user("\t=>Drive Stop\n");
  }
}

/**
 * Sets the rover's wheels to turn and turns untill the commandManualTurn gets a Zero passed into it
 * Used for a manual user keyboard, the user would be pressing(and holding) a button to make the value 1 (or -1), 
 *      and then when the user releases the button => rover stops turning
 * 
 * @param direction: the direction we want the rover to turn
 *      1: ClockWise (Left)
 *      0: Stop(if turning)
 *     -1: Counter-ClockWise (Right)
 */
void commandManualTurn(int direction){
  //if this is the same command as before, do nothing
  if(direction == manualTurnCur){return;}
  ardSerialID = rpiSerialID;
  ardReady = false;

  //if we want to turn ClockWise and are not already turning ClockWise
  if(direction == 1){
    msg2user("C#" + (String)ardSerialID + ", MANUAL: Turn Left\n");
    //get the Rover's wheels into turning position
    setAllArticulation(stateTurning);
    manualTurnCur = 1;
    //turn ClockWise
    activeTurn(FORWARD, turnSpeed);
  } 
  else if(direction == -1){
    msg2user("C#" + (String)ardSerialID + ", MANUAL: Turn Right\n");
    //get the Rover's wheels into turning position
    setAllArticulation(stateTurning);
    manualTurnCur = -1;
    //turn Counter Clock
    activeTurn(BACKWARD, turnSpeed);
  }
  else if(direction == 0){
    manualTurnCur = 0; 
    //we want to stop the motors now that we aren't holding the forward button
    stopAllMotor();
    //Tells R-Pi we are ready
    ardReady = true;
    msg2user("/t=>Turn Stop\n");
  }
}

//====================================================================================================================================================================
// Drive Methods, used to allow Drive commands drive Rover correct distances
//====================================================================================================================================================================
/**
 * Drive forward X distance, (does not reposition wheels), 
 * and if a new drive forward command is given progran will start new drive forward command without akwardly stoping between them
 * #Support method for commandDriveForward
 * 
 * @param distance: target distance we want to rover to travel
 */
void chainDriveForward(int distance){
  if(ardCancel == true){return;}
  //driveForward at least once
  driveForward(distance);
  while((ardSerialID != rpiSerialID) && (rpiDriveDistance != 0)){
    ardSerialID = rpiSerialID;
    msg2user("/nC#" + (String)ardSerialID + ", DriveForward Override to: ["+ (String)rpiDriveDistance + "]/n");
    driveForward(rpiDriveDistance);
  }
  stopAllDrive(); //set all drive motor's speed to zero (probably reduntant, but better safe then sorry)
}

/**
 * Drive forward X distance, (does not reposition wheels)
 * if distance is negative => go backwards
 * #Support method for chainDriveForward
 * 
 * @param distance: target distance we want to rover to travel
 */
void driveForward(int distance){
  if(ardCancel == true){return;}
  //float distanceTraveled = 0;
  int direction = FORWARD;
  //if distance is negative => go backwards
  if(distance < 0){
    direction = BACKWARD;
  }
  activeDriveForward(direction, driveSpeed);

  //find what time we need to stop at if we want to go X distance (distance = speed*time => time = distance/speed)
  long endTime = millis() + (abs(distance)/driveSpeed)*1000.0;

  //wait till target end time, and constantly check for interupting commands
  while(millis() < endTime){
    //delay so we don't run the while loop into an overflow
    simpleDelay();
    
    //if we need to pause, pause
    long pausedTime = checkForInterupt();
    if(ardCancel == true){return;}
    
    //if we have exited pause, resume motors update clock
    if(pausedTime > 0){
      //resume motor movement (paused stoped it)
      activeDriveForward(direction, driveSpeed);
      //add time lost in pause to new target endTime
      endTime = endTime + pausedTime;
    }

    //Sees if Rpi wants to contine with another drive command (checkForInterupt already updated rpi varables)
    if((ardSerialID != rpiSerialID) && (rpiDriveDistance != 0)){ 
      //If we have a new drive forward command, drop current and start anew with new drive distance
      return;
    }
    
    //Keeps track of how far we have traveled, based off time
    //distanceTraveled = distanceTraveled + driveSpeed/10.0;
  }

  stopAllDrive(); //set all drive motor's speed to zero
}

/**
 * actualy runs all 6 Drive Motors, 3 right wheels one way, 3 left wheels the other to go forward
 * #Support method for driveForward
 * 
 * @param direction:  FORWARD(clockwise) or BACKWARD(counter-clock)
 * @param speed:      The speed you want all 6 drive motors to turn
 */
void activeDriveForward(int direction, int speed){
  if(ardCancel == true){return;}
  msg2user("act-Drive, ");
  //set right motors to forward
  for(int i = 0; i < 3; i++){
    runMotor(i, direction, speed);
  }
  //set left motors to backward
  for(int i = 3; i < 6; i++){
    runMotor(i, (1-direction), speed);
  }
}

/*
 * Stops all Drive Motors (sets their speed to zero)
 * #Support method for driveFORWARD
 */
void stopAllDrive(){
  msg2user("stoped.\n");
  activeDriveForward(0, 0);
}

//====================================================================================================================================================================
// Turn Methods, used to allow Rover to turn itself the correct distance
//====================================================================================================================================================================

/**
 * Turn Rover Right X degrees
 * if degrees is negative => go backwards
 * #Support Method for commandTurnRover
 * 
 * @param:  degrees, target degrees we want to rover to turn
 */
void turnRover(int degrees){
  if(ardCancel == true){return;}
  int direction = FORWARD;
  //if distance is negative => go backwards(left)
  if(degrees < 0){
    direction = BACKWARD;
  }
  activeTurn(direction, turnDriveSpeed);
  
  //find what time we need to stop at if we want to go X distance (distance = speed*time => time = distance/speed)
  long endTime = millis() + (abs(degrees)/turnDriveSpeed)*1000.0;

  //wait till target end time, and constantly check for interupting commands
  while(millis() < endTime){
    //delay so we don't run the while loop into an overflow
    simpleDelay();
    
    //if we need to pause, pause
    long pausedTime = checkForInterupt();
    if(ardCancel == true){return;}
    
    //we have exited pause, resume motors update clock
    if(pausedTime > 0){
      //resume motor movement (paused stoped it)
      activeTurn(direction, turnDriveSpeed);
      //add time lost in pause to new target endTime
      endTime = endTime + pausedTime;
    }
  }
  
  stopAllDrive(); //set all drive motor's speed to zero
}

/**
 * actualy runs all Drive Motors to turn in place, (assuming rover is in stateTurning)
 * "inner wheels" of the rover turn slower because they are closer to Center of Mass and don't have to/shouldn't go as far 
 * #Support Method for turnRover
 * 
 * @param direction:  FORWARD(clockwise) or BACKWARD(counter-clock)
 * @param speed:      The speed you want all 6 drive motors to turn
 */
void activeTurn(int direction, int speed){
  if(ardCancel == true){return;}
  msg2user("act-Turn, ");
  runMotor(0, direction, speed);
  runMotor(1, direction, speed*innerTurnSpeed);
  runMotor(2, direction, speed);
  
  runMotor(3, direction, speed);
  runMotor(4, direction, speed*innerTurnSpeed);
  runMotor(5, direction, speed);
}

//====================================================================================================================================================================
// Digger/Dumper Methods, used to allow Rover to Dig and Dump correct measurments
//====================================================================================================================================================================

/**
 * drives one wheel forward to a given distance (Safe to run)
 * #Support Method for commandDigger and commandDumper
 * 
 * @param ID:       the id of the drive wheel we want to move
 * @param distance: the distance we want it to travel
 */
void driveOneForward(int ID, int distance, int speed){
  if(ardCancel == true){return;}
  int direction = FORWARD;
  //if distance is negative => go backwards
  if(distance < 0){
    direction = BACKWARD;
  }
  
  msg2user("\turn motor: " + (String)ID + " distance:" + (String)distance + ", ");
  runMotor(ID, direction, speed);
  
  //find what time we need to stop at if we want to go X distance (distance = speed*time => time = distance/speed)
  long endTime = millis() + (abs(distance)/speed)*1000.0;

  //wait till target end time, and constantly check for interupting commands
  while(millis() < endTime){
    //delay so we don't run the while loop into an overflow
    simpleDelay();
    
    //if we need to pause, pause
    long pausedTime = checkForInterupt();
    if(ardCancel == true){return;}
    
    //we have exited pause, resume motors update clock
    if(pausedTime > 0){
      //resume motor movement (paused stoped it)
      runMotor(ID, direction, speed);
      //add time lost in pause to new target endTime
      endTime = endTime + pausedTime;
    }
  }
  stopMotor(ID); //set this motor's speed to zero
  msg2user("stoped.\n");
}

//====================================================================================================================================================================
// Articulation Methods, used to allow Rover's articuation wheels to point in specific directions
//====================================================================================================================================================================

/**
 * Sets all 6 articulation wheels to point in the direction of a given array of [6] ints
 * Stops the wheels once they have reached their individual target "degrees"
 * 
 * @param degrees[6]: the new directions all 6 articulation motors will be set to point at
 */
void setAllArticulation(int degrees[6]){
  if(ardCancel == true){return;}
  boolean done = false;
  boolean isFinished = false;
  
  //Stops all motors, setAllArticulation expects the rover to be stationary
  stopAllMotor();
  
  if(degrees == stateStorage){
    msg2user("\tarticulating->stateStorage, ");
  }else if(degrees == stateDriveForward){
    msg2user("\tarticulating->stateDriveForward, ");
  }else if(degrees == stateDriveBackward){
    msg2user("\tarticulating->stateDriveBackward, ");
  }else if(degrees == stateTurning){
    msg2user("\tarticulating->stateTurning, ");
  }else{
    msg2user("\tarticulating->customState, ");
  }
  
  //update "newDegrees" array with adjusted values
  adjustForWheelOffset(degrees);

  boolean wheelDone[6] = {false, false, false, false, false, false};
  
  //while we are not done turning all motors
  while(done == false){
    //if we encounter no unfinished turning wheels we remain "done"
    done = true;

    //If we need to stop, pause, or eStop do it
    checkForInterupt();
    if(ardCancel == true){return;}
    
    //go through all 6 articulation motors
    for(int i = 6; i<12; i++){
      //sets the articulation of each motor to turn towards the correct degree
      if(wheelDone[i-6] == false){
        isFinished = setMotorArticulation(i, newDegrees[i-6]);
        
        //if ANY motor isn't finished we are not done
        if(isFinished == false){
          done = false;
        }
        else{
          wheelDone[i-6] = true;
        }
      }
    }
  }
  msg2user("stoped.\n");
}

/**
 * Sets the direction an articulation motor to turn towards a particular direction/degree
 * <<<WARNING>>> <<<This is NOT a call and forget method!!!>>>                             <<<WARNING>>>
 * <<<WARNING>>> <<<The method does NOT loop by itself, it will not stop started motor>>>  <<<WARNING>>>
 * <<<WARNING>>> <<<Wires will be PULLED out if the articulation motors run wild>>>        <<<WARNING>>>
 * @Suport Method for setAllArticulation
 * 
 * @return:       returns true if the motor is at the correct direction, false if still moving
 * 
 * @param ID:     the ID of the articulation wheel we want to move (should be wheel number 6-12 only)
 * @param degrees:the target angle we want the articulation wheel to point at
 */
boolean setMotorArticulation(int ID, int degrees){
  if(ardCancel == true){return true;}
  if(wheelID[ID] == 0){return true;}
  
  //update articulation
  int articulation = articulationRead(ID-6);
  
  //if the motor is already within the turnError stop
  if(abs(articulation/*(ID-6)*/  - degrees) < turnError){
    stopMotor(ID);
    stopMotor(ID-6);
    return true;
  }
  //if the motor is too far, go back
  else if(articulation/*(ID-6)*/ > degrees){
    runMotor(ID, FORWARD, turnSpeed);
    runMotor(ID-6, BACKWARD, (int)(turnSpeed*artHelperSpeed));
    return false;
  }
  //if the motor isn't far enough, go forward
  else{
    runMotor(ID, BACKWARD, turnSpeed);
    runMotor(ID-6, FORWARD, (int)(turnSpeed*artHelperSpeed));
    return false;
  }
}

/**
 * Returns the current Articulation of a given articulationWheelID //================================================= // ======================================================== \\
 * "analogRead()" returns an int of the degrees a articulation sensor is turned (from 0 to 1024)                       // <<<(This will have to be changed if encoders changed)>>> \\
 * "A0" "A1" ... "" is based off what "anolog in" wire slot the articulation on the Articulation sensor is plugged into// ======================================================== \\
 * #Support Method for setMotorArticulation
 * 
 * @param articulationWheelID: the articulation wheel we want to read it's analog sensor
 */
int articulationRead(int articulationWheelID){
  switch(articulationWheelID){
    case 0: case 6: return analogRead(A3);  //FR wheel
      break;
    case 1: case 7: return analogRead(A4);  //MR wheel
      break;
    case 2: case 8: return analogRead(A5);  //BR wheel
      break;
    case 3: case 9: return analogRead(A0);  //FL wheel
      break;
    case 4: case 10: return analogRead(A1);  //ML wheel
      break;
    case 5: case 11: return analogRead(A2);  //BL wheel
      break;
    default: 
      msg2user("\nError: Arduino called articulationRead for invalid ID: "+ (String)articulationWheelID + "\n");
      return 0;
      break;
  }
}

/**
 * Sets the newDegrees array to an adjusted (and correct) orientations
 * This method takes in target "degrees[6]" adds them to the wheel's inherent offset (from mecanical's side)
 * and writes the new wheel orientations to global variable "newDegrees[6]"
 * #Support method for setAllArticulation
 * 
 * @param: degrees[6]:  what direction the user wants the wheels to point, pretending the wheels "zero orientation" is at the folded in position
 */
void adjustForWheelOffset(int degrees[6]){
  if(ardCancel == true){return;}
  //go through all 6 articulation motors
  for(int ID = 6; ID<12; ID++){
    newDegrees[ID-6] = wheelTrueZero[ID-6] + degrees[ID-6];
  }
}

//====================================================================================================================================================================
// Misc Methods, used to allow Other Methods in use, [runMotor, keepGoing, stopMotor(s), simpleDelay, checkForInterupt, ext.]
//====================================================================================================================================================================

/**
 * runs a motor given motorID, direction, and speed
 * 
 * @param ID: the wheelID[] position
 * @param commandInt:  the direction we want to rotate the wheel
 *              0: REVERSE(counter-clock), 1: FORWARD(clockwise)
 * @param tempSpeed: the speed you want it to rotate
 *              0: Stopped, else Go.
 */
void runMotor(int ID, int commandInt, int tempSpeed){ //=====================================================================================================
  //If we are Paused or canceled, do not allow any Motor to run, only stop motor
  if(ardCancel == true && tempSpeed != 0){return;}
  //if(ardPause && tempSpeed != 0){return;}

  //int speed = tempSpeed;
//  if(wheelPinType[ID] == 2){
//    speed = 255 - tempSpeed;
//  }
  
  unsigned char address = wheelID[ID];
  unsigned char command;
  //reverse the wheels direction if need be
  if(wheelPinType[ID] == 1){
    //Move Drive Motor
    if((wheelDirection[ID] == true && commandInt == FORWARD) || 
       (wheelDirection[ID] == false && commandInt == BACKWARD)){
      //Drive FORWARD (Clockwise)
      command = (char)FORWARD_DRIVE;
    }
    else if ((wheelDirection[ID] == true && commandInt == BACKWARD) ||
             (wheelDirection[ID] == false && commandInt == FORWARD)){
      //Drive Backward (Counter-Clockwise)
      command = (char)BACKWARD_DRIVE;
    }
    
  }
  else if(wheelPinType[ID] == 2){
    //Move Articulation Motor
    if((wheelDirection[ID] == true && commandInt == FORWARD) || 
       (wheelDirection[ID] == false && commandInt == BACKWARD)){
      //Art. FORWARD (Clockwise)
      command = (char)FORWARD_ART;
    }
    else if ((wheelDirection[ID] == true && commandInt == BACKWARD) || 
             (wheelDirection[ID] == false && commandInt == FORWARD)){
      //Art. Backward (Counter-Clockwise)
      command = (char)BACKWARD_ART;
    }
  }

  //adjust speed for corection factor
  int speed = (int)(tempSpeed*wheelCorectFactor[ID]);
  if(speed > 127){
    speed = 127;
    msg2user("Error: Motor #" + (String)(ID) + " has been given a speed greater than allowed, motor was slown down to it's max speed\n");
  }
  
  //checksum is an important variable/line of code for moving the motor
  unsigned char checksum = (address + command + ((char)speed)) & 0b01111111;
  
  // Write to the correct serial packet.
  if(wheelPinType[ID] == 1 || wheelPinType[ID] == 2){
    //This is the 4 lines of code that ACTUALY moves the motor, do not mess with
    Serial1.write(address);
    Serial1.write(command);
    Serial1.write(((char)speed));
    Serial1.write(checksum);
  }
//  else if(wheelPinType[ID] == 2){  //TODO FIXED????
//    Serial2.write(address);
//    Serial2.write(command);
//    Serial2.write(((char)speed));
//    Serial2.write(checksum);
//  }
//  else if(wheelPinType[ID] == 3){
//    Serial3.write(address);
//    Serial3.write(command);
//    Serial3.write(((char)speed));
//    Serial3.write(checksum);
//  }
  //so we don't overload any serial buffers
  simpleDelay();
  //if we need to paused, paused
  //checkForInterupt(); // <-- May need this -------------------------------------------------------------------
}

/**
 * Stops all Motors
 */
void stopAllMotor(){
  for(int i = 0; i < numberOfWheels; i++){
    //sets all motor's speed to zero
    runMotor(i, 0, 0);
  }
}

/**
 * Sets a motor's speed to zero
 * 
 * @param ID: the target motor we want to stop
 */
void stopMotor(int ID){
  runMotor(ID, 0, 0);
}

/**
 * Delay Method so we don't overload any serial buffers
 */
void simpleDelay(){
  //Sleep for X time
  for(int i = 0; i < 7; i++){
    delayMicroseconds(1500);
  }
}

/**
 * updates all the commands that would interupt or system command,(pause, estop, cancel)
 * #Called in while loops
 * 
 * return:  Amount of time in Milliseconds we spent paused
 */
long checkForInterupt(){
  nh.spinOnce(); //ROS updates comunication 

  //if it's a new command, and the command is a cancel, pause, or eStop
  if((ardSerialID != rpiSerialID) && (rpiPause || rpiCancel || rpiEStop)){
    //Update SerialID because we are now doing a new command
    ardSerialID = rpiSerialID;

    if(rpiEStop){
      commandEStop(rpiEStop);
    }
    else if(rpiPause){
      //if we need to pause return how much time we were paused
      long startTime = millis();
      commandPause(rpiPause);
      return millis() - startTime;
    }
    else if(rpiCancel){
      //let the whole program know we wish to cancel
      msg2user("/t <<<Command Canceled>>> \n");
      stopAllMotor();
      ardCancel = true;
    }
  }

  //return 0 if we didn't pause
  return 0;
}

/**
 * Causes the code to stop forever
 * Also stops all motors
 */
void eternalSleep(){
  msg2user("\nPowering down and disabling all Systems\n***If the user wishes to restart Arduino, the reset button (located on the phisical arduino board) must be pressed***\n");
  stopAllMotor();
  msg2user("\tArduino Shutdown Sucessfully.\n");
  while(true){
    delay(15000);
  }
}

/**
 * Adds the new msg to adrProgress and increments ardMsgID (So R-Pi knows this is a unique message), publishes mesage
 * If we are testing, only print to Serial Monitor
 */
void msg2user(String printLine){
  if(mode == csTesting){
    //if we are testing print also print line to serial
    Serial.print(printLine);
  }
  else{
    //adds the new mesage to ardProgress
    ardProgress = printLine;
    ardMsgID++;
    //publishes the message
    updateMessage();
    //clears the old String (so it won't accedently print twice)
    ardProgress = "";
  }
}

//====================================================================================================================================================================
//Testing Methods (not used in final code)
//====================================================================================================================================================================
//
///**
// * Runs all Articulation Motors (For Testing purposes ONLY)
// * Should not be used in final code (Will rip out Wires)
// * 
// * @param command:  direction we want to go
// * @param speed:    the speed we want the articulation motors to turn
// */
//void runAllArticulation(boolean command, int speed){
//  for(int i = 6; i < 12; i++){
//    runMotor(i, command, speed);
//  }
//}
//
///**
// * sets one articulation motor to a given degrees (Safe to run)
// * 
// * @param ID:      the id of the articulation wheel we want to change
// * @param degrees: the target degrees we want to turn to
// */
//void setOneArticulation(int ID, int degrees){
//  boolean done = false;
//  int newOneDegree = degrees + wheelTrueZero[ID-6];
//  while(done == false){
//    done = setMotorArticulation(ID, newOneDegree);
//  }
//}
//
///**
// * runs all Drive Motors, All 6 drive wheels turn (clockwise/counter-clockwise) uniformly
// * 
// * @param direction:  FORWARD(clockwise) or BACKWARD(counter-clock)
// * @param speed:      The speed you want all 6 drive motors to turn
// */
//void runAllDriveUniform(boolean direction, int speed){
//  //set all drive motors to forward
//  for(int i = 0; i < 6; i++){
//    runMotor(i, direction, speed);
//  }
//}
//
///**
// * Sets the driveSpeed and adjusts the values of driveError (Safe to use)
// * #Called once in "setup"
// * 
// * @param newSpeed: the new speed of driveSpeed
// */
//void setDriveSpeed(int newSpeed){
//  //127 is Max Speed => our motors will not run above this
//  //15  is Min Speed => any less and motors will not move
//  driveSpeed = newSpeed;      //Speed of driveMotors going forward 
//  //driveError = driveSpeed/2;  //The allowed room for error in drive forward X distance
//}
//
///**
// * Sets the turnSpeed and adjusts the valuse of turnError (Safe to use)
// * #Called once in "setup"
// * 
// * @param newSpeed: the new speed of turnSpeed
// */
//void setTurnSpeed(int newSpeed){
//  //127 is Max Speed => our motors will not run above this
//  //15  is Min Speed => any less and motors will not move
//  turnSpeed = newSpeed;         //Speed of articulationMotors turning
//  turnError = turnSpeed/2;      //The allowed room for error when turning the articulation motors to X angle
//}
//
//
///**
// * Causes no change in motor control until distance = time*speed/timeMod //(Not currently in use but should be)================================================
// * 
// * @param distance: The distance we want to keep moving to
// * @param speed:    The speed at wich we are moving
// * @param timeMod:  The rate at which speed relates to distance
// */
//void keepGoing(int distance, int speed, double timeMod){
//  float distanceTraveled = 0.0;
//  
//  //if we are not within the driveError of the distance, we keep going
//  while((abs(distance) - distanceTraveled) > 0/*driveError*/){
//    //delay so we don't run the while loop into an overflow
//    simpleDelay();
//    //if we need to pause, pause
//    checkForInterupt();
//
//    //Keeps track of how far we have traveled, based off time
//    distanceTraveled += driveSpeed/timeMod;
//  }
//}
//
//
///**
// * Pauses   rover if newPause is true and rover is not Paused (NO LONGER USEFULL)
// * Releases rover if newPause is false and rover is Paused
// * 
// * @param newPause: The new pause state
// */
//void togglePause(boolean newPause){
//  if(newPause == true && ardPause == false){
//    //we need to Pause: stop motors from spining
//    msg2user("C#" + (String)ardSerialID + ", Paused, stopping all motors\n");
//    stopAllMotor();
//    //Bans motors from moving
//    ardPause = true;
//  }
//  else if(newPause == false && ardPause == true){
//    //if(ardPause == true){ //Allows motors to move
//    msg2user("\tPause Ended, allow motors to resume\n");
//    ardPause = false;
//  }
//}
//
//void sendMsg(){
//  if(mode == testing){return;};
//  
//  ardMsgID++;
//  //publishes the message
//  updateMessage();
//  //clears the old String (so it won't accedently print twice)
//  ardProgress = "";
//}




