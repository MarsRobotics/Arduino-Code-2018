////#include <ros.h>
////#include <std_msgs/String.h>
////#include <std_msgs/Int16.h>
////#include <std_msgs/Float32.h>
////#include <comand2ros/Manualcomand.h>
////#include <EncoderFL.h>
//
////EncoderFL efl;
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
//====================================================================================================================================================================
//====================================================================================================================================================================
//====================================================================================================================================================================

//====================================================================================================================================================================
// ROS Variables
//====================================================================================================================================================================

#include <ros.h>
#include <std_msgs/Int32.h>
//#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <command2ros/MovementCommand.h>
ros::NodeHandle nh;

/**
 * Test messageCb (copied from internet)
 */
void messageCb(const std_msgs::Int32 &msg){
  float var=msg.data;
}

//====================================================================================================================================================================
// Program Variables
//====================================================================================================================================================================
//int currentTime = 0;
//int runTime = 30;

//#include "iostream"

//127 is Max Speed => our motors will not run above this
//15  is Min Speed => any less and motors will not move
int driveSpeed = 50;    //Speed of driveMotors going forward 
int turnSpeed = 30;     //Speed of articulationMotors turning into positions
int turnDiveSpeed = 30; //Speed of driveMotors turning while rover is turning left/right

//double driveError;// = driveSpeed/2;  //The allowed room for error in drive forward X distance
double turnError = 5;//turnSpeed/2;    //The allowed room for error when turning the articulation motors to X angle
double artHelperSpeed = 0.8;          //Speed modifier. Used so the drive wheels turn during wheel articulation smoothly (so the wheels don't drag when steering)
double innerTurnSpeed = 0.8;          //Speed modifier. Used so the inner 2 drive wheels turn slower when the rover is turning in place

boolean FORWARD = true;   //Clockwise rotation
boolean BACKWARD = false; //Counter-Clockwise rotation
//These need special ints to control moter rotations
const int FORWARD_DRIVE = 1;   //Clockwise rotation for drive wheels
const int BACKWARD_DRIVE = 0; //Counter-Clockwise rotation for drive wheels
const int FORWARD_ART = 4;     //Clockwise rotation for articulation wheels
const int BACKWARD_ART =5;    //Counter-Clockwise rotation for articulation wheels

//List of "States" the 6 wheels can be facing, 1024 is 360*, 0 is 0* 
int stateStorage[6] =   {768, 256, 256, 256, 768, 768};  //orientation of wheels for the storage/collapsed position, (the position our robot will start RMC at)
int stateForward[6] =   {256, 768, 768, 768, 256, 256};  //orientation of wheels needed to drive straight/forward
int stateTurning[6] =   {384, 768, 640, 640, 256, 384};  //orientation of wheels needed to turn the robot

int newDegrees[6] = {0,0,0,0,0,0};  //storage value used to pass an array of Ints between methods //(Should be a better way of doing this, but IDK)

boolean pause = false; //If we are Pauseed we don't run motors pauses program


//====================================================================================================================================================================
// Digger and Dumper Varables
//====================================================================================================================================================================

//The amount of time the digger and dumper, spend diging and dumping
int diggerTime   = 1000; 
int wenchDistance= 1000;
int dumperTime  = 1000;

int diggerSpeed = 30;
int wenchSpeed  = 30;
int dumperSpeed = 30;

//====================================================================================================================================================================
// Wheel-Variables, Varables for each individual wheel
//====================================================================================================================================================================
int numberOfWheels = 15;       //0/6  1/7  2/8  3/9  4/10 5/11

//String wheelName[15] =       {"fr-0", "mr-1", "br-2", "fl-3", "ml-4", "bl-5",   //Drive motors  //"fr" means Fount-Right, ext
//                              "fr-6", "mr-7", "br-8", "fl-9", "ml-10", "bl-11",   //articulation motors //"rl" means Rear-Left
//                              "dumper-12", "wench-13", "digger-14"};   //Misc motors

//unsigned char wheelID[15] =  {133, 132, 131, 128, 129, 130,         //Drive motors
//                              133, 132, 131, 128, 129, 130,         //articulation motors
//                              100, 100, 100};                       //Misc motors

//Test (disabled 2 back wheels)
unsigned char wheelID[15] =  {133, 132, 131, 128, 129, 0,         //Drive motors
                              133, 132, 131, 128, 129, 0,         //articulation motors
                              100, 100, 100};                       //Misc motors

//If mecanical/electrical make a mistake and put a motor on backwards, set Direction to false to reverse rotations
boolean wheelDirection[15] = {true, true, true, true, true, true,   //Drive motors
                              false, true, true, true, true, true,   //articulation motors
                              true, true, true};                    //Misc motors

//int wheelArticulation[15] =  {0, 0, 0, 0, 0, 0,                     //Drive motors
//                              0, 0, 0, 0, 0, 0,                     //articulation motors
//                              0, 0, 0};                             //Misc motors
                              
//Serial1 on pins 19 (RX) and 18 (TX), Serial2 on pins 17 (RX) and 16 (TX), Serial3 on pins 15 (RX) and 14 (TX)
const int wheelPinType[15] = {1, 1, 1, 1, 1, 1,
                              2, 2, 2, 2, 2, 2,
                              1, 1, 1};

//articulation sensors's true zero relative to our ideal true zero on rover
const int wheelTrueZero[6] ={-16, 42, 9, 151, 416, 202};         

//Speed modifier for wheels, (because wheels turn at diffrent speeds for unknown reasons), so they turn at same speed
double wheelCorectFactor[15] =  {0.92, 1.00, 1.00, 1.02, 1.00, 1.00,        //Drive motors
                                 2.00, 1.10, 2.00, 1.15, 1.00, 1.00,        //articulation motors
                                 1.0, 1.0, 1.0};                      //Misc motors

//================================================================================================================================================
// R-Pi varables, R-Pi should be listening for and varables R-Pi writes to give comands 
//================================================================================================================================================

int rpiDriveDistance= 0;
int rpiTurnDegrees  = 0;
boolean rpiDigger = false;
boolean rpiDumper = false;
boolean rpiPackIn = false;
boolean rpiPause =  false;
boolean rpiEStop =  false;
//boolean rpiCancel = false;

//This should be true if we are ready for a new comand
boolean ardReady =  true;
int comandAccepted = 0;

//================================================================================================================================================
// Message Command
//================================================================================================================================================

/**
 * Updates the arduino with the current information from r-pi, and writes it to our program varables
 */
void messageCb(const command2ros::MovementCommand &msg){
  //Information from our msg extracted and copied to our varables
  rpiDriveDistance= msg.driveDist;
  rpiTurnDegrees  = msg.turn;
  rpiDigger = msg.dig;
  rpiDumper = msg.dump;
  rpiPackIn = msg.packin;
  rpiEStop =  msg.eStop;
  rpiPause =  msg.stop;
  //rpiCancel = false;
}

//ros::Subscriber<comand2ros::Movementcomand> sub("Movementcomand", &messageCb);
ros::Subscriber<std_msgs::Int32> sub("std_msgs", &messageCb);

//================================================================================================================================================
// Setup and Main Loop, "Setup" is called once at start, "Loop" loops infinitly
//================================================================================================================================================
void setup() {
  //Begin all Serial Writers
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  //Sets the speed of all our drive motors and articulation motors
  //setDriveSpeed(driveSpeed);
  //setTurnSpeed(turnSpeed);

  //pinMode(13, OUTPUT);
//  nh.initNode();
//  nh.subscribe(sub);
}



void loop() {
  //Main loop, reads from 
  boolean RPiActive = false;
  if(RPiActive == true){
    nh.spinOnce(); //ROS updates comunication
    //This should be our real looping method call, (temp. disabled for testing)
//    comandDriveForward(rpiDriveDistance);
//    comandTurnRover(rpiTurnDegrees);
//    comandDigger(rpiDigger);
//    comandDumper(rpiDumper);
//    comandPackIn(rpiPackIn);
//    comandSetPause(rpiPause);
//    comandEStop(rpiEStop);
    if(rpiPackIn == true){
      digitalWrite(13, HIGH);
    }
    else{
      digitalWrite(13, LOW);
    }
    
  }
  else{
    //for testing
    printArticulation();
//    int rA = 768; //6, 10, 11
//    int fA = 256;
//        
//    int rB = 256; //7, 8, 9
//    int fB = 768;

//    setAllArticulation(stateStorage); //stateTurning    stateStorage    stateForward
//    setAllArticulation(stateTurning);
//    setAllArticulation(stateForward);
//    comandDriveForward(10000);
    //testArticulation(stateForward);
//    testArticulation(stateForward);


//    runMotor(8, FORWARD, 30); //BACKWARD(CC)   FORWARD(CW) 
//    runMotor(10, FORWARD, 40);
//    delay(1000);
//    stopAllMotor();
//    delay(3000);
//    printArticulation();
//    runMotor(8, BACKWARD, 30);

    //setOneArticulation(8, )

    //setAllArticulation(stateStorage);

//    int stateStorage[6] =   {768, 256, 256, 256, 768, 768};  //orientation of wheels for the storage/collapsed position, (the position our robot will start RMC at)
//int stateForward[6] =   {256, 768, 768, 768, 256, 256};  //orientation of wheels needed to drive straight/forward
//int stateTurning[6] =   {384, 768, 640, 640, 256, 384};  //orientation of wheels needed to turn the robot


//    int fFR = 410;
//    int rFR = 768;
    //setOneArticulation(6, fA);
    
    //Test Code
    //driveForward(2000);     //1,000 is one rotation, 2,000 is two (approximation: varies by driveSpeed)
    //driveForward(2000);
//    setTurnSpeed(15);
    //setAllArticulation(stateForward);
    //int A = sensorValue -200; //-CW +CCW
    //int B = 300; //-CW +CCW
    //int BList[6] = {B,B,B, B,B,B};
    //setTurnSpeed(15);
    //setAllArticulation(BList);
    //setTurnSpeed(30);
    //setAllArticulation(stateForward);
    //stopAllMotor();
    
    

//    for(int i = 9; i < 12; i++){  //For Left
//    for(int i = 6; i < 9; i++){   //For Right
//      printArticulation();
//      runMotor(i, FORWARD, 30);
//      delay(1500);
//      printArticulation();
//      runMotor(i, BACKWARD, 30);
//      delay(1500);
//      stopMotor(i);
//      delay(300);
//    }
    /*
     * List of "States" the 6 wheels can be facing, 1024 is 360*, 0 is 0* 
     * FORWARDS = CounterClock
     * FR(A3-133), MR(A4-132), BR(A5-131), FL(A0-128), ML(A1-129), BL(A2-130)
     */
    
    //int start = analogRead(A3);
    //Serial.println("here" + analogRead(A3));
    //String testArticulation = "FR: %d, MR: %d, BR: %d,   FL: %d, ML: %d, BL: %d", 1,2,3,4,5,6;//analogRead(A3), analogRead(A4), analogRead(A5),  analogRead(A0), analogRead(A1), analogRead(A2);
    //Serial.println("FR: %d, MR: %d, BR: %d,   FL: %d, ML: %d, BL: %d", analogRead(A3), analogRead(A4), analogRead(A5),  analogRead(A0), analogRead(A1), analogRead(A2));

//    setOneArticulation(11, rest -512);
//    delay(1500);
//    setOneArticulation(11, rest);

    //setAllArticulation(stateForward);

    
    //delay(1500);
    //setOneArticulation(9, fB);
    //driveForward(3000);
    
    //runMotor(9, FORWARD, 30);
    //setOneArticulation(9, 340);
    //setOneArticulation(2+6, 400);

    //runAllArticulation(FORWARD, turnSpeed);

    //setAllArticulation(stateStorage);

    
//    runMotor(3+6, BACKWARD, 40); //==============================================================================================================
    delay(1500);
//    stopMotor(3+6);
    
    printArticulation();
    Serial.println("test done");
    eternalSleep();
  }
  
  //Delay so we don't overload any serial buffers
  simpleDelay();
}

/**
 * Prints the current Articulation reads for all 6 articulation sensors
 */
void printArticulation(){
  String articulation = "Relative to wheelTrueZero:   FR: "+
                           (String)(analogRead(A3)-wheelTrueZero[0])+
                ",\t MR: "+(String)(analogRead(A4)-wheelTrueZero[1])+
                ",\t BR: "+(String)(analogRead(A5)-wheelTrueZero[2])+
            ",\t\t\t FL: "+(String)(analogRead(A0)-wheelTrueZero[3])+
                ",\t ML: "+(String)(analogRead(A1)-wheelTrueZero[4])+
                ",\t BL: "+(String)(analogRead(A2)-wheelTrueZero[5]);

  Serial.println(articulation);
}
/**
 * prints the current articulation and prints how far it is from a specific stateCur[6]
 */
void testArticulation(int stateCur[]){
  //printArticulation();
  
  //prints the basic data the sensors are giving
  String rawDis= "Raw Data given by sensors:   FR: "+
                          (String)(analogRead(A3)+0)+
               ",\t MR: "+(String)(analogRead(A4)+0)+
               ",\t BR: "+(String)(analogRead(A5)+0)+
           ",\t\t\t FL: "+(String)(analogRead(A0)+0)+
               ",\t ML: "+(String)(analogRead(A1)+0)+
               ",\t BL: "+(String)(analogRead(A2)+0);
  Serial.println(rawDis);

  //Prints the distances each wheel needs to go (assuming the phisical wheels are pointing in "sateCur[]")
  String dis2Correct = "Dis to correct Articulation: FR: "+
                            (String)(analogRead(A3)-wheelTrueZero[0]-stateCur[0])+
               ",\t MR:   "+(String)(analogRead(A4)-wheelTrueZero[1]-stateCur[1])+
               ",\t BR:   "+(String)(analogRead(A5)-wheelTrueZero[2]-stateCur[2])+
           ",\t\t\t FL:   "+(String)(analogRead(A0)-wheelTrueZero[3]-stateCur[3])+
               ",\t ML:   "+(String)(analogRead(A1)-wheelTrueZero[4]-stateCur[4])+
               ",\t BL:   "+(String)(analogRead(A2)-wheelTrueZero[5]-stateCur[5]);
  Serial.println(dis2Correct);
  

  String newArt = "\n[New] wheelTrueZero = {"+
                  (String)(analogRead(A3)-stateCur[0])+", "+
                  (String)(analogRead(A4)-stateCur[1])+", "+
                  (String)(analogRead(A5)-stateCur[2])+", "+
                  (String)(analogRead(A0)-stateCur[3])+", "+
                  (String)(analogRead(A1)-stateCur[4])+", "+
                  (String)(analogRead(A2)-stateCur[5])+"};\n";
  Serial.println(newArt);
}

//====================================================================================================================================================================
// comand Methods, these are the only ones the R-pi is expected to call
//====================================================================================================================================================================
/**
 * Sets the Rover's wheels to point straight and drives FORWARD
 * 
 * @param distance: Distance we want to drive FORWARD (enter negitive number for backwards)
 */
void comandDriveForward(int distance){
  //if our value is zero our R-Pi didn't actualy want to call this method, also we want to turn the rover first
  if(distance == 0 || pause){return;}
  rpiDriveDistance = 0; //TODO-------------------------------------------------------------------------
  
  ardReady = false;
  setAllArticulation(stateForward);
  driveForward(distance);

  //Tells R-Pi we are ready and resets rpiDriveDistance
  ardReady = true;
}

/**
 * Sets the Rover's wheels to point in a circle and Rotates Rover in place
 * 
 * @param degrees: Degrees we want to turn Rover (enter negitive number for Left)
 */
void comandTurnRover(int degrees){
  //if our value is zero our R-Pi didn't actualy want to call this method
  if(degrees == 0 || pause){return;}
  rpiTurnDegrees = 0; //TODO-------------------------------------------------------------------------
  
  ardReady = false;
  setAllArticulation(stateTurning);
  turnRover(degrees);

  //Tells R-Pi we are ready and resets rpiTurnDegrees
  ardReady = true;
}

/**
 * Cause the rover to lower wench and runs the digger for "diggerTime"
 * 
 * @param startDigger: true if we want to dig now
 */
void comandDigger(boolean startDigger){
  //If digger in is false we didn't actualy want to call this method
  if(startDigger == false || pause){ return;}
  //move wench down, run digger, move wench back up
  driveOneForward(13, wenchDistance, wenchSpeed); //move wench
  driveOneForward(12, diggerTime, diggerSpeed);   //run dumper
  driveOneForward(13, -wenchDistance, wenchSpeed);//move wench back
}

/**
 * Cause the rover run the dumper for "dumperTime"
 * 
 * @param startDumper: true if we want to dump right now
 */
void comandDumper(boolean startDumper){
  //If dumper in is false we didn't actualy want to call this method
  if(startDumper == false || pause){ return;}
  //run the dumper for dumperTime
  driveOneForward(14, dumperTime, dumperSpeed); //run dumper
}

/**
 * Sets the Rover's wheels to the resting/storage position
 * 
 * @param packIn: true, if we actualy want to pack our wheels in
 *                false, if we don't actualy want to pack wheels in
 */
void comandPackIn(boolean packIn){ 
  //If pack in is false we didn't actualy want to call this method
  if(packIn == false || pause){ return;}
  rpiDriveDistance = 0;//TODO-------------------------------------------------------------------------
  rpiTurnDegrees = 0;  //TODO-------------------------------------------------------------------------
  
  ardReady = false;
  setAllArticulation(stateStorage);
  ardReady = true;
}

/**
 * If we need to Pause use this comand, Stops motors, banns new comands, and causes delays to last forever, untill unpaused
 * 
 * @param newPause: the new Pause, true for stop, false for resume
 *     => true:    Stop all motors and ban motors from turning
 *     => false:   Allow motors to move again (unpause)
 */
void comandSetPause(boolean tempPause){
  if(tempPause == true && pause){
    //Do nothing R-Pi requested Pause and we are already Pauseed
    return;
  }
  else if(tempPause == true && pause == false){
    //we need to Pause: stop motors from spining
    stopAllMotor();
    //Bans motors from moving
    pause = true;

    //simpleDelay();
    //checkForPause();
  }
  else{
    //Allows motors to move again
    pause = false;
  }
}

/**
 * Irreversably stop all motors and sleep forever
 */
void comandEStop(boolean tempEStop){
  if(tempEStop){
    //Stops all Motors and sleeps forever
    eternalSleep();
  }
}

//====================================================================================================================================================================
// Drive Methods, used to allow Drive comands drive Rover correct distances
//====================================================================================================================================================================

/**
 * Drive forward X distance, (does not reposition wheels)
 * if distance is negative => go backwards
 * #Support method for comandDriveForward
 * 
 * @param distance: target distance we want to rover to travel
 */
void driveForward(int distance){
  float distanceTraveled = 0;
  int direction = FORWARD;
  //if distance is negative => go backwards
  if(distance < 0){
    direction = BACKWARD;
  }
  activeDriveForward(direction, driveSpeed);

  //if we are not within the driveError of the distance, we keep driving
  while(abs(distance) > distanceTraveled /*driveError*/){
    //delay so we don't run the while loop into an overflow
    simpleDelay();
    //if we need to pause, pause
    checkForPause();

    //Keeps track of how far we have traveled, based off time
    distanceTraveled = distanceTraveled + driveSpeed/10.0;
  }

  stopAllDrive(); //set all drive motor's speed to zero
}

/**
 * actualy runs all 6 Drive Motors, 3 right wheels one way, 3 left wheels the other to go forward
 * #Support method for driveFORWARD
 * 
 * @param direction:  FORWARD(clockwise) or BACKWARD(counter-clock)
 * @param speed:      The speed you want all 6 drive motors to turn
 */
void activeDriveForward(int direction, int speed){
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
  activeDriveForward(0, 0);
}

//====================================================================================================================================================================
// Turn Methods, used to allow Rover to turn itself the correct distance
//====================================================================================================================================================================

/**
 * Turn Rover Right X degrees
 * if degrees is negative => go backwards
 * #Support Method for comandTurnRover
 * 
 * @param:  degrees, target degrees we want to rover to turn
 */
void turnRover(int degrees){
  float distanceTraveled = 0;
  int direction = FORWARD;
  //if distance is negative => go backwards(left)
  if(degrees < 0){
    direction = BACKWARD;
  }
  activeTurn(direction, turnDiveSpeed);
  
  //if we are not within the driveError of the distance, we keep driving
  while((abs(degrees) - distanceTraveled) > 0/*driveError*/){
    //delay so we don't run the while loop into an overflow
    simpleDelay();
    //if we need to pause, pause
    checkForPause();
    
    //Keeps track of how far we have traveled, based off time 
    distanceTraveled = distanceTraveled + turnDiveSpeed/10.0;
    
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
 * #Support Method for comandDigger and comandDumper
 * 
 * @param ID:       the id of the drive wheel we want to move
 * @param distance: the distance we want it to travel
 */
void driveOneForward(int ID, int distance, int speed){
  float distanceTraveled = 0;
  int direction = FORWARD;
  //if distance is negative => go backwards
  if(distance < 0){
    direction = BACKWARD;
  }
  runMotor(ID, direction, speed);
  
  //if we are not within the driveError of the distance, we keep driving
  while((abs(distance) - distanceTraveled) > 0 /*driveError*/){
    //delay so we don't run the while loop into an overflow
    simpleDelay();
    //if we need to pause, pause
    checkForPause();
    distanceTraveled = distanceTraveled + speed/10.0; //Temp place-holder
  }
  stopMotor(ID); //set all drive motor's speed to zero
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
  boolean done = false;
  boolean isFinished = false;
  
  //update "newDegrees" array with adjusted values
  adjustForWheelOffset(degrees);

  boolean wheelDone[6] = {false, false, false, false, false, false};
  
  //while we are not done turning all motors
  while(done == false){
    //if we encounter no unfinished turning wheels we remain "done"
    done = true;
    
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
  if(wheelID[ID] == 0){return true;}
  
  //update articulation
  int articulation = articulationRead(ID-6);
  Serial.println(articulation - wheelTrueZero[ID-6]);
  
  //if the motor is already within the turnError stop
  if(abs(articulation/*(ID-6)*/  - degrees) < turnError){
    stopMotor(ID);
    stopMotor(ID-6);//------------------------------------------------------------------------------
    return true;
  }
  //if the motor is too far, go back
  else if(articulation/*(ID-6)*/ > degrees){
    runMotor(ID, FORWARD, turnSpeed);
    runMotor(ID-6, BACKWARD, (int)(turnSpeed*artHelperSpeed));//-----------------------------------------------------
    return false;
  }
  //if the motor isn't far enough, go forward
  else{
    runMotor(ID, BACKWARD, turnSpeed);
    runMotor(ID-6, FORWARD, (int)(turnSpeed*artHelperSpeed));//------------------------------------------------------
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
    case 0: return analogRead(A3);  //FR wheel
      break;
    case 1: return analogRead(A4);  //MR wheel
      break;
    case 2: return analogRead(A5);  //BR wheel
      break;
    case 3: return analogRead(A0);  //FL wheel
      break;
    case 4: return analogRead(A1);  //ML wheel
      break;
    case 5: return analogRead(A2);  //BL wheel
      break;
    default: return 0;
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
  //go through all 6 articulation motors
  for(int ID = 6; ID<12; ID++){
    newDegrees[ID-6] = wheelTrueZero[ID-6] + degrees[ID-6];
  }
}

//====================================================================================================================================================================
// Misc Methods, used to allow Other Methods in use, [runMotor, keepGoing, stopMotor(s), simpleDelay, checkForPause, ext.]
//====================================================================================================================================================================

/**
 * runs a motor given motorID, direction, and speed
 * 
 * @param ID: the wheelID[] position
 * @param comandInt:  the direction we want to rotate the wheel
 *              0: REVERSE(counter-clock), 1: FORWARD(clockwise)
 * @param tempSpeed: the speed you want it to rotate
 *              0: Stopped, else Go.
 */
void runMotor(int ID, int comandInt, int tempSpeed){ //=====================================================================================================
  //If we are Pauseed, do not allow any Motor to run, only stop motor
  if(pause && tempSpeed != 0){return;}

  //int speed = tempSpeed;
//  if(wheelPinType[ID] == 2){
//    speed = 255 - tempSpeed;
//  }
  
  unsigned char address = wheelID[ID];
  unsigned char comand;
  //reverse the wheels direction if need be
  if(wheelPinType[ID] == 1){
    //Move Drive Motor
    if((wheelDirection[ID] == true && comandInt == FORWARD) || 
       (wheelDirection[ID] == false && comandInt == BACKWARD)){
      //Drive FORWARD (Clockwise)
      comand = (char)FORWARD_DRIVE;
    }
    else if ((wheelDirection[ID] == true && comandInt == BACKWARD) ||
             (wheelDirection[ID] == false && comandInt == FORWARD)){
      //Drive Backward (Counter-Clockwise)
      comand = (char)BACKWARD_DRIVE;
    }
    
  }
  else if(wheelPinType[ID] == 2){
    //Move Articulation Motor
    if((wheelDirection[ID] == true && comandInt == FORWARD) || 
       (wheelDirection[ID] == false && comandInt == BACKWARD)){
      //Art. FORWARD (Clockwise)
      comand = (char)FORWARD_ART;
    }
    else if ((wheelDirection[ID] == true && comandInt == BACKWARD) || 
             (wheelDirection[ID] == false && comandInt == FORWARD)){
      //Art. Backward (Counter-Clockwise)
      comand = (char)BACKWARD_ART;
    }
  }

  //adjust speed for corection factor
  int speed = (int)(tempSpeed*wheelCorectFactor[ID]);
  if(speed > 127){
    speed = 127;
    String errorMsgSpeed = "Error: Motor #" + (String)(ID) + " has been given a speed greater than allowed, motor was slown down to it's max speed";
    Serial.println(errorMsgSpeed);
  }
  
  //checksum is an important variable/line of code for moving the motor
  unsigned char checksum = (address + comand + ((char)speed)) & 0b01111111;
  
  // Write to the correct serial packet.
  if(wheelPinType[ID] == 1 || wheelPinType[ID] == 2){
    //This is the 4 lines of code that ACTUALY moves the motor, do not mess with
    Serial1.write(address);
    Serial1.write(comand);
    Serial1.write(((char)speed));
    Serial1.write(checksum);
  }
//  else if(wheelPinType[ID] == 2){  //TODO FIXED????
//    Serial2.write(address);
//    Serial2.write(comand);
//    Serial2.write(((char)speed));
//    Serial2.write(checksum);
//  }
//  else if(wheelPinType[ID] == 3){
//    Serial3.write(address);
//    Serial3.write(comand);
//    Serial3.write(((char)speed));
//    Serial3.write(checksum);
//  }
  //so we don't overload any serial buffers
  simpleDelay();
  //if we need to pause, pause
  //checkForPause(); // <-- May need this -------------------------------------------------------------------
}

/**
 * Causes no change in motor control until distance = time*speed/timeMod //(Not currently in use but should be)================================================
 * 
 * @param distance: The distance we want to keep moving to
 * @param speed:    The speed at wich we are moving
 * @param timeMod:  The rate at which speed relates to distance
 */
void keepGoing(int distance, int speed, double timeMod){
  float distanceTraveled = 0.0;
  
  //if we are not within the driveError of the distance, we keep going
  while((abs(distance) - distanceTraveled) > 0/*driveError*/){
    //delay so we don't run the while loop into an overflow
    simpleDelay();
    //if we need to pause, pause
    checkForPause();

    //Keeps track of how far we have traveled, based off time
    distanceTraveled += driveSpeed/timeMod;
  }
}

/**
 * Stops all Motors
 */
void stopAllMotor(){
  for(int i = 0; i < numberOfWheels; i++){
    //sets all motor's speed to zero
    stopMotor(i);
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
 * Updates pause and causes to delay untill unpaused
 * #Called in while loops
 */
void checkForPause(){
  //sleep forever if Pause needs to be true
  updatePause();
  while(pause){
    delay(10);
    updatePause();
  }
//  if(rpiCancel){ //---------------------------------------------------------------------------
//    rpiCancel = false;
//    return 999999;
//  }
//  return 0;
}

/**
 * updates the Pause comand,
 * #Support Method for checkForPause
 */
void updatePause(){
  //nh.spinOnce(); //ROS updates comunication //-----------------------------------------------------------------------------------------------
  comandSetPause(rpiPause);
  comandEStop(rpiEStop);
}

/**
 * Causes the code to stop forever
 * Also stops all motors
 */
void eternalSleep(){
  stopAllMotor();
  while(true){
    delayMicroseconds(15000);
  }
}

//====================================================================================================================================================================
//Testing Methods (not used in final code)
//====================================================================================================================================================================

/**
 * Runs all Articulation Motors (For Testing purposes ONLY)
 * Should not be used in final code (Will rip out Wires)
 * 
 * @param comand:  direction we want to go
 * @param speed:    the speed we want the articulation motors to turn
 */
void runAllArticulation(boolean comand, int speed){
  for(int i = 6; i < 12; i++){
    runMotor(i, comand, speed);
  }
}

/**
 * sets one articulation motor to a given degrees (Safe to run)
 * 
 * @param ID:      the id of the articulation wheel we want to change
 * @param degrees: the target degrees we want to turn to
 */
void setOneArticulation(int ID, int degrees){
  boolean done = false;
  int newOneDegree = degrees + wheelTrueZero[ID-6];
  while(done == false){
    done = setMotorArticulation(ID, newOneDegree);
  }
}

/**
 * runs all Drive Motors, All 6 drive wheels turn (clockwise/counter-clockwise) uniformly
 * 
 * @param direction:  FORWARD(clockwise) or BACKWARD(counter-clock)
 * @param speed:      The speed you want all 6 drive motors to turn
 */
void runAllDriveUniform(boolean direction, int speed){
  //set all drive motors to forward
  for(int i = 0; i < 6; i++){
    runMotor(i, direction, speed);
  }
}

/**
 * Sets the driveSpeed and adjusts the values of driveError (Safe to use)
 * #Called once in "setup"
 * 
 * @param newSpeed: the new speed of driveSpeed
 */
void setDriveSpeed(int newSpeed){
  //127 is Max Speed => our motors will not run above this
  //15  is Min Speed => any less and motors will not move
  driveSpeed = newSpeed;      //Speed of driveMotors going forward 
  //driveError = driveSpeed/2;  //The allowed room for error in drive forward X distance
}

/**
 * Sets the turnSpeed and adjusts the valuse of turnError (Safe to use)
 * #Called once in "setup"
 * 
 * @param newSpeed: the new speed of turnSpeed
 */
void setTurnSpeed(int newSpeed){
  //127 is Max Speed => our motors will not run above this
  //15  is Min Speed => any less and motors will not move
  turnSpeed = newSpeed;         //Speed of articulationMotors turning
  turnError = turnSpeed/2;      //The allowed room for error when turning the articulation motors to X angle
}




