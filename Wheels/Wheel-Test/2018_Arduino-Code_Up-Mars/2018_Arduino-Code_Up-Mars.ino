////#include <ros.h>
////#include <std_msgs/String.h>
////#include <std_msgs/Int16.h>
////#include <std_msgs/Float32.h>
////#include <command2ros/ManualCommand.h>
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
//============================================================================================================================
//============================================================================================================================
//============================================================================================================================

//============================================================================================================================
// ROS Variables
//============================================================================================================================

//#include <ros.h>
//#include <std_msgs/Int32.h>
//
//ros::NodeHandle nh;
//
//float var;
//
//void messageCb(const std_msgs::Int32 &msg)
//{
//  var=msg.data;
// 
//  if(var > 2000)
//   digitalWrite(13, HIGH);   // blink the led
//      else
//   digitalWrite(13, LOW);   // turn off the led
//}
//
//ros::Subscriber sub("rand_no", &messageCb);

//============================================================================================================================
// Program Variables
//============================================================================================================================
//int currentTime = 0;
//int runTime = 30;

//127 is Max Speed => our motors will not run above this
//15  is Min Speed => any less and motors will not move
int driveSpeed = 60;    //Speed of driveMotors going forward 
int turnSpeed = 20;     //Speed of articulationMotors turning
int turnDiveSpeed = 30; //Speed of driveMotors turning rover left/right

//double driveError;// = driveSpeed/2;  //The allowed room for error in drive forward X distance (if this is too small it drives infinitely!!!)
double turnError;// = turnSpeed/2;    //The allowed room for error when turning the articulation motors to X angle
double artHelperSpeed = 0.5;          //Speed modifier. Used so the drive wheels turn during wheel articulation smoothly (so the wheels don't drag when steering)
double innerTurnSpeed = 0.8;          //Speed modifier. Used so the inner 2 drive wheels turn slower when the rover is turning in place

const int FOWARD = 1;   //Clockwise rotation
const int BACKWARD = 0; //Counter-Clockwise rotation

//List of "States" the 6 wheels can be facing, 1024 is 360*, 0 is 0* 
int stateForward[6] =   {512, 512, 512, 512, 512, 512};  //orientation of wheels needed to drive straight/forward
int stateTurning[6] =   {240, 180, 120, 240, 180, 120};  //orientation of wheels needed to turn the robot
int stateTurningB[6] =  {100, 160, 100, 160, 100, 160};  //orientation of wheels needed to turn the robot
int stateStorage[6] =   { 20,  20,  20,  20,  20,  20};  //orientation of wheels for the storage/collapsed position, (the position our robot will start RMC at)

int newDegrees[6] = {0,0,0,0,0,0};  //storage value used to pass an array of Ints between methods //(Should be a better way of doing this, but IDK)

boolean pause = false; //If we are Pauseed we don't run motors pauses program


//============================================================================================================================
// Digger and Dumper Varables
//============================================================================================================================

//The amount of time the digger and dumper, spend diging and dumping
int diggerTime   = 1000; 
int wenchDistance= 1000;
int dumperTime  = 1000;

int diggerSpeed = 30;
int wenchSpeed  = 30;
int dumperSpeed = 30;

//============================================================================================================================
// Wheel-Variables, Varables for each individual wheel
//============================================================================================================================
int numberOfWheels = 15;
unsigned char wheelID[15] =  {100, 129, 100, 100, 100, 100,         //Drive motors
                              130, 100, 100, 100, 100, 100,         //articulation motors
                              100, 100, 100};                       //Misc motors

//If mecanical/electrical make a mistake and put a motor on backwards, set Direction to false to reverse rotations
boolean wheelDirection[15] = {true, true, true, true, true, true,   //Drive motors
                              true, true, true, true, true, true,   //articulation motors
                              true, true, true};                    //Misc motors

//int wheelArticulation[15] =  {0, 0, 0, 0, 0, 0,                     //Drive motors
//                              0, 0, 0, 0, 0, 0,                     //articulation motors
//                              0, 0, 0};                             //Misc motors
                              
//Serial1 on pins 19 (RX) and 18 (TX), Serial2 on pins 17 (RX) and 16 (TX), Serial3 on pins 15 (RX) and 14 (TX)
const int wheelSerial[15] =  {1, 1, 1, 1, 1, 1,
                              1, 1, 1, 1, 1, 1,
                              1, 1, 1};
//String wheelName[15] =       {"fr", "mr", "rr", "fl", "ml", "rl",   //Drive motors  //"fr" means Fount-Right, ext
//                              "fr", "mr", "rr", "fl", "ml", "rl",   //articulation motors //"rl" means Rear-Left
//                              "dumper12", "wench13", "digger14"};   //Misc motors

const int wheelTrueZero[15] ={0, 0, 0, 0, 0, 0,                  //Drive motors
                              0, 0, 0, 0, 0, 0,                  //articulation motors
                              0, 0, 0};                          //Misc motors


//========================================================================================================
// R-Pi varables, R-Pi should be listening for and varables R-Pi writes to give commands 
//========================================================================================================

int rpiDriveDistance= 0;
int rpiTurnDegrees  = 0;
boolean rpiDigger = false;
boolean rpiDumper = false;
boolean rpiPackIn = false;
boolean rpiPause =  false;
boolean rpiEStop =  false;
//boolean rpiCancel = false;

//This should be true if we are ready for a new command
boolean ardReady =  true;

//========================================================================================================
// Setup and Main Loop, "Setup" is called once at start, "Loop" loops infinitly
//========================================================================================================
void setup() {
  //Begin all Serial Writers
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
  //Sets the speed of all our drive motors and articulation motors
  setDriveSpeed(driveSpeed);
  setTurnSpeed(turnSpeed);

  //pinMode(13, OUTPUT);
  //nh.initNode();
  //nh.subscribe(sub);
}
void loop() {
  //Main loop, reads from 
  boolean RPiActive = false;
  if(RPiActive == true){
    //nh.spinOnce(); //ROS updates comunication
    //This should be our real looping method call, (temp. disabled for testing)
    commandDriveFoward(rpiDriveDistance);
    commandTurnRover(rpiTurnDegrees);
    commandDigger(rpiDigger);
    commandDumper(rpiDumper);
    commandPackIn(rpiPackIn);
    commandSetPause(rpiPause);
    commandEStop(rpiEStop);
  }
  else{
    //Test Code
    
    //eternalSleep();
    //driveForward(2000);     //1,000 is one rotation, 2,000 is two (approximation: varies by driveSpeed)
    //setDriveSpeed(60);
    //driveForward(1000);
    //Serial.println("Start");
    //setTurnSpeed(40);
    setDriveSpeed(20);
    driveForward(2000);
//    setTurnSpeed(15);
//    setAllArticulation(stateForward);
    //int A = sensorValue -200; //-CW +CCW
    //int B = 300; //-CW +CCW
    //int BList[6] = {B,B,B, B,B,B};
    //setTurnSpeed(15);
    //setAllArticulation(BList);
    //setTurnSpeed(30);
    //setAllArticulation(stateForward);
    //stopAllMotor();
    eternalSleep();
  }
  
  //Delay so we don't overload any serial buffers
  simpleDelay();
}

//============================================================================================================================
// Command Methods, these are the only ones the R-pi is expected to call
//============================================================================================================================
/**
 * Sets the Rover's wheels to point straight and drives foward
 * 
 * @param distance: Distance we want to drive foward (enter negitive number for backwards)
 */
void commandDriveFoward(int distance){
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
void commandTurnRover(int degrees){
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
void commandDigger(boolean startDigger){
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
void commandDumper(boolean startDumper){
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
void commandPackIn(boolean packIn){ 
  //If pack in is false we didn't actualy want to call this method
  if(packIn == false || pause){ return;}
  rpiDriveDistance = 0;//TODO-------------------------------------------------------------------------
  rpiTurnDegrees = 0;  //TODO-------------------------------------------------------------------------
  
  ardReady = false;
  setAllArticulation(stateStorage);
  ardReady = true;
}

/**
 * If we need to Pause use this command, Stops motors, banns new commands, and causes delays to last forever, untill unpaused
 * 
 * @param newPause: the new Pause, true for stop, false for resume
 *     => true:    Stop all motors and ban motors from turning
 *     => false:   Allow motors to move again (unpause)
 */
void commandSetPause(boolean tempPause){
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
void commandEStop(boolean tempEStop){
  if(tempEStop){
    //Stops allMotors and sleeps forever
    eternalSleep();
  }
}

//============================================================================================================================
// Drive Methods, used to allow Drive Commands drive Rover correct distances
//============================================================================================================================

/**
 * Drive forward X distance, (does not reposition wheels)
 * if distance is negative => go backwards
 * #Support method for commandDriveFoward
 * 
 * @param distance: target distance we want to rover to travel
 */
void driveForward(int distance){
  float distanceTraveled = 0;
  int direction = FOWARD;
  //if distance is negative => go backwards
  if(distance < 0){
    direction = BACKWARD;
  }
  activeDriveForward(direction, driveSpeed);

  //if we are not within the driveError of the distance, we keep driving
  while((abs(distance) - distanceTraveled) > 0/*driveError*/){
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
 * #Support method for driveFoward
 * 
 * @param direction:  FOWARD(clockwise) or BACKWARD(counter-clock)
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
 * #Support method for driveFoward
 */
void stopAllDrive(){
  activeDriveForward(0, 0);
}

//============================================================================================================================
// Turn Methods, used to allow Rover to turn itself the correct distance
//============================================================================================================================

/**
 * Turn Rover Right X degrees
 * if degrees is negative => go backwards
 * #Support Method for commandTurnRover
 * 
 * @param:  degrees, target degrees we want to rover to turn
 */
void turnRover(int degrees){
  float distanceTraveled = 0;
  int direction = FOWARD;
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
 * @param direction:  FOWARD(clockwise) or BACKWARD(counter-clock)
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

//============================================================================================================================
// Digger/Dumper Methods, used to allow Rover to Dig and Dump correct measurments
//============================================================================================================================

/**
 * drives one wheel forward to a given distance (Safe to run)
 * #Support Method for commandDigger and commandDumper
 * 
 * @param ID:       the id of the drive wheel we want to move
 * @param distance: the distance we want it to travel
 */
void driveOneForward(int ID, int distance, int speed){
  float distanceTraveled = 0;
  int direction = FOWARD;
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

//============================================================================================================================
// Articulation Methods, used to allow Rover's articuation wheels to point in specific directions
//============================================================================================================================

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
  
  //while we are not done turning all motors
  while(done == false){
    //if we encounter no unfinished turning wheels we remain "done"
    done = true;
    
    //go through all 6 articulation motors
    for(int i = 6; i<12; i++){
      //sets the articulation of each motor to turn towards the correct degree
      isFinished = setArticulation(i, newDegrees[i-6]);
      
      //if ANY motor isn't finished we are not done
      if(isFinished == false){
        done = false;
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
boolean setArticulation(int ID, int degrees){
  //update articulation
  int articulation = articulationRead(ID-6);
  
  //if the motor is already within the turnError stop
  if(abs(articulation/*(ID-6)*/  - degrees) < turnError){
    stopMotor(ID);
    stopMotor(ID-6);//------------------------------------------------------------------------------
    return true;
  }
  //if the motor is too far, go back
  else if(articulation/*(ID-6)*/ > degrees){
    runMotor(ID, BACKWARD, turnSpeed);
    runMotor(ID-6, BACKWARD, (int)(turnSpeed*artHelperSpeed));//-----------------------------------------------------
    return false;
  }
  //if the motor isn't far enough, go forward
  else{
    runMotor(ID, FOWARD, turnSpeed);
    runMotor(ID-6, FOWARD, (int)(turnSpeed*artHelperSpeed));//------------------------------------------------------
    return false;
  }
}

/**
 * Returns the current Articulation of a given articulationWheelID //================================================= // ======================================================== \\
 * "analogRead()" returns an int of the degrees a articulation sensor is turned (from 0 to 1024)                       // <<<(This will have to be changed if encoders changed)>>> \\
 * "A0" "A1" ... "" is based off what "anolog in" wire slot the articulation on the Articulation sensor is plugged into// ======================================================== \\
 * #Support Method for setArticulation
 * 
 * @param articulationWheelID: the articulation wheel we want to read it's analog sensor
 */
int articulationRead(int articulationWheelID){
  switch(articulationWheelID){
    case 0: return analogRead(A0);
      break;
    case 1: return analogRead(A0);
      break;
    case 2: return analogRead(A0);
      break;
    case 3: return analogRead(A0);
      break;
    case 4: return analogRead(A0);
      break;
    case 5: return analogRead(A0);
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
    newDegrees[ID-6] = wheelTrueZero[ID] + degrees[ID-6];
  }
}

//============================================================================================================================
// Misc Methods, used to allow Other Methods in use, [runMotor, keepGoing, stopMotor(s), simpleDelay, checkForPause, ext.]
//============================================================================================================================

/**
 * runs a motor given motorID, direction, and speed
 * 
 * @param ID: the wheelID[] position
 * @param commandInt:  the direction we want to rotate the wheel
 *              0: REVERSE(counter-clock), 1: FOWARD(clockwise)
 * @param speed: the speed you want it to rotate
 *              0: Stopped, 
 */
void runMotor(int ID, int commandInt, int speed){
  //If we are Pauseed, do not allow any Motor to run, only stop motor
  if(pause && speed != 0){return;}
  
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
  
  //checksum is an important variable/line of code for moving the motor
  unsigned char checksum = (address + command + ((char)speed)) & 0b01111111;
  
  // Write to the correct serial packet.
  if(wheelSerial[ID] == 1){
    //This is the 4 lines of code that ACTUALY moves the motor, do not mess with
    Serial1.write(address);
    Serial1.write(command);
    Serial1.write(((char)speed));
    Serial1.write(checksum);
  }
  else if(wheelSerial[ID] == 2){
    Serial2.write(address);
    Serial2.write(command);
    Serial2.write(((char)speed));
    Serial2.write(checksum);
  }
  else if(wheelSerial[ID] == 3){
    Serial3.write(address);
    Serial3.write(command);
    Serial3.write(((char)speed));
    Serial3.write(checksum);
  }
  //so we don't overload any serial buffers
  simpleDelay();
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
 * Sets the driveSpeed and adjusts the values of driveError
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
 * Sets the turnSpeed and adjusts the valuse of turnError
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
 * updates the Pause command,
 * #Support Method for checkForPause
 */
void updatePause(){
  //nh.spinOnce(); //ROS updates comunication //-----------------------------------------------------------------------------------------------
  commandSetPause(rpiPause);
  commandEStop(rpiEStop);
}

//============================================================================================================================
//Testing Methods (not used in final code)
//============================================================================================================================

/**
 * Runs all Articulation Motors (For Testing purposes ONLY)
 * Should not be used in final code (Will rip out Wires)
 * 
 * @param command:  direction we want to go
 * @param speed:    the speed we want the articulation motors to turn
 */
void runAllArticulation(int command, int speed){
  for(int i = 6; i < 12; i++){
    runMotor(i, command, speed);
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
  int newOneDegree = degrees + wheelTrueZero[ID];
  while(done == false){
    done = setArticulation(ID, newOneDegree);
  }
}

/**
 * runs all Drive Motors, All 6 drive wheels turn (clockwise/counter-clockwise) uniformly
 * 
 * @param direction:  FOWARD(clockwise) or BACKWARD(counter-clock)
 * @param speed:      The speed you want all 6 drive motors to turn
 */
void runAllDriveUniform(int direction, int speed){
  //set all drive motors to forward
  for(int i = 0; i < 6; i++){
    runMotor(i, direction, speed);
  }
}

//Causes the code to stop forever
void eternalSleep(){
  stopAllMotor();
  while(true){
    delayMicroseconds(15000);
  }
}

/**
 * Extra Pause method, for people who just want to Stop
 */
//void callPause(){ 
//  commandPause(true); 
//}




