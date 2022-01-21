// Code for 4 wheel base robot, with added functionality

#include<Servo.h>
#include<IRremote.h>

#define Interrupt    3            //pin for obstacle sensor

volatile bool trigger = LOW;
volatile bool interruptTrig = LOW;      

#define Right_Front   12
#define Right_Back    11
#define Left_Front    7
#define Left_Back     8

//define speed variable
#define Right_Speed   5
#define Left_Speed    6

//button Codes
#define ADVANCE       0x00FF18E7       //code from IR controller "^" button
#define BACK          0x00FF4AB5       //code from IR controller "v" button
#define RIGHT         0x00FF5AA5       //code from IR controller ">" button
#define LEFT          0x00FF10EF       //code from IR controller "<" button
#define STOP_         0x00FF38C7       //code from IR controller "OK" button
#define INCREASE      0x00FFB04F       //code from IR controller "#" button
#define DECREASE      0x00FF6897       //code from IR controller "*" button

#define ENTIRE_STOP   0x00FF02FD       //code from IR controller "5" button

#define MAX_POWER     0x00FFA857       //code from IR controller "8" button
#define SLOW_CAR      0x00FF9867       //code from IR controller "0" button

#define AUTOMATIC     0x00FFA25D       //code from IR controller "1" button
#define MANUAL        0x00FFE21D       //code from IR controller "3" button

//IR PIN
#define IR_PIN 4
  IRrecv IR(IR_PIN);    //gets code from IR remote
  decode_results results;

//set speed for motors
int SPEED = 135;

//variable to determine the last command
unsigned long LastCommand;

bool manual_mode = true;              //true if car is in manual mode, false if it isn't
bool auto_mode = true;                //true if car is in auto mode, false if it isn't

bool auto_gear = false;               //false if auto gear is not enabled, true if enabled


//enum is called in the Drivetick function
//When the Drive_Num is equal to one if these
//the case will call the function 
enum Funct{
  GO_ROBOT,
  GO_LEFT,
  GO_RIGHT,
  GO_BACK,
  INC_SPEED,
  DEC_SPEED,
  STOP_BOT,
  STOP_COMPLETELY,
  MAX,
  SLOW_DOWN,
  CAR_AUTO,
  DEF
}

Drive_Num = DEF;

/*************** Automatic Functions *******************/
Servo auto_look;

byte trig = 10;
byte echo = 2;
int maxDist = 150;                               //Maximum sensing distance (Objects further than this distance are ignored)
int stopDist = 50;                               //Minimum distance from an object to stop in cm
float timeOut = 2*(maxDist+10)/100/340*1000000;   //Maximum time to wait for a return signal

int motorSpeed = 135;                             //speed of car, different from SPEED
int turnSpeed = 60;                               //Amount to add to motor speed when turning


/*********************** set_speed() universal function ***********************/
void set_speed(int left, int right){
  digitalWrite(Left_Speed, left);
  digitalWrite(Right_Speed, right);
  }

void stop_bot(){
  digitalWrite(Right_Front, LOW);
  digitalWrite(Right_Back, LOW);
  digitalWrite(Left_Front, LOW);
  digitalWrite(Left_Back, LOW);
  }

/*********************** Functions Manual Control ***********************/

//robot will go forward
void forward(){
  digitalWrite(Right_Front, HIGH);
  digitalWrite(Right_Back, LOW);
  digitalWrite(Left_Front, HIGH);
  digitalWrite(Left_Back, LOW);
  set_speed(SPEED, SPEED);
  }

//will go left
void go_left(int t=0){
  digitalWrite(Right_Front, HIGH);
  digitalWrite(Right_Back, LOW);
  digitalWrite(Left_Front, LOW);
  digitalWrite(Left_Back, HIGH);
  set_speed(SPEED, SPEED);
  delay(t);
}

//will go right
void go_right(int t=0){
  digitalWrite(Right_Front, LOW);
  digitalWrite(Right_Back, HIGH);
  digitalWrite(Left_Front, HIGH);
  digitalWrite(Left_Back, LOW);
  set_speed(SPEED, SPEED);
  delay(t);
}

//will go backwards/reverse
void go_back(int t=0){
  digitalWrite(Right_Front, LOW);
  digitalWrite(Right_Back, HIGH);
  digitalWrite(Left_Front, LOW);
  digitalWrite(Left_Back, HIGH);
  set_speed(SPEED, SPEED);
  delay(t);
}

//increases the speed of the robot by increments of 15
void increase_speed(int t=0){
  Serial.println("INC");
  if(SPEED >= 0 && SPEED < 255){
    SPEED = SPEED + 15;
    set_speed(SPEED, SPEED);
    Serial.println(SPEED);
    delay(t);
  }
  else{
    Serial.println("Can't go any faster");
    Serial.println(SPEED);
  }
}

//decrements the speed of the robot by 15 until a complete stop
void decrease_speed(int t=0){
  Serial.println("DEC");
  if(SPEED > 0){
    SPEED = SPEED - 15;
    set_speed(SPEED, SPEED);
    Serial.println(SPEED);
    delay(t);
  }
  else{
    Serial.println("Car is already stopped");
  }
}

//will make robot go the max speed, which is 255
//by incrementing the current SPEED until it reaches 255
void max_speed(){
  Serial.print("MAX");
  for(int i = SPEED; i < 255; i++){
    set_speed(i,i);
    delay(20);
  }
  SPEED = 255;
}

//will decrement the SPEED of the robot until it reaches 1,
//then it will stop and change SPEED to 0
void decelerate(){
  //function that decelerates the car until it fully stops
  Serial.println("SLOW DOWN");
  for(int i = SPEED; i!= 0; i--){
    set_speed(i,i);
    delay(20);
  }
  SPEED = 0;
  stop_bot();
}

/*********************** Automatic Functions ***********************/
//function to accelerate the robot when in automatic mode
void auto_accelerate(){
  for(int i=0; i < motorSpeed; i++){
    set_speed(i,i);
    delay(10);
  }
}

//will make the robot slow down until it reaches a complete stop
void auto_decelerate(){
  for(int i=motorSpeed; i!=0; i--){
    set_speed(i,i);
    delay(10);
  }
}

//will make the robot turn left a certain amount
void turnLeft(int duration){
  int r_left= motorSpeed+turnSpeed;                 //create variables to look neater for set_SPEED function
  int r_right= motorSpeed+turnSpeed;
  set_speed(r_left, r_right);
  auto_left();
  delay(duration);
  int f_left = motorSpeed;                          //made variables to look neater
  int f_right = motorSpeed;
  set_speed(f_left, f_right);
  stop_bot();
}

//robot will turn right a certain amount
void turnRight(int duration){
  int r_left = motorSpeed+turnSpeed;                //create variables to look neater for set_SPEED function
  int r_right = motorSpeed+turnSpeed;
  set_speed(r_left, r_right);
  auto_right();
  delay(duration);                                  //delays by given time
  int f_left = motorSpeed;                          //made variables to look neater
  int f_right = motorSpeed;
  set_speed(f_left, f_right);
  stop_bot();
}

void auto_left(){
  digitalWrite(Right_Front, HIGH);
  digitalWrite(Right_Back, LOW);
  digitalWrite(Left_Front, LOW);
  digitalWrite(Left_Back, HIGH);
}

void auto_right(){
  digitalWrite(Right_Front, LOW);
  digitalWrite(Right_Back, HIGH);
  digitalWrite(Left_Front, HIGH);
  digitalWrite(Left_Back, LOW);
}

void auto_forward(){
  digitalWrite(Right_Front, HIGH);
  digitalWrite(Right_Back, LOW);
  digitalWrite(Left_Front, HIGH);
  digitalWrite(Left_Back, LOW);
  set_speed(motorSpeed, motorSpeed);
}

volatile bool object_in_path = false;

//will get the distance an object is from the car
int getDistance(){
  unsigned int pulseTime;                           //create a variable to store the pulse travel time
  int distance;                                     //create a variable to store the calculated distance
  digitalWrite(trig, HIGH);                         //generate 10 microsecond pulse
  delayMicroseconds(10);            
  digitalWrite(trig, LOW);
  pulseTime = pulseIn(echo, HIGH, timeOut);         //measure the time for pulse to return
  distance = (float)pulseTime * 340 / 2 / 10000;    //calculate the object distance based on the pulse time

  if(digitalRead(Interrupt) == trigger){
    Serial.println("Object in path");
    object_in_path = true;
  }
  else{
    object_in_path = false;
  }
  return distance;  
}

//will make servo turn certain directions so that it can get the distance of objects in its path
//will make a judgment based on hwo far an object is from the robot, and if the directions is clear

int checkDirection(){
  int distances[2] = {0,0};                                 //left and right distances
  int turnDir = 1;                                          //Direction to turn, 0 left, 1 reverse, 2 right
  auto_look.write(180);                                          //turn servo to look left
  delay(500);
  distances[0] = getDistance();                           //get left object distance
  auto_look.write(0);                                            //turn servo to look right
  delay(1000);  
  distances[1] = getDistance();                           //get right object distance
  
  if(distances[0] >= 200 && distances[1] >= 200)            //if both directions are clear, turn left
    turnDir = 0;
  else if(distances[0]<=stopDist && distances[1]<=stopDist) //if both directions are blocked, turn around
    turnDir = 1;
  else if(distances[0] >= distances[1])                     //if left has more space, turn left
    turnDir = 0;
  else if(distances[0] < distances[1])                      //if right has more space, turn right
    turnDir = 2;

  return turnDir;
}

/*********************** Detects IR ***********************/

/*
  manual mode is checked in every case since it uses the ir more
  auto_mode bool will be checked in each case to see if it is true or false
  ENTIRE_STOP will completely stop the robot and the user will be able to change between manual and auto mode
  when auto_mode is true, that means the car is now in automatic
  same for manual_mode 
*/
void IR_tick(){
  if(IR.decode(&results)){
    if(results.value == ADVANCE && manual_mode == true && auto_mode == false){
      auto_mode = false;
      
      LastCommand = results.value;
      Drive_Num = GO_ROBOT;
    }
    else if(results.value == LEFT && manual_mode == true && auto_mode == false){
      auto_mode = false;
      
      LastCommand = results.value;
      Drive_Num = GO_LEFT;
    }
    else if(results.value == RIGHT && manual_mode == true && auto_mode == false){
      auto_mode = false;
      
      LastCommand = results.value;
      Drive_Num = GO_RIGHT;
    }
    else if(results.value == BACK && manual_mode == true && auto_mode == false){
      auto_mode = false;
      
      LastCommand = results.value;
      Drive_Num = GO_BACK;
    }
    else if(results.value == INCREASE && manual_mode == true && auto_mode == false){
      auto_mode = false;
      
      LastCommand = results.value;
      Drive_Num = INC_SPEED;
    }
    else if(results.value == DECREASE && manual_mode == true && auto_mode == false){
      auto_mode = false;
      
      LastCommand = results.value;
      Drive_Num = DEC_SPEED;
    }
    else if(results.value == STOP_ && manual_mode == true && auto_mode == false){
      auto_mode = false;

      LastCommand = results.value;
      Drive_Num = STOP_BOT;
    }
    else if(results.value == ENTIRE_STOP){
      auto_mode = true;
      manual_mode = true;

      auto_gear = false;
      
      LastCommand = results.value;
      Drive_Num = STOP_COMPLETELY;
    }
    else if(results.value == MAX_POWER && manual_mode == true && auto_mode == false){
      auto_mode = false;
      
      LastCommand = results.value;
      Drive_Num = MAX;
    }
    else if(results.value == SLOW_CAR && manual_mode == true && auto_mode == false){
      auto_mode = false;
      
      LastCommand = results.value;
      Drive_Num = SLOW_DOWN;
    }
    else if(results.value == AUTOMATIC && auto_mode == true){
      auto_gear = true;
      manual_mode = false;
      auto_mode = true;
      
      Drive_Num = CAR_AUTO;
      LastCommand = results.value;
    }
    else if(results.value == MANUAL && manual_mode == true){
      auto_mode = false;
      auto_gear = false;
      
      manual_mode = true;
      
      LastCommand = results.value;
      Serial.println("MANUAL");
    }
    else if(results.value == 0xFFFFFFFF){
      results.value = LastCommand;
    }
    results.value = 0;
    IR.resume();
  }
}

/********************** CAR CONTROL ***********************/

//Drive_Num will be checked for in each case, and will call the function that corresponds to it
void Drive_tick(){
  switch(Drive_Num){
    case GO_ROBOT:
      Serial.println("F");
      forward();
      break;
    case GO_LEFT:
      Serial.println("L");
      go_left();
      break;
    case GO_RIGHT:
      Serial.println("R");
      go_right();
      break;
    case GO_BACK:
      Serial.println("BACK");
      go_back();
      break;
    case INC_SPEED:
      increase_speed();
      break;
    case DEC_SPEED:
      decrease_speed();
      break;
    case STOP_BOT:
      Serial.println("STOP");
      stop_bot();
      break;
    case STOP_COMPLETELY:
      Serial.println("STOPPING ENTIRELY");
      stop_bot();
      break;
    case MAX:
      max_speed();
      break;
    case SLOW_DOWN:
      decelerate();
      break;
    default:
      break;
  }
  Drive_Num = DEF;
}

/*********************** Functions for each mode ***********************/

void servo_loop(){
  Serial.println("AUTO");
  auto_look.write(90);                                    //set servo to look straight ahead
  delay(750);
  
  int distance = getDistance();                           //calls function to check if there is an object in the way
  
  if(distance >= stopDist){                               //if there is no object within stop distance, move forward
    auto_forward();
  }
  
  while(distance >= stopDist){                            //while loop will keep checking the object distance
    distance = getDistance();                             //until it is within the minimum stopping distance
    delay(250);
  }

  stop_bot();
  int turn_Direction = checkDirection();

  switch(turn_Direction){             
    case 0:
      turnLeft(400);                                      //turns left
      break;
    case 1:
      turnLeft(800);                                      //turns around
      break;
    case 2:          
      turnRight(400);                                     //turns right
      break;
  }
}

/************************** Car Setup **************************/

//will stop the car when the obstacle sensor is triggered, whether it be in manual or automatic mode
void interrupt_trigger(){
  interruptTrig = HIGH;
  while(1)
  {
    stop_bot();
    turnLeft(800);
    object_in_path = false;
    Serial.println("TRIGGERED");
    break;
  }
}

void setup() {
  Serial.begin(9600);
  
  pinMode(Right_Front, OUTPUT);
  pinMode(Right_Back, OUTPUT);
  pinMode(Left_Front, OUTPUT);
  pinMode(Left_Back, OUTPUT);

  pinMode(Right_Speed, OUTPUT);
  pinMode(Left_Speed, OUTPUT);

  stop_bot();

  //IR
  pinMode(IR_PIN, INPUT);
  digitalWrite(IR_PIN, HIGH);
  IR.enableIRIn();

  //Servo
  auto_look.attach(9);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);

  pinMode(Interrupt, INPUT_PULLUP);
 
  attachInterrupt(digitalPinToInterrupt(Interrupt), interrupt_trigger, HIGH);

}

void loop() {
  //will check when auto_gear is changed
  //if true, car is in auto_mode
  //if false, car is in manual mode
  //default state of car is stop_bot function, stop state
  switch(auto_gear){
    case true:
      servo_loop();
    case false:
      IR_tick();
      Drive_tick();
  }
}
