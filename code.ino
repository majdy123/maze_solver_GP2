// naser tayher pass from here 



#include <NewPing.h>

#define TRIGGER_PINL  A3  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINL     A0  // Arduino pin tied to echo pin on ping sensor.

#define MAX_DISTANCE 100 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

#define TRIGGER_PINF  A4  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINF     A1  // Arduino pin tied to echo pin on ping sensor.

#define TRIGGER_PINR  A5  // Arduino pin tied to trigger pin on ping sensor.
#define ECHO_PINR     A2  // Arduino pin tied to echo pin on ping sensor.





int dir;


#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4



float P = 0.5 ;
float D = 0.5 ;
float I = 0.5 ;
float oldErrorP ;
float totalError ;
int offset = 5 ;

int wall_threshold = 8 ;
//int left_threshold = 10 ;
//int right_threshold = 10 ;
int front_threshold = 10 ;

boolean frontwall ;
boolean leftwall ;
boolean rightwall ;
boolean first_turn ;
boolean rightWallFollow ;
boolean leftWallFollow ;




float leftarray[5];
float rightarray[5];
float frontarray[5];
int array_counter = 0;

int en1 = 2 ;
int en2 = 3 ;

int en3 = 4 ;
int en4 = 5 ;

int enA = 10 ;
int enB = 11 ;

int baseSpeed = 0 ;

int RMS ;
int LMS ;

int LED = 13 ;
int led1 = 8 ;
int led2 = 9 ;
int t1,t2,t3;


NewPing sonarLeft(TRIGGER_PINL, ECHO_PINL, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarRight(TRIGGER_PINR, ECHO_PINR, MAX_DISTANCE);
NewPing sonarFront(TRIGGER_PINF, ECHO_PINF, MAX_DISTANCE);

unsigned int pingSpeed = 30; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.


float oldLeftSensor, oldRightSensor, leftSensor, rightSensor, frontSensor, oldFrontSensor, lSensor, rSensor, fSensor;

//int TestNUM = 1  ;



void setup() {

  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.


  for (int i = 2; i <= 13; i++) //For Motor Shield
    pinMode(i, OUTPUT);



  first_turn = false ;
  rightWallFollow = false ;
  leftWallFollow = false ;
 pinMode(TRIGGER_PINL,OUTPUT); 
  pinMode(ECHO_PINL,INPUT); 
  pinMode(TRIGGER_PINF,OUTPUT); 
  pinMode(ECHO_PINF,INPUT); 
  pinMode(TRIGGER_PINR,OUTPUT); 
  pinMode(ECHO_PINR,INPUT);

}

void loop() {


  //Reading the sensors Data.
  ReadSensors();

  walls();


  if ( first_turn == false ) {

    pid_start();

  }
  else if (leftWallFollow == true ) {

    PID(true) ;

  }
  else if (rightWallFollow == true ) {
    PID(false) ;
  }


  if (leftwall == true && rightwall == false && frontwall == true ) {

    // turnright();
    PID(false) ;

    if ( first_turn == false ) {

      //      right_threshold = right_threshold - offset ;
      //      left_threshold = left_threshold + offset ;


      first_turn = true ;
      rightWallFollow = true ;
      
      digitalWrite(led2 , LOW );
      digitalWrite(led1 ,HIGH );
    }
  }
   if (leftwall == false && rightwall == true && frontwall == true ) {

    //  turnleft();
    PID(true) ;

    if ( first_turn == false ) {

      //      left_threshold = left_threshold - offset ;
      //      right_threshold = right_threshold + offset ;

      first_turn = true ;
      leftWallFollow = true ;
      digitalWrite(LED , HIGH);
       
    }
  }
   if ( leftSensor == 0 || leftSensor > 100 && rightSensor == 0 || rightSensor > 100 && frontSensor == 0 || frontSensor > 100 ) {

    setDirection(STOP);
  }



  // read sensors & print the result to the serial monitor //


  // Serial.print(" Left : ");
  // Serial.print(leftSensor);
  // Serial.print(" cm ");
  // Serial.print(" Right : ");
  // Serial.print(rightSensor);
  // Serial.print(" cm ");
  // Serial.print(" Front : ");
  // Serial.print(frontSensor);
  // Serial.println(" cm ");
  Serial.print("   left-wall : ");
  Serial.print(leftwall);
  Serial.print("   front-wall : ");
  Serial.print(frontwall);
  Serial.print("   right-wall : ");
  Serial.println(rightwall);
  // Serial.print(" left motor speed : ");
  // Serial.print(LMS);
  // Serial.print(" right motor speed : ");
  // Serial.println(RMS);

//delay(1000);

  //measure error & print the result to the serial monitor
  //Serial.print("error=");
  //Serial.println(totalError);


}

//--------------------------------- direction control ---------------------------------//

void setDirection(int dir) {

  if ( dir == FORWARD ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, LOW);  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == LEFT ) {
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, LOW );
    digitalWrite(en3, LOW );  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == RIGHT ) {
    digitalWrite(en1, LOW);   // Left wheel forward
    digitalWrite(en2, HIGH);
    digitalWrite(en3, HIGH);  // Right wheel forward
    digitalWrite(en4, LOW);
  }
  else if ( dir == STOP ) {
    digitalWrite(en1, HIGH);   // Left wheel forward
    digitalWrite(en2, HIGH );
    digitalWrite(en3, HIGH );  // Right wheel forward
    digitalWrite(en4, HIGH);
  }
  else if ( dir == BACKWARD ) {
    digitalWrite(en1, HIGH );   // Left wheel forward
    digitalWrite(en2, LOW );
    digitalWrite(en3, HIGH );  // Right wheel forward
    digitalWrite(en4, LOW );
  }
}
//---------------------------------------------------------------------------//


//--------------------------------- Sensors ---------------------------------//

void ReadSensors() {

  digitalWrite(TRIGGER_PINL,LOW); 
  delayMicroseconds(5); 
  digitalWrite(TRIGGER_PINL,HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TRIGGER_PINL,LOW); 
  t1 = pulseIn(ECHO_PINL,HIGH); 
  lSensor = t1 / 60 ; 

  
  digitalWrite(TRIGGER_PINF,LOW); 
  delayMicroseconds(5); 
  digitalWrite(TRIGGER_PINF,HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TRIGGER_PINF,LOW); 
  t2 = pulseIn(ECHO_PINF,HIGH); 
  fSensor = t2 / 57 ; 


  digitalWrite(TRIGGER_PINR,LOW); 
  delayMicroseconds(5); 
  digitalWrite(TRIGGER_PINR,HIGH); 
  delayMicroseconds(10); 
  digitalWrite(TRIGGER_PINR,LOW); 
  t3 = pulseIn(ECHO_PINR,HIGH); 
  rSensor = t3 / 60 ; 

 
  if(array_counter == 5){
    leftSensor  = (leftarray[0]  + leftarray[1]  + leftarray[2]  + leftarray[3]  +  leftarray[4])/5;
    rightSensor = (rightarray[0] + rightarray[1] + rightarray[2] + rightarray[3] + rightarray[4])/5;
    frontSensor = (frontarray[0] + frontarray[1] + frontarray[2] + frontarray[3] + frontarray[4])/5;
    array_counter=0;
  }else{
    leftarray[array_counter] =  lSensor;
    rightarray[array_counter] = rSensor;
    frontarray[array_counter] = fSensor;
    array_counter = array_counter + 1;
  }
  //leftSensor = (lSensor + oldLeftSensor) / 2; //average distance between old & new readings to make the change smoother
  //rightSensor = (rSensor + oldRightSensor) / 2;
  //frontSensor = (fSensor + oldFrontSensor) / 2;


  //oldLeftSensor = leftSensor; // save old readings for movment
  //oldRightSensor = rightSensor;
  //oldFrontSensor = frontSensor;

  
}

//---------------------------------------------------------------------------//


//--------------------------------- control ---------------------------------//

void pid_start() {

  float errorP = leftSensor - rightSensor ; // it's a measure of how off-center the robot is with respect to the desired path.
  float errorD = errorP - oldErrorP; // indicating how quickly the robot is moving away from the desired path.
  float errorI = (2.0 / 3.0) * errorI + errorP ;

  totalError = P * errorP + D * errorD + I * errorI ;
  
  oldErrorP = errorP ;

  RMS = baseSpeed + totalError ;
  LMS = baseSpeed - totalError ;

  if (RMS < 0) {

    RMS = map(RMS , 0 , -255, 0, 255);

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(RIGHT);

  }
  else if (LMS < 0) {
    LMS = map(LMS , 0 , -255, 0, 255);


    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(LEFT);
  }
  else {

    analogWrite(enA , RMS);
    analogWrite(enB , LMS);

    setDirection(FORWARD);
  }

}


//----------------------------- wall follow  control -------------------------------//

void PID( boolean left ) {

  if (left == true ) {

    float errorP = leftSensor - rightSensor - offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;


    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

    //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
    //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;


    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }
  else {

    float errorP = leftSensor - rightSensor + offset ;
    float errorD = errorP - oldErrorP;
    float errorI = (2.0 / 3) * errorI + errorP ;

    totalError = P * errorP + D * errorD + I * errorI ;

    oldErrorP = errorP ;


    RMS = baseSpeed + totalError ;
    LMS = baseSpeed - totalError ;

    //  if(RMS < -255) RMS = -255; if(RMS > 255)RMS = 255 ;
    //  if(LMS < -255) LMS = -255;  if(LMS > 255)LMS = 255 ;


    if (RMS < 0) {

      RMS = map(RMS , 0 , -255, 0, 255);

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(RIGHT);

    }
    else if (LMS < 0) {
      LMS = map(LMS , 0 , -255, 0, 255);


      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(LEFT);
    }
    else {

      analogWrite(enA , RMS);
      analogWrite(enB , LMS);

      setDirection(FORWARD);
    }

  }

}

//--------------------------- wall detection --------------------------------//

void walls() {


  if ( leftSensor < wall_threshold ) {
    leftwall = true ;
  }
  else {
    leftwall = false ;
  }


  if ( rightSensor < wall_threshold ) {
    rightwall = true ;
  }
  else {
    rightwall = false ;


  } if ( frontSensor < front_threshold ) {
    frontwall = true ;
  }
  else {
    frontwall = false ;
  }

}



//---------------------------------------------------------------------------//

void turnright() {


  LMS = baseSpeed ;

  RMS = LMS * rightSensor / ( rightSensor + 11 ) ;


}

//---------------------------------------------------------------------------//

void turnleft() {


  RMS = baseSpeed ;

  LMS = RMS * leftSensor / ( leftSensor + 11 ) ;

}


//---------------------------------------------------------------------------//




