#include <PID_v1.h>

// ME450 Steering Rig Motor Controller, Team 16
// set two desired positions
double targetPos1=0;
double targetPos2=0;

// initialize the variables
double currentPos1 = 0;
double currentPos2 = 0;
double n1 = 0;
double n2 = 0;

// Ram for PD control
long currentPos1Last=0;
long currentPos2Last=0;

// to measure time
unsigned long oldTime;
unsigned long newTime;
unsigned long timeDelay;

// Encoder
const int leftEncoderPinA = 2;
const int leftEncoderPinB = 3;
const int rightEncoderPinA = 4;
const int rightEncoderPinB = 5;
int leftEncoderPinALast = LOW;
int rightEncoderPinALast = LOW;
int encoderCount1 = LOW;
int encoderCount2 = LOW;
double velocity1=0;
double velocity2=0;
// set all to 0 to tune
double Kp=0.3; // increase until response is steady oscillation
double Kd=0.01; // increase until oscillations go away
// keep changing Kp and Kd until increasing Kd doesn't stop oscillations
// set Kp and Kd to last stable values
double Ki=0; // increase until it gets you to target
double base1Cmd=30; // value that makes motor just start to rotate
double base2Cmd=30; // value that makes motor just start to rotate
double velocity1Cmd;
double velocity2Cmd;

// PID Motor Controller
const int leftpwmPin = 10;
const int lefti1Pin = 8;
const int lefti2Pin = 9;
const int rightpwmPin = 11;
const int righti1Pin = 12;
const int righti2Pin = 13;
int direct1=DIRECT;
int direct2=DIRECT;
PID leftPID(&currentPos1, &velocity1Cmd, &targetPos1, Kp, Ki, Kd, DIRECT);
PID rightPID(&currentPos2, &velocity2Cmd, &targetPos2, Kp, Ki, Kd, DIRECT);

// Switch
const int rightSwitch = A0;
const int leftSwitch = A1;
int rightNew;
int leftNew;
int rightOld;
int leftOld;

void setup(){
  // declare which digital pins are inputs and which are outputs
  pinMode(leftpwmPin,OUTPUT);
  pinMode(lefti1Pin,OUTPUT);
  pinMode(lefti2Pin,OUTPUT);
  pinMode(rightpwmPin,OUTPUT);
  pinMode(righti1Pin,OUTPUT);
  pinMode(righti2Pin,OUTPUT);

  // set initial speed of motor to 0
  analogWrite(leftpwmPin,0);
  analogWrite(rightpwmPin,0);

  // turn the PID on
  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);

  // begin serial communication for display of variable states
  Serial.begin(115200);
  Serial.println("start");
}

void loop(){
  // determine time interval
  oldTime = newTime; 
  newTime = millis();  //elapsed time since Arduino began running the program 
  timeDelay = newTime - oldTime;
  if(timeDelay==0)
  {
    timeDelay=140;
  }
  // increment target position as switch is held
  rightOld = rightNew;
  leftOld = leftNew;
  rightNew = digitalRead(rightSwitch);
  leftNew = digitalRead(leftSwitch);
  if(rightNew && rightOld == rightNew)
  {
    n1-=1;
    n2-=1;
  }
  else if(leftNew && leftOld == leftNew)
  {
    n1+=1;
    n2+=1;
  }
  if(n1 >= 20)
  {
    n1 = 20;
  }
  else if(n1 <= -20)
  {
    n1 = -20;
  }
  if(n2 >= 20)
  {
    n2 = 20;
  }
  else if(n2 <= -20)
  {
    n2 = -20;
  }
  targetPos1 = round(n1);
  targetPos2 = round(n2);
  // determing encoder count change
  currentPos1Last=currentPos1;
  encoderCount1 = digitalRead(leftEncoderPinA);
  if ((leftEncoderPinALast == LOW) && (encoderCount1 == HIGH)) {
    if (digitalRead(leftEncoderPinB) == LOW) {
      currentPos1--;
    } else {
      currentPos1++;
    }
  } 
  leftEncoderPinALast = encoderCount1;
  currentPos2Last=currentPos2;
  encoderCount2 = digitalRead(rightEncoderPinA);
  if ((rightEncoderPinALast == LOW) && (encoderCount2 == HIGH)) {
    if (digitalRead(rightEncoderPinB) == LOW) {
      currentPos2--;
    } else {
      currentPos2++;
    }
  } 
  rightEncoderPinALast = encoderCount2;

  // determine speed
  velocity1=-velocity1*0.2+float(currentPos1-currentPos1Last)/float(timeDelay)*1000*1.2; //unit count/sec
  velocity2=-velocity2*0.2+float(currentPos2-currentPos2Last)/float(timeDelay)*1000*1.2; //unit count/sec

  if(currentPos1 == targetPos1)
  {
    velocity1Cmd=0;
//    if(targetPos == desiredPos2)
//    {
//      targetPos=desiredPos1;
//    }
//    else
//    {
//      targetPos=desiredPos2;
//    }
  }
  else
  {
    if (currentPos1 < targetPos1)
    {
      digitalWrite(lefti1Pin,HIGH);
      digitalWrite(lefti2Pin,LOW); // rotate backward
      direct1=DIRECT;
    }
    else
    {
      digitalWrite(lefti1Pin,LOW);
      digitalWrite(lefti2Pin,HIGH);  //rotate forward
      direct1=REVERSE;
    }
    leftPID.SetControllerDirection(direct1);
    leftPID.Compute();
    // if no PID set, this will calibrate the start-up current
    if(Kp==0 && Kd==0 && Ki==0)
    {
      velocity1Cmd=int((targetPos1-currentPos1)/abs(targetPos1-currentPos1)*base1Cmd);
    }
    // if PID set, this will compute the PID output signal
    else
    {
      velocity1Cmd = velocity1Cmd*(1+base1Cmd/abs(velocity1Cmd));
    }
    // so it doesn't generate more than maximum
    if(velocity1Cmd > 255)
    {
      velocity1Cmd = 255;
    }
  }
  if(currentPos2 == targetPos2)
  {
    velocity2Cmd=0;
//    if(targetPos == desiredPos2)
//    {
//      targetPos=desiredPos1;
//    }
//    else
//    {
//      targetPos=desiredPos2;
//    }
  }
  else
  {
    if (currentPos2 < targetPos2)
    {
      digitalWrite(righti1Pin,HIGH);
      digitalWrite(righti2Pin,LOW); // rotate backward
      direct2=DIRECT;
    }
    else
    {
      digitalWrite(righti1Pin,LOW);
      digitalWrite(righti2Pin,HIGH);  //rotate forward
      direct2=REVERSE;
    }
    rightPID.SetControllerDirection(direct2);
    rightPID.Compute();
    // if no PID set, this will calibrate the start-up current
    if(Kp==0 && Kd==0 && Ki==0)
    {
      velocity2Cmd=int((targetPos2-currentPos2)/abs(targetPos2-currentPos2)*base2Cmd);
    }
    // if PID set, this will compute the PID output signal
    else
    {
      velocity2Cmd = velocity2Cmd*(1+base2Cmd/abs(velocity2Cmd));
    }
    // so it doesn't generate more than maximum
    if(velocity2Cmd > 255)
    {
      velocity2Cmd = 255;
    }
  }
//  if(rightNew && rightOld != rightNew || leftNew && leftOld != leftNew)
//  {
    // display the current position, desired position, and signal on the serial monitor
    Serial.print("V1: ");
    Serial.print(velocity1);
    Serial.print("     CP1: ");
    Serial.print(currentPos1);
    Serial.print("     TP1: ");
    Serial.print(targetPos1);
    Serial.print("     Cmd1: ");
    Serial.print(velocity1Cmd);
    Serial.print("     T: ");
    Serial.print(timeDelay);
    if(direct1==DIRECT)
    {
      Serial.println("    D: F");
    }
    else if(direct1==REVERSE)
    {
      Serial.println("    D: R");
    }
    Serial.print("V2: ");
    Serial.print(velocity2);
    Serial.print("     CP2: ");
    Serial.print(currentPos2);
    Serial.print("     TP2: ");
    Serial.print(targetPos2);
    Serial.print("     Cmd2: ");
    Serial.print(velocity2Cmd);
    Serial.print("     T: ");
    Serial.print(timeDelay);
    if(direct2==DIRECT)
    {
      Serial.println("    D: F");
    }
    else if(direct2==REVERSE)
    {
      Serial.println("    D: R");
    }
//  }
  // send output signal to motor
  analogWrite(leftpwmPin, velocity1Cmd);
  analogWrite(rightpwmPin, velocity2Cmd);
}
