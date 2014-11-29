// ME450 Steering Rig Motor Controller, Team 16

// MEASURE TIME
unsigned long prevTime, currTime;

// ENCODER VARIABLES
const int leftEncoderPinA = 4;
const int leftEncoderPinB = 2;
const int rightEncoderPinA = 3;
const int rightEncoderPinB = 5;
double leftVel = 0, rightVel = 0;
// set all to 0 to tune
double Kp=10; // increase until response is steady oscillation
double Kd=5; // increase until oscillations go away
// keep changing Kp and Kd until increasing Kd doesn't stop oscillations
// set Kp and Kd to last stable values
double Ki=0; // increase until it gets you to target
double leftMinCurrent = 78; // value that makes motor just start to rotate
double rightMinCurrent = 78; // value that makes motor just start to rotate
double Iterm;
int leftPower, rightPower;
int leftPowerLast, rightPowerLast;
int leftPrevPos, rightPrevPos;
volatile long leftCurrPos, rightCurrPos;

// PID CONTROLLER VARIABLES
const int leftpwmPin = 10;
const int lefti1Pin = 9;
const int lefti2Pin = 8;
const int rightpwmPin = 11;
const int righti1Pin = 13;
const int righti2Pin = 12;
int direct1 = 1; //positive = forward, negative = backward
int direct2 = 1;

// ROCKER SWITCH VARIABLES
const int rightSwitch = A0;
const int leftSwitch = A1;
int leftNew, rightNew;
int leftOld, rightOld;
int leftTargPos, rightTargPos;
int leftTargPosLast, rightTargPosLast;
double increment1 = 0, increment2 = 0;

// STARTING FUNCTION, ONLY RUNS ONCE
void setup() {
  // declare digital pins that are inputs and outputs
  pinMode(leftpwmPin,OUTPUT);
  pinMode(lefti1Pin,OUTPUT);
  pinMode(lefti2Pin,OUTPUT);
  pinMode(rightpwmPin,OUTPUT);
  pinMode(righti1Pin,OUTPUT);
  pinMode(righti2Pin,OUTPUT);
  pinMode(leftEncoderPinA,INPUT);
  pinMode(leftEncoderPinB,INPUT);
  pinMode(rightEncoderPinA,INPUT);
  pinMode(rightEncoderPinB,INPUT);

  //turn on the pullup resistors for the encoder (no idea why)
  digitalWrite(leftEncoderPinA,HIGH);  
  digitalWrite(leftEncoderPinB,HIGH);
  digitalWrite(rightEncoderPinA,HIGH);  
  digitalWrite(rightEncoderPinB,HIGH);

  //set interrupt for encoder pins: 0 for pin 2, 1 for pin 3
  attachInterrupt(0,doEncoder1,CHANGE);
  attachInterrupt(1,doEncoder2,CHANGE);

  // set initial speed of motor to 0
  analogWrite(leftpwmPin,0);
  analogWrite(rightpwmPin,0);

  // begin serial communication for display of variable states
  Serial.begin(115200);
  Serial.println("start");
}

// THIS FUNCTION RUNS EVERY TIME THE ARDUINO IS ON, INDEFINITELY
void loop() {
  // increment target position as switch is held
  rightOld = rightNew;
  leftOld = leftNew;
  rightNew = digitalRead(rightSwitch);
  leftNew = digitalRead(leftSwitch);
  if(rightNew && rightOld == rightNew)
  {
    increment1-=.5;
    increment2-=.5;
  }
  else if(leftNew && leftOld == leftNew)
  {
    increment1+=.5;
    increment2+=.5;
  }
  else
  {
    increment1=0;
    increment2=0;
  }
  if(increment1 >= 20)
  {
    increment1 = 20;
  }
  else if(increment1 <= -20)
  {
    increment1 = -20;
  }
  if(increment2 >= 20)
  {
    increment2 = 20;
  }
  else if(increment2 <= -20)
  {
    increment2 = -20;
  }
  leftTargPos = round(increment1);
  rightTargPos = round(increment2);
  
  // print new target pos if it changed
  if(leftTargPosLast != leftTargPos)
  {
    Serial.print(leftTargPos);
    Serial.print('/');
    leftTargPosLast = leftTargPos;
  }
  if(rightTargPosLast != rightTargPos)
  {
    Serial.print(rightTargPos);
    Serial.print('\n');
    rightTargPosLast = rightTargPos;
  }
  
  // determing encoder count change
  leftPrevPos=leftCurrPos;
  rightPrevPos=rightCurrPos;

  // use PID to control output signals
  leftPower = PIDcontrol(leftTargPos, leftCurrPos, leftPrevPos, leftVel,
  leftMinCurrent, lefti1Pin, lefti2Pin, direct1);
  rightPower = PIDcontrol(rightTargPos, rightCurrPos, rightPrevPos, rightVel,
  rightMinCurrent, righti1Pin, righti2Pin, direct2);

  // print new current pos or output current if it changed
  if(leftPrevPos != leftCurrPos)
  {
    Serial.print("L:");
    Serial.print(leftCurrPos);
    Serial.print('/');
  }
  if(rightPrevPos != rightCurrPos)
  {
    Serial.print("R:");
    Serial.print(rightCurrPos);
    Serial.print('/');
  }
  if(rightPowerLast != rightPower)
  {
    Serial.print("Pow:");
    Serial.print(rightPower);
    Serial.print('/');
    rightPowerLast = rightPower;
  }
  
  // send output signal to motor
  analogWrite(leftpwmPin, leftPower);
  analogWrite(rightpwmPin, rightPower);
}

// Start of Interrupt routine for encoder #1
// Interrupts can run any time during the loop
void doEncoder1() {
  if (digitalRead(leftEncoderPinA) == HIGH) {       // test for a low-to-high on channel A
    if ( digitalRead(leftEncoderPinB) == LOW ) {     // check channel B to see which way encoder is turning
      leftCurrPos++;
    }
    else {
      leftCurrPos--;
    }
  }
  else {                                        // it was a high-to-low on channel A
    if ( digitalRead(leftEncoderPinB) == HIGH ) {    // check channel B to see which way encoder is turning
      leftCurrPos++;
    }
    else {
      leftCurrPos--;
    }
  }
}

// Start of Interrupt routine for encoder #2
void doEncoder2() {
  if (digitalRead(rightEncoderPinA) == HIGH) {       // test for a low-to-high on channel A
    if ( digitalRead(rightEncoderPinB) == LOW ) {     // check channel B to see which way encoder is turning
      rightCurrPos++;
    }
    else {
      rightCurrPos--;
    }
  }
  else {                                        // it was a high-to-low on channel A
    if ( digitalRead(rightEncoderPinB) == HIGH ) {    // check channel B to see which way encoder is turning
      rightCurrPos++;
    }
    else {
      rightCurrPos--;
    }
  }
}

// Start of PID Controller function
// Modifies velocity and direct variables
// Returns output current
int PIDcontrol(int targetPos, int currentPos, int currentPosLast,
double &velocity, int minCurrent, int i1Pin, int i2Pin, int &direct)
{
  // determine time difference
  prevTime = currTime;
  currTime = millis();
  int timeDiff = currTime-prevTime;

  // PID calculations here
  int error = targetPos-currentPos;
  Iterm += Ki*error;
  if(Iterm > 255) Iterm = 255;
  else if(Iterm < 0) Iterm = 0;
  double dCurrentPos = currentPos-currentPosLast;
  double output = Kp*error+Iterm-Kd*dCurrentPos; // cmd signal ranging from 0-255
  
  // determine velocity
  velocity = dCurrentPos*1000/timeDiff;

  // if no PID set, this will calibrate the start-up current
  if(Kp==0 && Kd==0 && Ki==0)
  {
    output = int((targetPos-currentPos)/abs(targetPos-currentPos)*minCurrent);
  }
  // else if PID set, this will compute the PID output signal
  else
  {
    output = output*(1+minCurrent/abs(output));
  }
  // Determine rotation direction
  if (output < 0)
  {
    // rotate backward
    digitalWrite(i1Pin,HIGH);
    digitalWrite(i2Pin,LOW);
    output = -output;
    direct = -1;
  }
  else
  {
    //rotate forward
    digitalWrite(i1Pin,LOW);
    digitalWrite(i2Pin,HIGH);
    direct = 1;
  }
  // if current is greater than 255, set it to 255 (max)
  if(output > 255) output = 255;
  currentPosLast = currentPos;
  return output;
}
