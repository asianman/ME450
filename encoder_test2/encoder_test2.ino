// ME450 Steering Rig Motor Controller, Team 16
// set two desired positions
int leftTargPos, rightTargPos;

// initialize the variables
volatile long leftCurrPos, rightCurrPos;
double n1 = 0;
double n2 = 0;

// Ram for PD control
int leftPrevPos, rightPrevPos;

// to measure time
unsigned long prevTime;
unsigned long currTime;

// Encoder
const int leftEncoderPinA = 4;
const int leftEncoderPinB = 2;
const int rightEncoderPinA = 3;
const int rightEncoderPinB = 5;
double leftVel=0;
double rightVel=0;
// set all to 0 to tune
double Kp=0.1; // increase until response is steady oscillation
double Kd=0.1; // increase until oscillations go away
// keep changing Kp and Kd until increasing Kd doesn't stop oscillations
// set Kp and Kd to last stable values
double Ki=0; // increase until it gets you to target
double leftMinCurrent=30; // value that makes motor just start to rotate
double rightMinCurrent=30; // value that makes motor just start to rotate
double Iterm;
int leftPower;
int rightPower;

// PID Motor Controller
const int leftpwmPin = 10;
const int lefti1Pin = 8;
const int lefti2Pin = 9;
const int rightpwmPin = 11;
const int righti1Pin = 12;
const int righti2Pin = 13;
int direct1=1;
int direct2=1;

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
  pinMode(leftEncoderPinA,INPUT);
  pinMode(leftEncoderPinB,INPUT);
  pinMode(rightEncoderPinA,INPUT);
  pinMode(rightEncoderPinB,INPUT);
  
  //turn on the pullup resistors
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

void loop(){
  // increment target position as switch is held
  rightOld = rightNew;
  leftOld = leftNew;
  rightNew = digitalRead(rightSwitch);
  leftNew = digitalRead(leftSwitch);
  if(rightNew && rightOld == rightNew)
  {
    n1-=.01;
    n2-=.01;
    Serial.println(round(n1));
  }
  else if(leftNew && leftOld == leftNew)
  {
    n1+=.01;
    n2+=.01;
    Serial.println(round(n1));
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
  leftTargPos = round(n1);
  rightTargPos = round(n2);
  // determing encoder count change
  leftPrevPos=leftCurrPos;
  rightPrevPos=rightCurrPos;

  // use PID to control output signals
  leftPower = PIDcontrol(leftTargPos, leftCurrPos, leftPrevPos, leftVel,
    leftMinCurrent, lefti1Pin, lefti2Pin, direct1);
  rightPower = PIDcontrol(rightTargPos, rightCurrPos, rightPrevPos, rightVel,
    rightMinCurrent, righti1Pin, righti2Pin, direct2);
  
  // send output signal to motor
  analogWrite(leftpwmPin, leftPower);
  analogWrite(rightpwmPin, rightPower);
}

// Start of Interrupt routine for encoder #1
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
  if(output > 255) output = 255;
  
  // determine velocity
  velocity = dCurrentPos*1000/timeDiff;
  
  // if no PID set, this will calibrate the start-up current
  if(Kp==0 && Kd==0 && Ki==0)
  {
    output = int((targetPos-currentPos)/abs(targetPos-currentPos)*minCurrent);
  }
  // if PID set, this will compute the PID output signal
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
  
  currentPosLast = currentPos;
  return output;
}
