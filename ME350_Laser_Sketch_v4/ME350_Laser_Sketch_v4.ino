// ME350 Laser Reflection Sketch
// with diagnostics for "toggle switch off" and "object in the way"

// set the values of the variables that affect performance
int proxSwitchThreshold = 150;
// set the desired positions (encoder counts) for 4 lanes
int desPosition1=0; //suggest to put it as 0
int desPosition2=100;
int desPosition3=200;
int desPosition4=300;
int LowerBound=0;  // same as desPosition1, can be reset every time hit position 1
int UpperBound=300;  // same as desPosition4, can be reset every time hit position 4
int targetPosition=0;
int targetPositionLst=0;

// initialize the variables
int proxSwitch1=0;
int limitSwitchForward=0;
int limitSwitchReverse=0;
int onOffSwitch=0;
int lane1=0;
int lane2=0;
int lane3=0;
int lane4=0;
volatile long encoderCount = 0;  

// Ram for PD control
long encoderCountNew=0;
long encoderCountLst=0;
long encoderStep=0;
int direct=0;

unsigned long tic;
unsigned long toc;
unsigned long timeDelay;
float spd=0;
float spdlst=0;
int spdCmd=0;
double integral=0;

// set the values of the variables that represent pin numbers
const int proxSwitch1Pin = 1;
const int lane1Pin=4;
const int lane2Pin=6;
const int lane3Pin=12;
const int lane4Pin=13;
const int onOffSwitchPin = 5;
const int limitSwitchForwardPin=7;
const int limitSwitchReversePin=8;
const int pwmPin = 9;
const int i1Pin = 10;
const int i2Pin = 11;
const int encoderPinA = 2;
const int encoderPinB = 3;


void setup(){
  // declare which digital pins are inputs and which are outputs
  pinMode(encoderPinA,INPUT);
  pinMode(encoderPinB,INPUT);
  pinMode(lane1Pin,INPUT);
  pinMode(lane2Pin,INPUT);
  pinMode(lane3Pin,INPUT);
  pinMode(lane4Pin,INPUT);
  pinMode(pwmPin,OUTPUT);
  pinMode(i1Pin,OUTPUT);
  pinMode(i2Pin,OUTPUT);

  //turn on the pullup resistors
  digitalWrite(encoderPinA,HIGH);  
  digitalWrite(encoderPinB,HIGH);
  // (the other sensors already have physical resistors on the breadboard)

  //set interrupt for encoder pins
  attachInterrupt(0,doEncoderA,CHANGE);
  attachInterrupt(1,doEncoderB,CHANGE);

  //set initial speed of motor to 0
  analogWrite(pwmPin,0);

  //begin serial communication for display of variable states
  Serial.begin(9600);
  Serial.println("start");
}

void loop(){
//acquire signals from pins
  proxSwitch1=analogRead(proxSwitch1Pin);
  limitSwitchForward=digitalRead(limitSwitchForwardPin);
  limitSwitchReverse=digitalRead(limitSwitchReversePin);
  onOffSwitch=digitalRead(onOffSwitchPin);
  lane1=digitalRead(lane1Pin);
  lane2=digitalRead(lane2Pin);
  lane3=digitalRead(lane3Pin);
  lane4=digitalRead(lane4Pin);
  
  
// if the limit switch is made,
// then set the encoder count to zero
  if (limitSwitchReverse==HIGH)
  {
    encoderCount=LowerBound;
  }
  else if (limitSwitchForward==HIGH)
  {
    encoderCount=UpperBound;
  }
  
// if the limit switch is not made,
// then do the actions listed below
// determine time interval
   toc=tic; 
   tic=millis();  //elapsed time since Arduino began running the program 
   timeDelay=tic-toc;
   if(timeDelay==0)
   {
     timeDelay=140;
   }
  
//determing encoder count change
   encoderCountLst=encoderCountNew;
   encoderCountNew=encoderCount;
   encoderStep=encoderCountNew-encoderCountLst;
  
//determine speed
 
  spd=-spdlst*0.2+float(encoderStep)/float(timeDelay)*1000*1.2; //unit count/sec
  spdlst=spd;

  if (onOffSwitch==HIGH && proxSwitch1<proxSwitchThreshold)
  { 
//determine integral
    integral=integral+((double)targetPosition-(double)encoderCountNew)*(double)timeDelay/1000;

  

// see which lane has a laser on; set that lane's encoder count as the desired position
    if (lane1==HIGH)
    {
      targetPosition=desPosition1;
    }
    else if (lane2==HIGH)
    {
      targetPosition=desPosition2;
    }
    else if (lane3==HIGH)
    {
      targetPosition=desPosition3;
    }
    else if (lane4==HIGH)
    {
      targetPosition=desPosition4;
    }
    else 
    {
      targetPosition=targetPositionLst;
    }
    targetPositionLst=targetPosition;


// read the current position of the encoder count, then 
// calculate the error by comparing it to the desired position

// Use a PID controller to send the appropriate signal to the motor
// This uses the function "pdcontrol" at the end of the sketch

    spdCmd=pdControl(targetPosition, encoderCountNew, integral, spd);

    analogWrite(pwmPin,spdCmd);
  }
  else if(onOffSwitch==LOW)
  {
    integral=0;
    spd=0;
    spdCmd=0;
    analogWrite(pwmPin,spdCmd);
    Serial.println("The toggle switch is off.");
  }
 else if(proxSwitch1>=proxSwitchThreshold)
  {
    integral=0;
    spd=0;
    spdCmd=0;
    analogWrite(pwmPin,spdCmd);
    Serial.println("There is an object in the way.");
  }
  
// display the current position, desired position, and signal
// on the serial monitor
   Serial.print("Power: ");
  Serial.print(onOffSwitch);
  Serial.print("      Spd: ");
  Serial.print(spd);
  Serial.print("      Current Position: ");
  Serial.print(encoderCount);
  Serial.print("      Desired Position: ");
  Serial.print(targetPosition);
  Serial.print("      Command Signal: ");
  Serial.print(spdCmd);
  Serial.print("      Time Delay: ");
  Serial.print(timeDelay);
  if(direct>=0)
  {
    Serial.println("    Direction: Forward");
  }
  else if(direct<0)
  {
    Serial.println("    Direction: Reverse");
  }
/*  Serial.print("Command Signal: ");
  Serial.println(spdCmd);
  if(direct>0)
  {
    Serial.println("Direction: Forward");
  }
  else if(direct<0)
  {
    Serial.println("Direction: Reverse");
  }
  Serial.print("Spd: ");
  Serial.println(spd);
  Serial.print("Integral: ");
  Serial.println(integral);*/
}

// Below is the function that will increment/decrement the count
// every time there is a change on the A channel of the encoder.
void doEncoderA(){
  // look for a low-to-high on channel A
  if (digitalRead(encoderPinA) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderPinB) == LOW) {  
      encoderCount++;         // CW
    } 
    else {
      encoderCount--;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoderPinB) == HIGH) {   
      encoderCount++;          // CW
    } 
    else {
      encoderCount--;          // CCW
    }
  }
}

void doEncoderB(){
  // look for a low-to-high on channel B
  if (digitalRead(encoderPinB) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(encoderPinA) == HIGH) {  
      encoderCount++;         // CW
    } 
    else {
      encoderCount--;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoderPinA) == LOW) {   
      encoderCount++;          // CW
    } 
    else {
      encoderCount--;          // CCW
    }
  }
}

int pdControl(int targetPos, float currentPos, float integral, float mspd )
{
  float  Kp=0.385;
  float  Kd=0;
  float  Ki=0.01;
  float  baseCmd=78; // The value of cmd that makes the motor just start to rotate
  int    error;
  int    Cmd;   //closed loop motor PWM control signal
  
  //limit the error, to reduce overshoot if travelling through a large angle
  error=targetPos-currentPos;
  if((targetPos-currentPos)>120)
  {
     error=120;
  }
  if((targetPos-currentPos)<-120)
  {
     error=-120;
  }
    
  Cmd=(Kp*error+Ki*integral-Kd*mspd);
  
  if(Kp==0 && Kd==0 && Ki==0)
  {
    Cmd=int((targetPos-currentPos)/abs(targetPos-currentPos)*baseCmd);
  }
  
  else
  {
    Cmd=int(Cmd/abs(Cmd)*baseCmd+Cmd);
  }
  
  // Determine rotation direction
  if (Cmd>=0)
  {
    digitalWrite(i1Pin,LOW);
    digitalWrite(i2Pin,HIGH);  //rotate forward
    direct=1;
    if (Cmd>255)
    {
      Cmd=255;
    }
  }
  
  else
  {
    digitalWrite(i1Pin,HIGH);
    digitalWrite(i2Pin,LOW); // rotate backward
    Cmd=-Cmd;
    direct=-1;
    if (Cmd>255)
    {
       Cmd=255;
    }
  }
  
  if (abs(targetPos-currentPos)<3.2)
  {
     Cmd=0;
  }
  
  return Cmd;
}
