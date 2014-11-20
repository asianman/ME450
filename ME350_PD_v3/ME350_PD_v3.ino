// ME350 PD control Sketch
// for tuning the motor ONLY

// set the desired position (encoder count)
int desPosition1=150; 

int targetPosition=0;
int targetPositionLst=0;

// initialize the variables
int onOffSwitch=0;
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
const int onOffSwitchPin = 5;
const int pwmPin = 9;
const int i1Pin = 10;
const int i2Pin = 11;
const int encoderPinA = 2;
const int encoderPinB = 3;


void setup(){
  // declare which digital pins are inputs and which are outputs
  pinMode(encoderPinA,INPUT);
  pinMode(encoderPinB,INPUT);
  pinMode(pwmPin,OUTPUT);
  pinMode(i1Pin,OUTPUT);
  pinMode(i2Pin,OUTPUT);

  //turn on pullup resistors
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
  onOffSwitch=digitalRead(onOffSwitchPin);

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
    
  if (onOffSwitch==HIGH)
  {
  
//determine integraal
    integral=integral+((double)targetPosition-(double)encoderCountNew)*(double)timeDelay/1000;

    targetPosition=desPosition1;

// read the current position of the encoder count, then 
// calculate the error by comparing it to the desired position

// use a PID controller to send the appropriate signal to the motor
// This uses the function "pdcontrol" at the end of the sketch

    spdCmd=pdControl(targetPosition, encoderCountNew, integral, spd);

    analogWrite(pwmPin,spdCmd);
  }
  else if(onOffSwitch==LOW)
  {
    integral=0;
    spdCmd=0;
    analogWrite(pwmPin,spdCmd);
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
  float  Kp=0.39;
  float  Kd=0;
  float  baseCmd=78; // The value of cmd that makes the motor just start to rotate
  int    Cmd;   //closed loop motor PWM control signal
  int    error;
  error=targetPos-currentPos;
  if((targetPos-currentPos)>120)
  {
     error=120;
  }
  if((targetPos-currentPos)<-120)
  {
     error=-120;
  }
  Cmd=(Kp*error-Kd*mspd);
  
  if(Kp==0 && Kd==0)
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
  
  if (abs(targetPos-currentPos)<1.2)
  {
     Cmd=0;
  }
  
  return Cmd;
}
