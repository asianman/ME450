// ME450 Steering Rig Motor Controller, Team 16
// Switch
const int rightSwitch = A0;
const int leftSwitch = A1;
int rightNew;
int leftNew;
int rightOld;
int leftOld;
int n=0;

void setup(){
  // declare which digital pins are inputs and which are outputs
  pinMode(rightSwitch,OUTPUT);
  pinMode(leftSwitch,OUTPUT);
  // begin serial communication for display of variable states
  Serial.begin(115200);
  Serial.println("start");
}

void loop(){
  // determine time interval
  rightOld = rightNew;
  leftOld = leftNew;
  rightNew = digitalRead(rightSwitch);
  leftNew = digitalRead(leftSwitch);
  if(rightOld != rightNew) {
    n=0;
  }
  else if(rightNew)
  {
    Serial.println(n++);
  }
  if(leftOld != leftNew) {
    n=0;
  }
  else if(leftNew)
  {
    Serial.println(n++);
  }
}
