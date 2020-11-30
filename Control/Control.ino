/*
* Group 14
* Avoiding Obstacle Robot
* Duy Hieu Vo
* Hong Phuc Mai
* Minh Gia Huy Cao
* Advanced Real Time Systems
* High Integrity Systems
* FH Frankfurt
*/

// defines pins numbers for ultrasonic sensor
const int trigPinMid = 12;
const int echoPinMid = 13;
const int trigPinRig = 3;
const int echoPinRig = 4;
const int trigPinLef = 11;
const int echoPinLef = 2;

////defines pins numbers for motors
//#define enA 6
//#define enB 5//10
//#define in1 9//6
//#define in2 8
//#define in3 10
//#define in4 7

//defines pins numbers for motors
#define enA 5
#define enB 6//10
#define in1 7//6
#define in2 10
#define in3 8
#define in4 9



// defines variables
int finaldistance[3];

long duration;

int distanceMid;
int prevmeasureMid;

int distanceRig;
int prevmeasureRig;

int distanceLef;
int prevmeasureLef;

float initcovarianceMid;
float initcovarianceRig;
float initcovarianceLef;

//threshold values
int  distMidMax;
int  distRigMax;
int  distLefMax;
int  distCritical;

void setup() {
  //Ultrasonic sensors
  pinMode(trigPinMid, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinMid, INPUT); // Sets the echoPin as an Input

  pinMode(trigPinRig, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinRig, INPUT); // Sets the echoPin as an Input
  
  pinMode(trigPinLef, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPinLef, INPUT); // Sets the echoPin as an Input
  Serial.begin(9600); // Starts the serial communication

  //Sensors
  pinMode(enA, OUTPUT); //Set the motor A as an Output
  pinMode(enB, OUTPUT); //Set the motor B as an Output
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  prevmeasureMid=measure(trigPinMid,echoPinMid);
  prevmeasureRig=measure(trigPinRig,echoPinRig);
  prevmeasureLef=measure(trigPinLef,echoPinLef);
  
  initcovarianceMid = 10e-3;
  initcovarianceRig = 10e-3;
  initcovarianceLef = 10e-3;

  //threshold ultrasonic sensor values (cm)
  distMidMax = 30; //threshold for mid sensor
  distRigMax = 20; //threshold for right sensor
  distLefMax = 20; //threshold for left sensor
  distCritical = 5; //threshold for obstacles are too near
} 
void loop() {
  distanceMid = measure(trigPinMid,echoPinMid);
  distanceMid = KF(distanceMid,prevmeasureMid,initcovarianceMid);

  distanceRig = measure(trigPinRig,echoPinRig);
  distanceRig = KF(distanceRig,prevmeasureRig,initcovarianceRig);
  
  distanceLef = measure(trigPinLef,echoPinLef);
  distanceLef = KF(distanceLef,prevmeasureLef,initcovarianceLef);

//  Serial.print("Mid: ");
//  Serial.print(distanceMid);
//  Serial.print(", Right: ");
//  Serial.print(distanceRig);
//  Serial.print(", Left: ");
//  Serial.println(distanceLef);
//  Serial.flush();
//  
  //control motor
  control(distanceMid, distanceRig, distanceLef);
//  avoidNear(600);
}

//control the car
void control(int distMid, int distRig, int distLef) {

  if ((distMid>distMidMax) and (distLef>distLefMax) and (distRig>distRigMax)) {
    runForward(30);
//    Serial.println("Run forward");
  }
  
  //obstacles on left side, turn right slightly
  else if ((distMid>distMidMax) and (distLef<=distLefMax) and (distRig>distRigMax)) {
    turnRight(25);
//    Serial.println("Turn right slightly");
  }
  
  //obstacles on right side, turn left slightly
  else if ((distMid>distMidMax) and (distLef>distLefMax) and (distRig<=distRigMax)) {
    turnLeft(25);
//    Serial.println("Turn left slightly");
  }
  
  //obstacles on both sides, run forward as it's going to a narrow way
  else if ((distMid>distMidMax) and (distLef<=distLefMax) and (distRig<=distRigMax)) {
    runForward(30);
//    Serial.println("Run forward");
  }
  
  //obstacles on left and mid side, turn right hardly
  else if ((distMid<=distMidMax) and (distLef<=distLefMax) and (distRig>distRigMax)) {
    turnRight(150); 
//    Serial.println("Turn right hardly");
  }
  
  //obstacles on right and mid side, turn left hardly
  else if ((distMid<=distMidMax) and (distLef>distLefMax) and (distRig<=distRigMax)) {
    turnLeft(150);
//    Serial.println("Turn left hardly");
  }
  
  //obstacles only on mid side, turn randomly
  else if ((distMid<=distMidMax) and (distLef>distLefMax) and (distRig>distRigMax)) {
      chooseDirection();
//    Serial.println("Turn randomly");
  }
  
  //obstacles on all three sides: DEAD END!!!
  else if ((distMid<=distMidMax) and (distLef<=distLefMax) and (distRig<=distRigMax)){
    while((distLef<=distLefMax) and (distRig<=distRigMax)){
      runBackward(800);
//      Serial.println("Run backward");

      //read values continously
      distRig = measure(trigPinRig,echoPinRig);
      distRig = KF(distRig,prevmeasureRig,initcovarianceRig);
  
      distLef = measure(trigPinLef,echoPinLef);
      distLef = KF(distLef,prevmeasureLef,initcovarianceLef);
    }
    backUp(800);
//    Serial.println("Run backup");
  }
}

//obstacle too near!
//void avoidNear(int delaytime){
//  //read new values after doing the action from method control 
//  //to detect whether after avoiding the first obstacle, another one appears
//  //which is too near the robot (could make the car cannot avoid it)
//  //distCritical is the threshold value of too near obstacles -> runBackward
//  
//  int distMid = measure(trigPinMid,echoPinMid);
//  distMid = KF(distanceMid,prevmeasureMid,initcovarianceMid);
//
//  int distRig = measure(trigPinRig,echoPinRig);
//  distRig = KF(distanceRig,prevmeasureRig,initcovarianceRig);
//  
//  int distLef = measure(trigPinLef,echoPinLef);
//  distLef = KF(distanceLef,prevmeasureLef,initcovarianceLef);
//  
//  if ((distMid <= distCritical) or (distLef <= distCritical) or (distRig <= distCritical)){
//      runBackward(delaytime);
//      backUp(delaytime);
////      Serial.println("Obstacles are too near");
//  }
//}

//---------------------------Motor------------------------------

//no obstacle -> run forward
void runForward(int delaytime) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  analogWrite(enA, 180); //127 
  analogWrite(enB, 200); //120 
  delay(delaytime);
}

//obstacle at all directions (deadend) -> run backward
void runBackward(int delaytime){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enA, 180); //127                     
  analogWrite(enB, 200); //120
  delay(delaytime);
}

//turn left or right randomly when mid sensor detects obstacles
void chooseDirection(){
  int choose = random(1,3);
  int delaytime = 800;
  
  if (choose == 1) {
    turnLeft(delaytime);
  }
  else if (choose == 2){
    turnRight(delaytime);
  }
}

void backUp(int delaytime){
  int choose = random(1,3);
  //turn left backup
   if (choose == 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, 180);                        
    analogWrite(enB, 20); 
    delay(delaytime);  
  }
  //turn right backup
  else if (choose == 2){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enA, 20);                        
    analogWrite(enB, 200);
    delay(delaytime);  
  }
}

//obstacles on the right side -> car turns left
void turnLeft(int delaytime){
  analogWrite(enA, 180);
  analogWrite(enB, 20);
  delay(delaytime); 
}

//obstacles on the left side -> car turns right
void turnRight(int delaytime){
  analogWrite(enA, 20);
  analogWrite(enB, 200);
  delay(delaytime);
}


//--------------------------Ultrasonic sensor--------------------
int measure(int trigPin, int echoPin){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  int distance= duration*0.034/2;
  
  return distance;
}

int KF(int currentmeasure, int prevmeasure, float initcovariance){
  float covariance = initcovariance + 1e-5;
  float k = covariance*1/((covariance + (2.92e-5)));
  int distance1 = prevmeasure + k*(currentmeasure - prevmeasure);
  covariance = (1-k)*covariance;
  prevmeasure = distance1;
  initcovariance = covariance;
  return distance1;
}
