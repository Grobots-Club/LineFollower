#include <QTRSensors.h>


#define IN1 5
#define IN2 6
#define IN3 9
#define IN4 10

QTRSensors qtr;
const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

int velocity=80;
int baseSpeed=170;
int LS;
int RS;
const float kp=0.020;
const float kd=0.55;
uint16_t ideal=2500;
int lastError=0;
int boundary=200;
int maxSpeed=255;

void setup(){
    Serial.begin(9600);               
    pinMode(IN1,OUTPUT); 
    pinMode(IN2,OUTPUT); 
    pinMode(IN3,OUTPUT); 
    pinMode(IN4,OUTPUT); 
    for(int i=A0;i<=A5;i++){
      pinMode(i,INPUT);
    }
    delay(2000);
    manualCalibration();
    stopBot();
    delay(2000);
}
void loop(){
   specialCases();
   applyPID();
   moveForward();
}

void applyPID(){
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error=position-ideal;
   float PError=error*kp;
   float DError=(error-lastError)*kd;
   float adjust=PError+DError;
   LS=baseSpeed-adjust;
   RS=baseSpeed+adjust;
   LS=constrain(LS,0,maxSpeed);
   RS=constrain(RS,0,maxSpeed);
   lastError=error;
}


void manualCalibration(){
  Serial.println("QTR configuration started:");
    // configure the sensors
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
    moveAround();
    for (uint16_t i = 0; i < 200; i++){
      qtr.calibrate();
    }
    digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
}


void specialCases(){
  if(sensorValues[0]>boundary){
    rotateRight();
    //Serial.println("RIGHT");
  }
  else if(sensorValues[5]>boundary){
    rotateLeft();
    //Serial.println("LEFT");
  }
  else if(sensorValues[0]<boundary && sensorValues[1]<boundary && sensorValues[2]<boundary && sensorValues[3]<boundary &&sensorValues[4]<boundary && sensorValues[5]<boundary){
    rotateLeft();
    //Serial.println("TURN AROUND");
    
  }
 else if(sensorValues[0]>boundary && sensorValues[1]>boundary && sensorValues[2]>boundary && sensorValues[3]>boundary &&sensorValues[4]>boundary && sensorValues[5]>boundary){
    rotateRight();
    //Serial.println("ALL BLACK");
  }
}

void rotateLeft(){
  int error;
  uint16_t pos;
  do{
     pos = qtr.readLineBlack(sensorValues);
     error=pos-ideal;
    analogWrite(IN1,0);
    analogWrite(IN2,255);
    analogWrite(IN3,255);
    analogWrite(IN4,0);
  }while(error>=20&&error<=-20);
}

void rotateRight(){
  int error;
  uint16_t pos;
  do{
    pos = qtr.readLineBlack(sensorValues);
     error=pos-ideal;
    analogWrite(IN1,255);
    analogWrite(IN2,0);
    analogWrite(IN3,0);
    analogWrite(IN4,255);
  }while(error>=20&&error<=-20);
}


void rotate180(){
  analogWrite(IN1,255);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,255);
  delay(200);
}
void moveForward(){
  analogWrite(IN1,LS);
  analogWrite(IN2,0);
  analogWrite(IN3,RS);
  analogWrite(IN4,0);
}

void moveAround(){
  analogWrite(IN1,velocity);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,velocity);
}


void stopBot(){
  analogWrite(IN1,0);
  analogWrite(IN2,0);
  analogWrite(IN3,0);
  analogWrite(IN4,0);
}
