#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Motor control pins
int motorEnA = 10;    
int motorIn1 = A0;    
int motorIn2 = A3;    
int motorEnB = 11;   
int motorIn3 = A1;   
int motorIn4 = A2;   
int irL = 12, irR = 13;

volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;

const int encoderL = 2;
const int encoderR = 3;

const float wheelCircumference = 21.70;
const int pulsesPerRevolution = 19;

Adafruit_MPU6050 mpu;
float roll = 0;

int i = 1;

float gyroZ, previousGyroZ;
unsigned long previousTime;
float angleZ = 0; // Total angle of rotation in degrees
const float dt = 0.01;

float distance = 0;

float time = 0;

void leftEncoderInterrupt() {
  leftEncoderCount++;
}

void rightEncoderInterrupt() {
  rightEncoderCount++;
}

void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);

  pinMode(motorIn1, OUTPUT);
  pinMode(motorIn2, OUTPUT);
  pinMode(motorIn3, OUTPUT);
  pinMode(motorIn4, OUTPUT);
  pinMode(motorEnA, OUTPUT);
  pinMode(motorEnB, OUTPUT);

  pinMode(irL, INPUT);
  pinMode(irR, INPUT);

  pinMode(encoderL, INPUT_PULLUP);
  pinMode(encoderR, INPUT_PULLUP);
  attachPCINT(digitalPinToPCINT(encoderL), leftEncoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderR), rightEncoderInterrupt, CHANGE);

  if (!mpu.begin()) {
    while (1); // Halt if MPU6050 initialization fails
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  previousTime = millis();
}



 
void loop() {
  int valL = digitalRead(irL);
  int valR = digitalRead(irR);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // Get sensor data once per loop

  float leftDistance = (leftEncoderCount / (float)pulsesPerRevolution) * wheelCircumference;
  float rightDistance = (rightEncoderCount / (float)pulsesPerRevolution) * wheelCircumference;
  float distance = (leftDistance + rightDistance) / 2.0;

  // Calculate pitch angle (the angle on the ramp)
  float angle = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  

  lcd.setCursor(0, 0);
  lcd.print("Angle: ");
  lcd.print(angle); 
  lcd.print(" deg ");

  lcd.setCursor(0, 1);
  lcd.print("D: ");
  lcd.print(distance);
  lcd.print(" cm");
  
if (distance < 573 || i == 8){
  if (valL == 1 && valR == 1) {
      analogWrite(motorEnA, 70);
      analogWrite(motorEnB, 70);
      digitalWrite(motorIn1, HIGH);
      digitalWrite(motorIn2, LOW);
      digitalWrite(motorIn3, HIGH);
      digitalWrite(motorIn4, LOW);

    
      if (i == 1 || i == 2) {
      analogWrite(motorEnA, 205);
      analogWrite(motorEnB, 205);
      digitalWrite(motorIn1, HIGH);
      digitalWrite(motorIn2, LOW);
      digitalWrite(motorIn3, HIGH);
      digitalWrite(motorIn4, LOW);
      delay(500);
        i = i + 1;
        Serial.print("1");
      }
      if ((angle > 10 ) && i == 3) {
      analogWrite(motorEnA, 255);
      analogWrite(motorEnB, 255);
      digitalWrite(motorIn1, HIGH);
      digitalWrite(motorIn2, LOW);
      digitalWrite(motorIn3, HIGH);
      digitalWrite(motorIn4, LOW);
        delay(1200); // Stop at ramp
        i = i + 1;
        Serial.print("2");
      }
      if (i == 4) {
      analogWrite(motorEnA, 0);
      analogWrite(motorEnB, 0);
      digitalWrite(motorIn1, LOW);
      digitalWrite(motorIn2, LOW);
      digitalWrite(motorIn3, LOW);
      digitalWrite(motorIn4, LOW);
        delay(4000); // Stop at ramp
        i = i + 1;
        Serial.print("3");
      }
      if (i == 5) {
      analogWrite(motorEnA, 255);
      analogWrite(motorEnB, 255);
      digitalWrite(motorIn1, HIGH);
      digitalWrite(motorIn2, LOW);
      digitalWrite(motorIn3, LOW);
      digitalWrite(motorIn4, HIGH);
      delay(2100);
      i = i + 1;
      Serial.print("4");
      }
    
    if (i == 6) {
      analogWrite(motorEnA, 80);
      analogWrite(motorEnB, 80);
      digitalWrite(motorIn1, HIGH);
      digitalWrite(motorIn2, LOW);
      digitalWrite(motorIn3, HIGH);
      digitalWrite(motorIn4, LOW);
      delay(1500) ; 
      i = i + 1 ;
    }
  }
  else if (valL == 1 && valR == 0) {
      analogWrite(motorEnA, 255);
      analogWrite(motorEnB, 255);
      digitalWrite(motorIn1, HIGH);
      digitalWrite(motorIn2, LOW);
      digitalWrite(motorIn3, LOW);
      digitalWrite(motorIn4, HIGH);
      Serial.print("left");
  } 
  else if (valL == 0 && valR == 1) {
      analogWrite(motorEnA, 255);
      analogWrite(motorEnB, 255);
      digitalWrite(motorIn1, LOW);
      digitalWrite(motorIn2, HIGH);
      digitalWrite(motorIn3, HIGH);
      digitalWrite(motorIn4, LOW);
      Serial.print("right");
  } 
  else if (valL == 0 && valR == 0) {
      analogWrite(motorEnA, 255);
      analogWrite(motorEnB, 255);
      digitalWrite(motorIn1, LOW);
      digitalWrite(motorIn2, LOW);
      digitalWrite(motorIn3, LOW);
      digitalWrite(motorIn4, LOW);
      
  } 
}
else if (distance > 573 && i == 7){
  analogWrite(motorEnA, 255);
      analogWrite(motorEnB, 255);
      digitalWrite(motorIn1, LOW);
      digitalWrite(motorIn2, LOW);
      digitalWrite(motorIn3, LOW);
      digitalWrite(motorIn4, LOW);
      delay(3000);
      i = i + 1;
      
      
}

}
