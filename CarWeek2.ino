#include <LiquidCrystal.h>
#include <PinChangeInterrupt.h>  

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

int motor1pin1 = A0, motor1pin2 = A3;
int motor2pin1 = A2, motor2pin2 = A1;
int enA = 10, enB = 11;

int irL = 12, irR = 13;

volatile int leftEncoderCount = 0;   // Count pulses for left encoder
volatile int rightEncoderCount = 0; // Count pulses for right encoder

const int encoderL = 2;  // Left encoder connected to pin 2
const int encoderR = 3;  // Right encoder connected to pin 3
//please send help 
const float wheelCircumference = 20.106; // Circumference in cm (radius = 3.2 cm)
const int pulsesPerRevolution = 20;      // Number of pulses per wheel rotation



void leftEncoderInterrupt() {
  leftEncoderCount++;
}

void rightEncoderInterrupt() {
  rightEncoderCount++;
}

void setup() {
  lcd.begin(16, 2);

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(irL, INPUT);
  pinMode(irR, INPUT);

  pinMode(encoderL, INPUT_PULLUP); // Enable pull-up resistor for A5
  pinMode(encoderR, INPUT_PULLUP); // Enable pull-up resistor for 13

  // Attach interrupts
  attachPCINT(digitalPinToPCINT(encoderL), leftEncoderInterrupt, CHANGE); // Pin change interrupt for A5
  attachPCINT(digitalPinToPCINT(encoderR), rightEncoderInterrupt, CHANGE); // Hardware interrupt for pin 13

  Serial.begin(9600);
}

void loop() {
  int valL = digitalRead(irL);
  int valR = digitalRead(irR);

  // Motor control logic
  if (valL == 1 && valR == 1) { //move forward
    analogWrite(enA, 65);
    analogWrite(enB, 65);
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
  } else if (valL == 1 && valR == 0) { //turn left
    analogWrite(enA, 255);
    analogWrite(enB, 255);
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, HIGH);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
  } else if (valL == 0 && valR == 1) { //turn right
    analogWrite(enA, 255);
    analogWrite(enB, 255);
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, HIGH);
    digitalWrite(motor2pin2, LOW);
  } else { //stop
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
  }

  // Calculate distances
  float leftDistance = (leftEncoderCount / (float)pulsesPerRevolution) * wheelCircumference;
  float rightDistance = (rightEncoderCount / (float)pulsesPerRevolution) * wheelCircumference;

  // Calculate average distance
  float distance = (leftDistance + rightDistance) / 2.0;

  // Display on LCD
  lcd.setCursor(0, 0);
  lcd.print("Distance: ");
  lcd.print(distance);
  lcd.print(" cm");

}
