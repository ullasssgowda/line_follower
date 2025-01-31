



#include <Arduino.h>

#include <QTRSensors.h>
#include "Grove_Motor_Driver_TB6612FNG.h"

// Define pins for the IR sensors
const int sensorPins[8] = { 1, 2, 3, 4, 5, 6, 7, 8};  // Adjust these pins based on your connections

// Define pins for the motor driver
const int AIN1 = 18;
const int AIN2 = 5;
const int PWMA = 4;
const int BIN1 = 21;
const int BIN2 = 22;
const int PWMB = 23;
const int STBY = 19;

// PID control variables
float Kp = 0.6;  // Proportional gain (adjust for competition speed)
float Ki = 0.0;  // Integral gain (typically not used for line following)
float Kd = 0.3;  // Derivative gain (adjust for competition speed)

float lastError = 0;
float integral = 0;

// Motor speed
int baseSpeed = 200;  // Base speed of the motors (adjust for competition)
int maxSpeed = 255;   // Maximum speed of the motors

void setup() {
  // Initialize motor control pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Initialize IR sensor pins
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Start the motor driver
  digitalWrite(STBY, HIGH);

  Serial.begin(115200);
}

void loop() {
  int position = readSensors();
  float error = position - 3500;  // Centered on 3500 for an 8-sensor array

  integral = constrain(integral + error, -1000, 1000);  // Limit the integral term
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  driveMotors(leftSpeed, rightSpeed);

  delay(5);  // Small delay for stability, adjust as needed for speed
}

int readSensors() {
  int sensorValues[8];
  int weightedSum = 0;
  int sum = 0;

  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    weightedSum += sensorValues[i] * (i * 1000);  // Weight each sensor
    sum += sensorValues[i];
  }

  if (sum == 0) return 3500;  // Default to center if no line detected

  return weightedSum / sum;
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  analogWrite(PWMA, leftSpeed);

  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    rightSpeed = -rightSpeed;
  }
  analogWrite(PWMB, rightSpeed);
}
