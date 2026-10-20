/*
  ==============================================================
  ROBOT OTONOM 4 RODA (2 DRIVER BTS7960) + GRIPPER (2 SERVO)
  KOMUNIKASI SERIAL USB DENGAN RASPBERRY PI 5
  
  Mode: Autonomous Navigation (Master-Slave Architecture)
  Master: Raspberry Pi 5 (Vision + LiDAR + State Machine)
  Slave: ESP32 (Motor Control + Servo Control)
  
  Serial Protocol:
    Format: <speedLeft>,<speedRight>,<liftAngle>,<gripAngle>\n
    Example: 150,-100,90,30\n
    
    speedLeft, speedRight: -255 to 255 (negative = backward)
    liftAngle: 45-120 (45=down, 120=up)
    gripAngle: 30-90 (30=open, 90=close)
  
  Hardware:
    - ESP32-S3
    - 2x BTS7960 Motor Driver (4 wheels, differential drive)
    - 2x Servo MG90 (lift + gripper)
  
  Library yang Dibutuhkan:
    - ESP32Servo (install via Library Manager)
  ==============================================================
*/

#include <ESP32Servo.h>

// ================= PIN MOTOR BTS7960 =================
// Index 0: Sisi Kiri (2 roda paralel)
// Index 1: Sisi Kanan (2 roda paralel)
const int rpwmPins[] = {17, 6};  // {RPWM Kiri, RPWM Kanan}
const int lpwmPins[] = {18, 7};  // {LPWM Kiri, LPWM Kanan}
const int numDrivers = 2;

const int pwmFreq = 1000;
const int pwmResolution = 8;

// ================= GRIPPER (SERVO) =================
Servo servoLift;
Servo servoGripper;

const int pinLift = 4;
const int pinGripper = 5;

// ================= ULTRASONIC =================
const int trigPin = 15;
const int echoPin = 16;
const int ledPin = 10;
unsigned long lastUltraTime = 0;

int currentLift = 45;
int currentGrip = 30;

const int LIFT_MIN = 45;
const int LIFT_MAX = 120;
const int GRIP_MIN = 30;
const int GRIP_MAX = 90;

// ================= SERIAL COMMUNICATION =================
const unsigned long SERIAL_BAUD = 115200;
const unsigned long WATCHDOG_TIMEOUT = 1000;
unsigned long lastCommandTime = 0;
bool isMoving = false;

String serialBuffer = "";

// ================= ULTRASONIC SENSOR =================
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 999.0;
  return duration * 0.034 / 2.0;
}

// ================= FUNGSI MOTOR =================
void setMotor(int index, int speed) {
  if (index < 0 || index >= numDrivers) return;

  speed = constrain(speed, -255, 255);

  if (speed == 0) {
    ledcWrite(rpwmPins[index], 0);
    ledcWrite(lpwmPins[index], 0);
  } else if (speed > 0) {
    ledcWrite(rpwmPins[index], speed);
    ledcWrite(lpwmPins[index], 0);
  } else {
    ledcWrite(rpwmPins[index], 0);
    ledcWrite(lpwmPins[index], -speed);
  }
}

void setMotors(int speedLeft, int speedRight) {
  setMotor(0, speedLeft);
  setMotor(1, speedRight);
  
  if (speedLeft != 0 || speedRight != 0) {
    isMoving = true;
  } else {
    isMoving = false;
  }
}

void stopMotors() {
  setMotors(0, 0);
}

// ================= FUNGSI SERVO =================
void setServoLift(int angle) {
  angle = constrain(angle, LIFT_MIN, LIFT_MAX);
  currentLift = angle;
  servoLift.write(angle);
}

void setServoGrip(int angle) {
  angle = constrain(angle, GRIP_MIN, GRIP_MAX);
  currentGrip = angle;
  servoGripper.write(angle);
}

// ================= PARSING SERIAL COMMAND =================
void parseCommand(String cmd) {
  cmd.trim();
  
  if (cmd.length() == 0) return;
  
  int commaIndex1 = cmd.indexOf(',');
  int commaIndex2 = cmd.indexOf(',', commaIndex1 + 1);
  int commaIndex3 = cmd.indexOf(',', commaIndex2 + 1);
  
  if (commaIndex1 == -1 || commaIndex2 == -1 || commaIndex3 == -1) {
    Serial.println("ERR:INVALID_FORMAT");
    return;
  }
  
  String strSpeedLeft = cmd.substring(0, commaIndex1);
  String strSpeedRight = cmd.substring(commaIndex1 + 1, commaIndex2);
  String strLift = cmd.substring(commaIndex2 + 1, commaIndex3);
  String strGrip = cmd.substring(commaIndex3 + 1);
  
  int speedLeft = strSpeedLeft.toInt();
  int speedRight = strSpeedRight.toInt();
  int liftAngle = strLift.toInt();
  int gripAngle = strGrip.toInt();
  
  setMotors(speedLeft, speedRight);
  setServoLift(liftAngle);
  setServoGrip(gripAngle);
  
  lastCommandTime = millis();
  
  Serial.println("OK");
}

// ================= SETUP =================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  Serial.println("\n=== ESP32 Autonomous Robot Controller ===");
  Serial.println("Serial communication ready");
  Serial.println("Waiting for commands from Raspberry Pi...\n");
  
  for (int i = 0; i < numDrivers; i++) {
    ledcAttach(rpwmPins[i], pwmFreq, pwmResolution);
    ledcAttach(lpwmPins[i], pwmFreq, pwmResolution);
    ledcWrite(rpwmPins[i], 0);
    ledcWrite(lpwmPins[i], 0);
  }
  
  servoLift.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);
  servoLift.attach(pinLift, 500, 2400);
  servoGripper.attach(pinGripper, 500, 2400);
  
  servoLift.write(currentLift);
  servoGripper.write(currentGrip);
  
  lastCommandTime = millis();
  
  Serial.println("READY");
}

// ================= LOOP =================
void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    if (c == '\n') {
      parseCommand(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += c;
      
      if (serialBuffer.length() > 64) {
        serialBuffer = "";
        Serial.println("ERR:BUFFER_OVERFLOW");
      }
    }
  }
  
  // Stream ultrasonic data every 100ms
  if (millis() - lastUltraTime > 100) {
    float dist = getDistance();
    Serial.print("U:");
    Serial.println(dist);
    
    if (dist < 5.0) digitalWrite(ledPin, HIGH);
    else digitalWrite(ledPin, LOW);
    
    lastUltraTime = millis();
  }
  
  if (isMoving && (millis() - lastCommandTime > WATCHDOG_TIMEOUT)) {
    stopMotors();
    Serial.println("WARN:WATCHDOG_STOP");
  }
  
  delay(1);
}