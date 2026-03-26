/*
 * FINAL ROBOT CODE: SMART EXPLORER + FINISH LINE + STUCK CHECK
 * * Behavior:
 * 1. Cruises Straight (PID).
 * 2. Obstacle? -> Stop -> BACK UP -> Scan Sectors -> Turn to BEST Angle.
 * 3. Black Line? -> STOP FOREVER.
 * 4. ABNORMALITY: Detects if stuck (motors running but not moving).
 */

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Arduino_LSM6DSOX.h> 
#include <math.h>

// --- SETTINGS ---
const float GYRO_SCALE = 1.8;   // Your calibrated scale
const int SAFE_DIST = 10;       
const int CRUISE_SPEED = 90;   
const int MIN_SPEED = 40;       // Anti-Stall Minimum

const int BACKUP_SPEED = 120;   
const int BACKUP_TIME = 400;    

const int TURN_PULSE_PWR = 140; 
const int TURN_PULSE_MS = 60;   
const int TURN_TOLERANCE = 3;   

// --- STUCK DETECTION SETTINGS (NEW) ---
const int STUCK_TIME_MS = 1500; // Check every 1.5 seconds
const int STUCK_TOLERANCE = 2;  // If dist changes < 2cm, we are stuck

// --- PINS ---
#define enA 2   
#define in1 3   
#define in2 4   
#define in3 5   
#define in4 6   
#define enB 7   
#define echoPin 8  
#define trigPin 9  
#define servoPin 10 
#define irPin 12    // IR Sensor for Finish Line

Servo myServo;
float yawDeg = 0.0f;
float headingTarget = 0.0f;
float gyroBiasZ = 0.0f;
unsigned long lastIMUms = 0;
bool isMoving = false; 

// Stuck Detection Variables (NEW)
unsigned long lastStuckCheck = 0;
int lastDistReading = 0;

// PID Tuning
const float Kp_h = 3.0f; 

void setup() {
  Serial.begin(115200);

  pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT); pinMode(enB, OUTPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(irPin, INPUT); 

  myServo.attach(servoPin);
  myServo.write(90); 

  if (!IMU.begin()) { while (1); }

  Serial.println("Calibrating... DO NOT MOVE");
  float sum = 0;
  for(int i=0; i<1000; i++) { 
    float x, y, z;
    if (IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(x, y, z);
      sum += z;
    }
    delay(1); 
  }
  gyroBiasZ = sum / 1000.0;
  
  lastIMUms = millis();
  headingTarget = 0.0; 
  yawDeg = 0.0;
  
  Stop();
  delay(1000);
}

void loop() {
  updateIMU(); 

  // 1. CHECK FINISH LINE
  if (digitalRead(irPin) == HIGH) {
    Stop();
    Serial.println("FINISH LINE DETECTED! STOPPING.");
    while(1); // Stop forever
  }

  int frontDist = getDistance();

  // 2. CHECK ABNORMALITY (Are we stuck?)
  bool isStuck = checkStuckCondition(frontDist);

  // 3. DECISION LOGIC: Obstacle OR Stuck
  if ((frontDist > 0 && frontDist < SAFE_DIST) || isStuck) {
    
    Serial.println(isStuck ? "ABNORMALITY: STUCK!" : "OBSTACLE DETECTED!");
    
    Stop();
    isMoving = false;
    delay(200);

    // --- STEP A: BACK UP ---
    Serial.println("Backing Up...");
    moveBackward();
    delay(BACKUP_TIME);
    Stop();
    delay(300);

    // --- STEP B: SECTOR SCAN ---
    Serial.println("Scanning Sectors...");
    float turnAngle = scanSectorsAndFindBest();
    Serial.print("Best Angle Found: "); Serial.println(turnAngle);

    // --- STEP C: EXECUTE TURN ---
    if (abs(turnAngle) > 5) {
       rotateStepByStep(turnAngle);
    }

    // Reset PID Target to the new heading
    headingTarget = yawDeg; 
    
    // Reset Stuck Detection so we don't trigger immediately
    lastStuckCheck = millis();
    lastDistReading = getDistance();
    
    Stop();
    delay(300);
  } 
  // 4. CLEAR PATH -> Drive Straight
  else {
    forwardWithPID(CRUISE_SPEED);
  }
}

// ===============================================
// ABNORMALITY CHECK (NEW FUNCTION)
// ===============================================
bool checkStuckCondition(int currentDist) {
  // Only check if we are currently trying to move
  if (isMoving && (millis() - lastStuckCheck > STUCK_TIME_MS)) {
    
    int change = abs(currentDist - lastDistReading);
    
    // Update history
    lastStuckCheck = millis();
    lastDistReading = currentDist;
    
    // 
    // If distance changed less than 2cm in 1.5 seconds, we are blocked.
    if (change < STUCK_TOLERANCE) {
      return true;
    }
  }
  return false;
}

// ===============================================
// SECTOR SCANNING
// ===============================================
float scanSectorsAndFindBest() {
  int maxDist = -1;
  int bestServoAngle = 90;

  for (int angle = 0; angle <= 180; angle += 30) { 
    myServo.write(angle);
    delay(250); 
    int d = getDistance();
    if (d == -1) d = 300; 
    
    Serial.print("Sec:"); Serial.print(angle); 
    Serial.print(" D:"); Serial.println(d);

    if (d > maxDist) {
      maxDist = d;
      bestServoAngle = angle;
    }
  }
  myServo.write(90); 
  delay(300);
  return (float)(bestServoAngle - 90);
}

// ===============================================
// MOTION CONTROL
// ===============================================
void forwardWithPID(int targetSpeed) {
  setDirForward();
  
  // Kickstart if starting from stop
  if (!isMoving) {
    analogWrite(enA, 255); analogWrite(enB, 255);
    delay(50);
    isMoving = true;
    
    // Reset Stuck Timer on fresh start
    lastStuckCheck = millis();
    lastDistReading = getDistance();
  }
  
  float error = headingTarget - yawDeg;
  float correction = Kp_h * error; 

  int speedLeft = targetSpeed - correction;
  int speedRight = targetSpeed + correction;

  speedLeft = constrain(speedLeft, MIN_SPEED, 255);
  speedRight = constrain(speedRight, MIN_SPEED, 255);

  setPWM(speedLeft, speedRight);
}

void moveBackward() {
  setDirBackward(); 
  analogWrite(enA, 255); analogWrite(enB, 255); // Kickstart
  delay(50);
  setPWM(BACKUP_SPEED, BACKUP_SPEED);
}

// ===============================================
// PRECISION TURN
// ===============================================
void rotateStepByStep(float targetDelta) {
  float initialYaw = yawDeg;
  float targetYaw = initialYaw + targetDelta;
  unsigned long startTurnTime = millis();

  while (true) {
    updateIMU();
    float error = targetYaw - yawDeg;
    
    if (abs(error) <= TURN_TOLERANCE) break;
    if (millis() - startTurnTime > 5000) break; 

    if (error > 0) setDirTurnLeftInPlace(); 
    else setDirTurnRightInPlace();

    analogWrite(enA, TURN_PULSE_PWR);
    analogWrite(enB, TURN_PULSE_PWR);
    
    unsigned long startPulse = millis();
    while (millis() - startPulse < TURN_PULSE_MS) updateIMU();

    Stop();
    unsigned long startSettle = millis();
    while (millis() - startSettle < 100) updateIMU();
  }
  Stop();
  isMoving = false; 
}

// ===============================================
// SENSORS & HELPERS
// ===============================================
void updateIMU() {
  unsigned long now = millis();
  float dt = (now - lastIMUms) / 1000.0f;
  if(dt > 0.1 || dt <= 0) dt = 0.01;
  lastIMUms = now;

  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);
    float rate = z - gyroBiasZ;
    if (abs(rate) < 2.0) rate = 0; 
    yawDeg += (rate * dt * GYRO_SCALE);
  }
}

int getDistance() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 10000UL); 
  if (duration == 0) return -1;
  return (int)(duration * 0.0343 / 2);
}

void setPWM(int L, int R) { 
  analogWrite(enA, L); 
  analogWrite(enB, R); 
}
void setDirForward() { 
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); 
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW); 
}
void setDirBackward() { 
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH); 
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH); 
}
void setDirTurnLeftInPlace() { 
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH); 
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW); 
}
void setDirTurnRightInPlace() { 
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW); 
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH); 
}
void Stop() { 
  digitalWrite(in1, LOW); digitalWrite(in2, LOW); 
  digitalWrite(in3, LOW); digitalWrite(in4, LOW); 
  analogWrite(enA, 0); analogWrite(enB, 0); 
}