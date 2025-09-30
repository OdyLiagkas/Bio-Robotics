#include <Wire.h>
#include <MPU6050.h>
#include <math.h>
#include <VL53L1X.h>
#include <I2Cdev.h>
#include <Servo.h>

// ==== lEDS
#include <FastLED.h>
#define NUM_LEDS 8
#define DATA_PIN 38


// ===== Distance sensor =====
VL53L1X sensorDist;
#define XSHUT_CENTER 39

// ====== Servo ========
Servo servo;
int pos =0;

// ===== Button pin =====
#define BTN_PIN 11

// ===== BUZZER pin =====
#define BUZZER_PIN  A7

// ===== MPU6050 gyroscope =====
MPU6050 mpu; // (0x69);
float gyroZ_offset = 0;
float totalGyroZ = 0;
bool trackingRotation = false;
unsigned long lastTime = 0;

// ===== CNC Shield motor control pins =====
const byte enablePin = 8;
  // left-front Z
  // left-rear X
  // right-front Y
  // right-rear A
const int StepX = 2, DirX = 5;
const int StepY = 3, DirY = 6;
const int StepZ = 4, DirZ = 7;
const int StepA = 12, DirA = 13;
const int detPin = 16;

// ===== Motion constants =====
const float STEPS_PER_MM = 9.0;
const float STEPS_PER_DEGREE = 10;
const float TURN_CORRECTION_K = 0.98930;  

enum ChassisDirection { FORWARD, BACKWARD, LEFT, RIGHT }; 
enum Diagonal { FORWARD_RIGHT, FORWARD_LEFT };
int current_half_period = 500;  // half-period: 100 is the minimum number; fastest. 

// ===== Distance measurement results =====
int distanceRear, distanceLeft, distanceRight;

// ===== IMU data streaming =====
unsigned long lastImuUpdate = 0;
const unsigned long IMU_UPDATE_INTERVAL = 200; // 200ms = 5Hz update rate

long totalStepsX = 0;
long totalStepsY = 0;
bool isMoving = false;
int currentMoveDirection = FORWARD;

// —— Inertial detection logic ——  
const float MOTION_THRESHOLD = 1.0;    
unsigned long stationary_time   = 0;
bool was_stationary             = false;

// —— Raw data accumulation for dynamic offset estimation ——  
long  biasAccumulatorRaw = 0;  
int   biasCountRaw       = 0;

// Limit switch pins
const int limitSwitch  = 31; 

// Sound 
const int marioMelody[] = {
  659, 659,   0, 659,   0, 523, 659,   0,
  784,   0,  };
const int marioTempo[] = {
  12, 12, 12, 12, 12, 12, 12, 12,
  12, 12,  };
const int marioLength = sizeof(marioMelody) / sizeof(int);
  // Example: Simple melody (Twinkle Twinkle Little Star)
#define c4   262
#define d4   294
#define e4   330
#define f4   349
#define g4   392
#define a4   440
#define rest 0

const int twinkleMelody[] = {
  c4, c4, g4, g4, a4, a4, g4, rest,
  // f4, f4, e4, e4, d4, d4, c4, rest,
  // g4, g4, f4, f4, e4, e4, d4, rest,
  // g4, g4, f4, f4, e4, e4, d4, rest,
  // c4, c4, g4, g4, a4, a4, g4, rest
};
const int twinkleTempo[] = {
  4, 4, 4, 4, 4, 4, 2, 4,
  // 4, 4, 4, 4, 4, 4, 2, 4,
  // 4, 4, 4, 4, 4, 4, 2, 4,
  // 4, 4, 4, 4, 4, 4, 2, 4,
  // 4, 4, 4, 4, 4, 4, 2, 4
};

const int twinkleLength = sizeof(twinkleMelody) / sizeof(int);

// Define the tempo (BPM - beats per minute)
int tempo = 120; // Example: 120 BPM (quarter notes)



void calibrateGyroZ(int samples = 200) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += mpu.getRotationZ();
    delay(5);
  }
  gyroZ_offset = sum / (float)samples;
  Serial.print("gyroZ_offset = ");
  Serial.println(gyroZ_offset);
}

// Define the array of leds
CRGB leds[NUM_LEDS];


void setup() {
 FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
 FastLED.setBrightness(50);  
  
  playMarioTune();

  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  while (!Serial);  
  while (Serial.available()) Serial.read();  

  // Initialize MPU6050
  mpu.initialize();
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected");
    calibrateGyroZ();
  } else {
    Serial.println("MPU6050 DEAD");
  }

  // Initialize motor control pins
  pinMode(enablePin, OUTPUT); digitalWrite(enablePin, LOW);
  pinMode(detPin, INPUT); digitalWrite(detPin, HIGH);
  pinMode(StepX, OUTPUT); pinMode(DirX, OUTPUT);
  pinMode(StepY, OUTPUT); pinMode(DirY, OUTPUT);
  pinMode(StepZ, OUTPUT); pinMode(DirZ, OUTPUT);
  pinMode(StepA, OUTPUT); pinMode(DirA, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);

    // 初始化蜂鸣器管脚
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // 系统自检完成，蜂鸣一次提示
  // 方法一：如果是被动蜂鸣器（需要频率），用 tone()
  tone(BUZZER_PIN, 1000, 200);   // 1kHz，响 200ms

  trackRotationStart();

  // Initialize distance sensor 
  initializeDistanceSensor();

  // Servo
  servo.attach(27);
  servo.write(60);
  servo.write(90);

  //Limit switch
  pinMode(limitSwitch, INPUT_PULLUP);

  playTwinkleTune();
}
/* C++ CODE
#include <iostream>
using namespace std;

const int STEP = 262; //in mm
const int ROTATION = 90; //in degrees

int x=0;
int y=0;
bool orientation_in_X = true;
int fx=0;
int fy=0;
int dx=0;
int dy=0;

void block2coords(int BLOCK, int& fx, int& fy){
switch (BLOCK) {
    case 0: //START TILE
        fx = 0;
        fy = 0;
        break; 
    case 1:
        fx = 1;
        fy = 0;
        break; 
    case 2:
        fx = 2;
        fy = 0;
        break; 
    case 3:
        fx = 0;
        fy = 1;
        break; 
    case 4:
        fx = 1;
        fy = 1;
        break; 
    case 5:
        fx = 2;
        fy = 1;
        break; 
    case 6:
        fx = 0;
        fy = 2;
        break; 
    case 7:
        fx = 1;
        fy = 2;
        break; 
    case 8:
        fx = 2;
        fy = 2;
        break; 
    case 9:
        fx = 0;
        fy = 3;
        break; 
    case 10:
        fx = 1;
        fy = 3;
        break; 
    case 11:
        fx = 2;
        fy = 3;
        break; 
    case 12:
        fx = 1;
        fy = 4;
        break; 
    case 13:
        fx = 2;
        fy = 4;
        break; 
    case 14: // END TILE
        fx = 0;
        fy = 4;
        break; 

}
}

void find_dx_dy(int x, int y, int fx, int fy, int& dx, int& dy){ //d-> differnece, f->future
    dx = fx-x;
    dy = fy-y;
}

void move(int& x, int& y, int dx, int dy){

    //movement in X axis//////////////////
    if(dx>0){
        //MOVE F by dx * STEP
    }
    else if(dx<0){
        //MOVE B by -dx * STEP
    }

    //movement in Y axis//////////////////
    //RORATE by ROTATION (90)
    if(dy>0){
        //MOVE F by dy * STEP
    }
    else if(dy<0){
        //MOVE B by -dy * STEP
    }    
    //ROTATE by -ROTATION (-90) to return to original orientation

    x += dx;
    y += dy;
}

void sequence_of_moves(int * block_array){
    int block;
    for(int i=0;i<6;i++){
        block = block_array[i];
        block2coords(block, fx, fy);
        find_dx_dy(x, y, fx, fy, dx, dy);
        move(x, y, dx, dy);
        cout<<"CURRENT X is="<<x<<" AND CURRENT Y is="<<y<<endl;
    }

}

int main(){

    int block_array[] = {9,12,4,8,13,14}; //FINAL BLOCK ALWAYS 14!!

    sequence_of_moves(block_array);

    return 0;
}
*/
const int STEP = 262; //in mm
const int ROTATION = 90; //in degrees

int x=0;
int y=0;
bool orientation_in_X = true;
int fx=0;
int fy=0;
int dx=0;
int dy=0;

void block2coords(int BLOCK, int& fx, int& fy){
switch (BLOCK) {
    case 0: //START TILE
        fx = 0;
        fy = 0;
        break; 
    case 1:
        fx = 1;
        fy = 0;
        break; 
    case 2:
        fx = 2;
        fy = 0;
        break; 
    case 3:
        fx = 0;
        fy = 1;
        break; 
    case 4:
        fx = 1;
        fy = 1;
        break; 
    case 5:
        fx = 2;
        fy = 1;
        break; 
    case 6:
        fx = 0;
        fy = 2;
        break; 
    case 7:
        fx = 1;
        fy = 2;
        break; 
    case 8:
        fx = 2;
        fy = 2;
        break; 
    case 9:
        fx = 0;
        fy = 3;
        break; 
    case 10:
        fx = 1;
        fy = 3;
        break; 
    case 11:
        fx = 2;
        fy = 3;
        break; 
    case 12:
        fx = 1;
        fy = 4;
        break; 
    case 13:
        fx = 2;
        fy = 4;
        break; 
    case 14: // END TILE
        fx = 0;
        fy = 4;
        break; 

}
}

void find_dx_dy(int x, int y, int fx, int fy, int& dx, int& dy){ //d-> differnece, f->future
    dx = fx-x;
    dy = fy-y;
}

void move(int& x, int& y, int dx, int dy){
    static String movement = "";
    int distance = 0;

    //movement in X axis//////////////////
    if(dx>0){
        //MOVE F by dx * STEP
      distance = dx*STEP;
      movement = "F"+ String(distance);
      processChassisCommand(movement);
    }
    else if(dx<0){
        //MOVE B by -dx * STEP
        distance = (-dx)*STEP;
      movement = "B"+ String(distance);
      processChassisCommand(movement);
    }

    //movement in Y axis//////////////////
    //RORATE by ROTATION (90)
    if(dy>0){
      //TESTTEST
      processChassisCommand("ML90");
        //MOVE F by dy * STEP
        distance = dy*STEP;
      movement = "F"+ String(distance);
      processChassisCommand(movement);
      processChassisCommand("MR90");
    }
    else if(dy<0){
      processChassisCommand("ML90");
        //MOVE B by -dy * STEP
        distance = (-dy)*STEP;
      movement = "B"+ String(distance);
      processChassisCommand(movement);
      processChassisCommand("MR90");
    }    
    //ROTATE by -ROTATION (-90) to return to original orientation

    x += dx;
    y += dy;
}

void sequence_of_moves(int * block_array){
    int block;
  for (int i = 0; i < 6; i++) {
    block = block_array[i];
    block2coords(block, fx, fy);
    find_dx_dy(x, y, fx, fy, dx, dy);
    move(x, y, dx, dy);

    // Choose LED color based on i
    switch (i) {
      case 0:
        for(int i = 0; i<8; i++){ leds[i] = CRGB::Green; } FastLED.show();
        break;
      case 1:
        for(int i = 0; i<8; i++){ leds[i] = CRGB::Red; } FastLED.show();
        break;
      case 2:
        for(int i = 0; i<8; i++){ leds[i] = CRGB::Blue; } FastLED.show();
        break;
      case 3:
        for(int i = 0; i<8; i++){ leds[i] = CRGB::Pink; } FastLED.show();
        break;
      case 4:
        for(int i = 0; i<8; i++){ leds[i] = CRGB::Cyan; } FastLED.show();
        break;
      case 5:
        for(int i = 0; i<8; i++){ leds[i] = CRGB::Yellow; } FastLED.show();
        break;
      default:
        for(int i = 0; i<8; i++){ leds[i] = CRGB::Black; } FastLED.show(); // fallback color
        break;
    } // green/red/blue/pink/cyan/yellow

    // Flash LEDs
    delay(500);  // on
    for(int i = 0; i<8; i++){ leds[i] = CRGB::Black; } FastLED.show();
  }


}

void loop() {
  static String input = "";

/*
    int block_array[] = {9,12,4,8,13,14}; //FINAL BLOCK ALWAYS 14!!

    sequence_of_moves(block_array);
*/
  static int counter = 0;
  static int block_array[] = {0,0,0,0,0,0}; //FINAL BLOCK ALWAYS 14!!

  // Real-time IMU update (angle integration)
  trackRotationUpdate();
    while (Serial.available() > 0) {
    char chr = Serial.read();

    if (chr == '\n' || chr == '\r') {
      // End of input line
      if (input.length() > 0) {
        int number = input.toInt();
        block_array[counter] = number;
        counter += 1;
        input = ""; // Reset input string
      }

      if (counter == 5) {
        sequence_of_moves(block_array);
        counter = 0; // Reset counter for next sequence
        input = "";
        x=0;
        y=0;
        fx=0;
        fy=0;
        dx=0;
        dy=0;
      }
    } 
    else if (chr == ' ') {
      // Space encountered, end of a number
      if (input.length() > 0) {
        int number = input.toInt();
        block_array[counter] = number;
        counter += 1;
        input = ""; // Reset input string
      }
    }
    else {
      // Build the number string
      input += chr;
    }
  }

 /*
  // read serial input e.g. F100; forward 10 cm MR90; clockwise rotation for 90 degrees
  while (Serial.available() > 0) {
    char chr = Serial.read();

    if (chr == '\n' || chr == '\r') {
      input.trim();  // Remove whitespace
      if (counter == 5) {
        sequence_of_moves(block_array);
      }
      counter = 0;
      input = "";
    } 
    else if(chr != ' '){
      input +=chr;
    }

    else {
      int number = input.toInt();
      input = "";
      block_array[counter]= number;
      counter += 1;
    }
  }

  */
}

void trackRotationStart() {
  totalGyroZ = 0;
  lastTime = millis();
  trackingRotation = true;
}

void trackRotationUpdate() {
  if (!trackingRotation) return;

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // 1) Read raw gyroscope values
  int16_t gz_raw = mpu.getRotationZ();

  // 2) Convert to °/s
  float rate = (gz_raw - gyroZ_offset) / 131.0;

  // —— Stationary detection and raw data accumulation ——  
  if (abs(rate) < MOTION_THRESHOLD) {
    if (!was_stationary) {
      was_stationary  = true;
      stationary_time = now;
      biasAccumulatorRaw = 0;
      biasCountRaw       = 0;
    }
    // Accumulate raw readings
    biasAccumulatorRaw += gz_raw;
    biasCountRaw++;

    // If the system remains stationary for more than 3 seconds, set the offset to the average raw value during that period
    if (now - stationary_time > 3000 && biasCountRaw > 0) {
      gyroZ_offset = biasAccumulatorRaw / biasCountRaw;
      // Restart the next round of accumulation
      stationary_time = now;
      biasAccumulatorRaw = 0;
      biasCountRaw       = 0;
    }
    return;  
  }
  // Once the rate exceeds the threshold, exit the stationary state
  was_stationary = false;

  // Not stationary: normal integration  
  totalGyroZ += rate * dt;
}

/**
 * Serial commands（F/B/SL/SR/ML/MR）
 */
void processChassisCommand(String cmd) {
  // Optional: Set speed (half-cycle)  
  // Sending "Vxxx" to dynamically adjust the half-cycle of speed for all straight-line and lateral movement commands
  if (cmd.startsWith("V")) {
    current_half_period = cmd.substring(1).toInt();
    Serial.print("Speed set: half_period = ");
    Serial.print(current_half_period);
    Serial.println(" us");
  }
  // Move Forward: Fxxx  xxx==correspoonds to cm (or mm) it will travel!!!!
  else if (cmd.startsWith("F")) {
    int dist = cmd.substring(1).toInt();
    moveChassisDir(FORWARD,  dist, current_half_period);
  }
  // Move Back: Bxxx ——  
  else if (cmd.startsWith("B")) {
    int dist = cmd.substring(1).toInt();
    moveChassisDir(BACKWARD, dist, current_half_period);
  }
  // Move Laterally Left: SLxxx ——  
  else if (cmd.startsWith("SL")) {
    int dist = cmd.substring(2).toInt();
    moveChassisSideways(LEFT, dist, current_half_period);
  }
  // Move Laterally Right: SRxxx ——  
  else if (cmd.startsWith("SR")) {
    int dist = cmd.substring(2).toInt();
    moveChassisSideways(RIGHT, dist, current_half_period);
  }
  // IMU-Corrected Left Turn: MLxxx ——  
  else if (cmd.startsWith("ML")) {
    int angle = cmd.substring(2).toInt();
    turnLeftWithMPU(angle * TURN_CORRECTION_K, 100);
  }
  // IMU-Corrected Right Turn: MRxxx ——  
  else if (cmd.startsWith("MR")) {
    int angle = cmd.substring(2).toInt();
    turnRightWithMPU(angle * TURN_CORRECTION_K, 100);
  }
  // Yaw value  ==> measures the yaw rotation angle! 
  else if (cmd == "YAW") {
    Serial.print("IMU_YAW=");
    Serial.println(getYaw());
  }
  // Distance measurement
  else if (cmd == "SCAN") {
    measureDistances();
  }

  else if (cmd == "S") {
    Serial.println("DONE");
  }
  else {
    Serial.println("?");
  }
}

float normalizeAngle(float angle) {
  angle = fmod(angle, 360.0);
  if (angle > 180.0)  angle -= 360.0;
  if (angle <= -180.0) angle += 360.0;
  return angle;
}

void trackRotationStop() {
  trackingRotation = false;
  int roundedAngle = round(totalGyroZ);
}

void resetYaw() {
  totalGyroZ = 0;
  Serial.println("IMU yaw reset to 0.");
}

void sendImuUpdate() {
  Serial.print("IMU:");
  Serial.println(getYaw());
}

// === Chassis movement functions ===
void moveChassisDir(int dir, float mm, int half_period_target) {
  // 1. Start time of recording
  unsigned long t0 = millis();

  // 2. Compute total number of steps
  int steps = round(mm * STEPS_PER_MM);
  Serial.print("Moving dir="); Serial.print(dir);
  Serial.print("  mm="); Serial.print(mm);
  Serial.print(" -> steps="); Serial.println(steps);

  // 3. Number of steps for acceleration/deceleration
  int accel_steps = max((int)(steps * 0.05), 10);
  int decel_start = steps - accel_steps;
  int start_period = half_period_target * 2;
  int current_period;

  // 4. Set direction pin
  setChassisMovementDirection(dir);

  // 5. Main loop: pulse output + bump detection
  for (int i = 0; i < steps; i++) {
    // —— Acceleration phase ——  
    if (i < accel_steps) {
      current_period = start_period - (start_period - half_period_target) * i / accel_steps;
    }
    // —— Deceleration phase ——  
    else if (i >= decel_start) {
      int j = i - decel_start;
      current_period = half_period_target + (start_period - half_period_target) * j / accel_steps;
    }
    // —— Constant speed phase ——  
    else {
      current_period = half_period_target;
    }

    // —— Send pulse ——  
    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(current_period);
    digitalWrite(StepX, LOW);  digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW);  digitalWrite(StepA, LOW);
    delayMicroseconds(current_period);

// === Bump detection; Limit Switch ====
    bool Bump = digitalRead(limitSwitch) == LOW;
    if (Bump) {
      Serial.println("BUMP detected! Stopping and retreating.");
      
      // Determine retreat direction based on original movement direction
      if (dir == FORWARD) {
        // If we were moving forward, retreat backward
        setChassisMovementDirection(BACKWARD);
      } else {
        // Otherwise (we were moving backward), retreat forward
        setChassisMovementDirection(FORWARD);
      }
      
      int retreatSteps = round(100 * STEPS_PER_MM);
      for (int k = 0; k < retreatSteps; k++) {
        digitalWrite(StepX, HIGH);
        digitalWrite(StepY, HIGH);
        digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
        delayMicroseconds(half_period_target);
        digitalWrite(StepX, LOW);  digitalWrite(StepY, LOW);
        digitalWrite(StepZ, LOW);  digitalWrite(StepA, LOW);
        delayMicroseconds(half_period_target);
      }

      unsigned long dt = millis() - t0;
      Serial.print("TIME_MS:"); Serial.println(dt);
      Serial.println("DONE");
      playTwinkleTune();
      return;
    }
  }
  // 6. After completing all steps normally, print elapsed time and completion flag
  unsigned long dt = millis() - t0;
  Serial.print("TIME_MS:"); Serial.println(dt);
  Serial.println("DONE");
}

// Helper function to update total steps based on current direction
void updateTotalSteps(long steps) {
  // Determine which direction we're currently moving
  switch (currentMoveDirection) {
    case FORWARD:
      // Forward motion: Y increases
      totalStepsY += steps;
      break;
    case BACKWARD:
      // Backward motion: Y decreases
      totalStepsY -= steps;
      break;
    case LEFT:
      // Left motion: X decreases
      totalStepsX -= steps;
      break;
    case RIGHT:
      // Right motion: X increases
      totalStepsX += steps;
      break;
  }
}

void setChassisMovementDirection(int dir) {
  // Store current direction for odometry
  currentMoveDirection = dir;
  // left-front Z
  // left-rear X
  // right-front Y
  // right-rear A
  switch (dir) {
    case FORWARD:
      digitalWrite(DirX, HIGH); digitalWrite(DirY, LOW);
      digitalWrite(DirZ, HIGH); digitalWrite(DirA, LOW); break;
    case BACKWARD:
      digitalWrite(DirX, LOW); digitalWrite(DirY, HIGH);
      digitalWrite(DirZ, LOW); digitalWrite(DirA, HIGH); break;
    case LEFT:
      digitalWrite(DirX, LOW); digitalWrite(DirY, LOW);
      digitalWrite(DirZ, HIGH); digitalWrite(DirA, HIGH); break;
    case RIGHT:
      digitalWrite(DirX, HIGH); digitalWrite(DirY, HIGH);
      digitalWrite(DirZ, HIGH); digitalWrite(DirA, HIGH); break;
  }
}

// We didn't use it in this example, but it is worth being kept here. 
void moveDiagonal(Diagonal dir, float mm, int half_period) {
  int steps = round(mm * STEPS_PER_MM);

  // Next, set the direction of the two corresponding wheels based on dir，
  // Only send pulses to these two wheels; the other two do not receive pulses.
  // The following example is written in the order: front-left (Z), back-left (X), front-right (Y), back-right (A)
  if (dir == FORWARD_RIGHT) {
    // Front-right diagonal movement: drive back-left (X+) and front-right (Y+)，
    // Then, 'forward' corresponds to X = HIGH, and 'right' corresponds to Y = HIGH (or you may need to reverse them depending on the actual mechanical setup)
    digitalWrite(DirX, HIGH); // Left-rear single wheel moves forward
    digitalWrite(DirY, HIGH); // Right-front wheel moves forward
    // The remaining two wheels, Z and A, should remain stopped
    // digitalWrite(DirZ, …); 
    // digitalWrite(DirA, …); 
    // But only send pulses to the X and Y wheels：
    for (int i = 0; i < steps; i++) {
      digitalWrite(StepX, HIGH);
      digitalWrite(StepY, HIGH);
      delayMicroseconds(half_period);
      digitalWrite(StepX, LOW);
      digitalWrite(StepY, LOW);
      delayMicroseconds(half_period);
    }
  }
  else if (dir == FORWARD_LEFT) {
    // Front-left diagonal movement: drive front-left (Z+) and back-right (A+)
    digitalWrite(DirZ, HIGH); // Left-front 
    digitalWrite(DirA, HIGH); // Right-back
    for (int i = 0; i < steps; i++) {
      digitalWrite(StepZ, HIGH);
      digitalWrite(StepA, HIGH);
      delayMicroseconds(half_period);
      digitalWrite(StepZ, LOW);
      digitalWrite(StepA, LOW);
      delayMicroseconds(half_period);
    }
  }
  // Other directions can be added as needed
  Serial.println("DONE");
}


// Perform in-place left turn using MPU6050 with IMU feedback
void turnLeftWithMPU(float targetDegrees, int half_period_target) {
  unsigned long t0 = millis();
  trackRotationStart();
  totalGyroZ = 0;
  lastTime = millis();
  lastImuUpdate = millis();

  // According to the LEFT rule given at the top
  digitalWrite(DirX, LOW);
  digitalWrite(DirY, LOW);
  digitalWrite(DirZ, LOW);
  digitalWrite(DirA, LOW);

  const int pulse_interval = half_period_target;

  while (abs(totalGyroZ) < abs(targetDegrees)) {
    // Send pulse
    digitalWrite(StepX, HIGH);
    digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH);
    digitalWrite(StepA, HIGH);
    delayMicroseconds(pulse_interval);
    digitalWrite(StepX, LOW);
    digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW);
    digitalWrite(StepA, LOW);
    delayMicroseconds(pulse_interval);

    trackRotationUpdate();
    if (millis() - lastImuUpdate >= IMU_UPDATE_INTERVAL) {
      sendImuUpdate();
      lastImuUpdate = millis();
    }
  }

  trackRotationStop();
  unsigned long dt = millis() - t0;
  Serial.print("TIME_MS:"); Serial.println(dt);
  Serial.println("DONE");
}


// Use MPU6050 to perform an in-place right turn with IMU feedback

void turnRightWithMPU(float targetDegrees, int half_period_target) {
  unsigned long t0 = millis();

  trackRotationStart();
  totalGyroZ = 0;
  lastTime = millis();
  lastImuUpdate = millis();

  digitalWrite(DirX, HIGH);
  digitalWrite(DirY, HIGH);
  digitalWrite(DirZ, HIGH);
  digitalWrite(DirA, HIGH);

  const int pulse_interval = half_period_target;

  while (abs(totalGyroZ) < abs(targetDegrees)) {

    digitalWrite(StepX, HIGH); digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH); digitalWrite(StepA, HIGH);
    delayMicroseconds(pulse_interval);
    digitalWrite(StepX, LOW);  digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW);  digitalWrite(StepA, LOW);
    delayMicroseconds(pulse_interval);

    trackRotationUpdate();

    if (millis() - lastImuUpdate >= IMU_UPDATE_INTERVAL) {
      sendImuUpdate();
      lastImuUpdate = millis();
    }
  }

  trackRotationStop();

  unsigned long dt = millis() - t0;
  Serial.print("TIME_MS:"); Serial.println(dt);
  Serial.println("DONE");
}


// Move side ways
void moveChassisSideways(int dir, float mm, int half_period) {
  unsigned long t0 = millis();

  int steps = round(mm * STEPS_PER_MM);
  Serial.print("Sideways dir="); Serial.print(dir == LEFT ? "LEFT" : "RIGHT");
  Serial.print("  mm="); Serial.print(mm);
  Serial.print(" -> steps="); Serial.println(steps);

  int accel_steps  = max(steps / 10, 10);
  int decel_start  = steps - accel_steps;
  int start_period = half_period * 2;
  int current_period;

  if (dir == LEFT) {
    digitalWrite(DirX, LOW);
    digitalWrite(DirY, HIGH);
    digitalWrite(DirZ, HIGH);
    digitalWrite(DirA, LOW);
} else { // RIGHT
    digitalWrite(DirX, HIGH);  
    digitalWrite(DirY, LOW);   
    digitalWrite(DirZ, LOW);   
    digitalWrite(DirA, HIGH);  
}

  for (int i = 0; i < steps; i++) {
    if (i < accel_steps) {
      current_period = start_period - (start_period - half_period) * i / accel_steps;
    }
    else if (i >= decel_start) {
      int j = i - decel_start;
      current_period = half_period + (start_period - half_period) * j / accel_steps;
    }
    else {
      current_period = half_period;
    }

    digitalWrite(StepX, LOW); digitalWrite(StepY, LOW);
    digitalWrite(StepZ, LOW); digitalWrite(StepA, LOW);
    delayMicroseconds(current_period);
    digitalWrite(StepX, HIGH);  digitalWrite(StepY, HIGH);
    digitalWrite(StepZ, HIGH);  digitalWrite(StepA, HIGH);
    delayMicroseconds(current_period);
  }

  unsigned long dt = millis() - t0;
  Serial.print("TIME_MS:"); Serial.println(dt);
  Serial.println("DONE");
}

// IMU Yaw value:
float getYaw() {
  return -normalizeAngle(totalGyroZ);
}

void initializeDistanceSensor() {
  pinMode(XSHUT_CENTER, OUTPUT);
  digitalWrite(XSHUT_CENTER, LOW);

// Initialize center sensor
  digitalWrite(XSHUT_CENTER, HIGH);

  delay(10);
  if (sensorDist.init()) {
    sensorDist.setAddress(0x30);
    sensorDist.setDistanceMode(VL53L1X::Medium);
    sensorDist.setMeasurementTimingBudget(50000);
    sensorDist.startContinuous(50);
    Serial.println("CENTER sensor initialized at 0x31");
  } else {
    Serial.println("CENTER sensor init FAILED");
  }

}

// === Distance scanning functions ===
void measureDistances() {
  unsigned long t0 = millis();

  servo.write(25);
  delay(1000);
  distanceLeft  = sensorDist.read();
  if (sensorDist.timeoutOccurred()) {
    Serial.println("LEFT sensor TIMEOUT");
    distanceLeft = 0;
  }

  servo.write(90);
  delay(1000);
  distanceRear  = sensorDist.read();
  if (sensorDist.timeoutOccurred()) {
    Serial.println("CENTER sensor TIMEOUT");
    distanceRear = 0;
  }
  
  servo.write(155);
  delay(1000);
  distanceRight = sensorDist.read();
  if (sensorDist.timeoutOccurred()) {
    Serial.println("RIGHT sensor TIMEOUT");
    distanceRight = 0;
  }

  Serial.print("SCAN_RESULT:L="); Serial.println(distanceLeft);
  Serial.print("SCAN_RESULT:C="); Serial.println(distanceRear);
  Serial.print("SCAN_RESULT:R="); Serial.println(distanceRight);
  Serial.print("IMU_YAW=");    Serial.println(getYaw());

  Serial.print("TIME_MS:"); Serial.println(millis() - t0);
  Serial.println("DONE");
  servo.write(90);

} 

void playMarioTune() {
  for (int i = 0; i < marioLength; i++) {
    int note = marioMelody[i];
    int duration = 1000 / marioTempo[i];  // 基本拍长：1000ms/节拍数
    if (note == 0) {
      // 休止符
      delay(duration);
    } else {
      tone(BUZZER_PIN, note, duration);
      delay(duration * 1.3);  // 多延一点让音符更分明
    }
  }
  noTone(BUZZER_PIN);  // 停止蜂鸣
}

void playTwinkleTune() {
  // Iterate through each note of the melody
  for (int i = 0; i < twinkleLength; i++) {
    
    // Get the current note and tempo value from the arrays
    int note = twinkleMelody[i];
    int tempo = twinkleTempo[i];

    // Calculate the duration of the note in milliseconds.
    // A larger tempo number means a shorter note duration.
    int duration = 1000 / tempo;

    // Check if the current note is a rest (a pause)
    if (note == rest) {
      delay(duration);
    } else {
      // Play the note on the buzzer for the calculated duration
      tone(BUZZER_PIN, note, duration);
      
      // Add a brief pause between notes to make them more distinct
      delay(duration * 1.3);
    }
  }
  // Stop any sound from the buzzer after the tune is finished
  noTone(BUZZER_PIN);
}

