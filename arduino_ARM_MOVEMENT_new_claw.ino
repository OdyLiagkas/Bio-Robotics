/**
 * This sketch receives simple serial commands (e.g., "G0", "G1") to trigger
 * predefined arm movements and supports direct control of individual servos.
 */

// ====== Library Includes ======
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <FastLED.h>

// ====== LED Constants ======
#define NUM_LEDS 8
#define DATA_PIN 31
CRGB leds[NUM_LEDS];

// ====== Servo Control Constants ======
#define SERVOMIN_PULSE 102 // Minimum pulse length out of 4096
#define SERVOMAX_PULSE 512 // Maximum pulse length out of 4096

// Physical angle limits for each servo (in degrees).
const int SERVO_PHY_MIN[16] = {0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const int SERVO_PHY_MAX[16] = {180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180, 180};

// ====== Global Objects ======
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
int servoChannels[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
static int seedPos = 30;  // CHANGE THE ORIGINAL POSITION FOR SEED DROPPER

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(30);
for(int i = 0; i<8; i++){
 leds[i] = CRGB::White;
 }
 FastLED.show();
  Wire.begin();
  Wire.setClock(400000);
  if (!pwm.begin()) {
    Serial.println("PCA9685 Dead");
    while (1);
  } else {
    pwm.setPWMFreq(50);
    Serial.println("PCA9685 Ready");
  }
  //setSeedPose();
  moveServoSmoothly(servoChannels[7], 28, seedPos, 10, 7);   
  //delay(2000);
  //moveToRightBin();
  setDefaultPose();
}

void loop() {
  if (Serial.available() > 0) {
    static String input = "";
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      input.trim();
      if (input.length() > 0) {
        processGripperCommand(input);
        input = "";
      }
    } else {
      input += ch;
    }
  }
}

// Routes incoming serial commands to the appropriate function.
void processGripperCommand(String cmd) {
  if (cmd == "G0") { setDefaultPose(); } // Arm extended vertically & claw set to closed
  else if (cmd == "G1") { moveToTop(); Serial.println("DONE");} // pointed upwards w/ claw closed
  else if (cmd == "G2") { moveToLeftBin(); } // Arm pulled inside frame w/ claw closed
  else if (cmd == "GL") { rotateBase(155); rotateString(10);} // Claw wide open
  else if (cmd == "GC") { rotateBase(10); } // Claw slightly (med) opened 
  else if (cmd == "GR") { rotateBase(150); } // Claw fully closed
  else if (cmd == "BR") { releaseRightBin();} // Release Right Bin
  else if (cmd == "BL") { releaseLeftBin();} // Release Right Bin
  else if (cmd == "BB") { releaseBothBins();} // Release Right Bin
  else if (cmd == "SD") {releasenextSeed(seedPos);}
  else if (cmd == "SR") {moveServoSmoothly(servoChannels[7], seedPos, 30, 10, 7);}
  else if (cmd == "SP") {setSeedPose();}
  else if (cmd == "TESTG") {GrabLeftPlant();}
  else if (cmd == "GRABR"){ GrabRightPlant(); delay(500); rotateBase(145); delay(500); moveToRightBin(); delay(1000); rotateBase(10); delay(2000); setDefaultPose();}
  else if (cmd == "GRABL"){ GrabLeftPlant(); delay(500);  rotateBase(145); delay(500);  moveToLeftBin(); delay(1000); rotateBase(10); delay(2000); setDefaultPose();}
  else if (cmd=="TEST") {rotateMotor(12,130);}
  // REVISED: Added handler for the ZERO command to support Python handshake
  else if (cmd == "ZERO") {
    Serial.println("BASELINE_RESET");
  }
  else { Serial.println("?"); }
}
// Moves a servo to release the next seed.
void releasenextSeed(int& seedPos) {
  moveServoSmoothly(servoChannels[7], seedPos, seedPos+20, 10, 7);
  seedPos +=10;
  Serial.print("TIME_MS:0, DONE"); // Simplified response
  Serial.println();
}
// Moves a servo for scanning purposes.
void rotateString(int targetAngle) {
  // Assumes the scanner servo starts at a 90-degree center point
  moveServoSmoothly(servoChannels[12], 10, targetAngle, 10, 12);
  Serial.print("TIME_MS:0, DONE"); // Simplified response
  Serial.println();
}

void rotateMotor(int targetMotor,int targetAngle) {
  // Assumes the scanner servo starts at a 90-degree center point
  moveServoSmoothly(servoChannels[targetMotor], 10, targetAngle, 10, targetMotor);
  delay(200);
  moveServoSmoothly(servoChannels[targetMotor],targetAngle,10, 10, targetMotor);
  Serial.print("TIME_MS:0, DONE"); // Simplified response
  Serial.println();
}

// Moves a servo for scanning purposes.
void rotateBase(int targetAngle) {
  // Assumes the scanner servo starts at a 90-degree center point
  moveServoSmoothly(servoChannels[12], 10, targetAngle, 10, 12);
  //Serial.print("TIME_MS:0, DONE"); // Simplified response
  //Serial.println();
}

// Smoothly moves a single servo from a start to an end angle.
void moveServoSmoothly(int channel, int startAngle, int endAngle, int stepDelay, int servoIndex) {
  if (startAngle < endAngle) {
    for (int angle = startAngle; angle <= endAngle; angle++) {
      pwm.setPWM(channel, 0, angleToPulse(angle, servoIndex));
      delay(stepDelay);
    }
  } else {
    for (int angle = startAngle; angle >= endAngle; angle--) {
      pwm.setPWM(channel, 0, angleToPulse(angle, servoIndex));
      delay(stepDelay);
    }
  }
}

// Converts an angle in degrees to a PWM pulse value.
int angleToPulse(int angle, int servoIndex) {
  angle = constrain(angle, SERVO_PHY_MIN[servoIndex], SERVO_PHY_MAX[servoIndex]);
  return map(angle, 0, 180, SERVOMIN_PULSE, SERVOMAX_PULSE);
}

// --- Pre-existing Arm Pose Functions ---
void setDefaultPose() {
  
  moveServoSmoothly(servoChannels[0], 48, 50, 10, 0); // Eblow
  moveServoSmoothly(servoChannels[1], 160, 170, 10, 1);  // Elbow2
  moveServoSmoothly(servoChannels[2], 78, 80, 10, 2);  // Wrist
  moveServoSmoothly(servoChannels[5], 108, 113, 10, 5);  // Wrist2
  moveServoSmoothly(servoChannels[12], 145, 10, 10, 12); // Claw
  moveServoSmoothly(servoChannels[4], 88, 98, 10, 4); // Soulder
  Serial.println("DONE");
}
void moveToTop() {
  moveServoSmoothly(servoChannels[4], 98, 105, 10, 4); // Soulder
  moveServoSmoothly(servoChannels[0], 50, 30, 10, 0); // Elbow
  moveServoSmoothly(servoChannels[1], 170, 80, 10, 1);  // Elbow2
  moveServoSmoothly(servoChannels[2], 80, 80, 10, 2);  // Wrist
  moveServoSmoothly(servoChannels[5], 110, 110, 10, 5);  // Wrist2
  moveServoSmoothly(servoChannels[3], 110, 40, 3, 3);  // Claw
  //Serial.println("DONE");
}

void GrabRightPlant() {
    
  moveServoSmoothly(servoChannels[1], 170, 90, 10, 1);  // Elbow2
  moveServoSmoothly(servoChannels[0], 50, 90, 10, 0); // Eblow
  moveServoSmoothly(servoChannels[4], 98, 83, 30, 4); // Soulder
  moveServoSmoothly(servoChannels[2], 80, 90, 10, 2);  // Wrist
  moveServoSmoothly(servoChannels[5], 113, 148, 10, 5);  // Wrist2
  moveServoSmoothly(servoChannels[4], 83, 73, 80, 4); // Soulder
  
}

void GrabLeftPlant() {
    
  moveServoSmoothly(servoChannels[1], 170, 90, 10, 1);  // Elbow2
  moveServoSmoothly(servoChannels[0], 50, 90, 10, 0); // Eblow
  moveServoSmoothly(servoChannels[4], 98, 132, 30, 4); // Soulder
  moveServoSmoothly(servoChannels[2], 80, 90, 10, 2);  // Wrist
  moveServoSmoothly(servoChannels[5], 113, 148, 10, 5);  // Wrist2
  moveServoSmoothly(servoChannels[4], 132, 123, 80, 4); // Soulder
}

void moveToRightBin() {
  
  moveServoSmoothly(servoChannels[0], 90, 30, 10, 0); // Elbow
  moveServoSmoothly(servoChannels[1], 90, 80, 10, 1);  // Elbow2
  moveServoSmoothly(servoChannels[2], 90, 80, 10, 2);  // Wrist
  moveServoSmoothly(servoChannels[5], 148, 110, 10, 5);  // Wrist2
  moveServoSmoothly(servoChannels[4], 73, 105, 10, 4); // Soulder
  //moveServoSmoothly(servoChannels[3], 145, 40, 3, 3);  // Claw
  //Serial.println("DONE");
}

void moveToLeftBin() {
  moveServoSmoothly(servoChannels[0], 90, 30, 10, 0); // Elbow
  moveServoSmoothly(servoChannels[1], 90, 80, 10, 1);  // Elbow2
  moveServoSmoothly(servoChannels[2], 90, 80, 10, 2);  // Wrist
  moveServoSmoothly(servoChannels[5], 148, 110, 10, 5);  // Wrist2
  //moveServoSmoothly(servoChannels[3], 110, 40, 3, 3);  // Claw
  moveServoSmoothly(servoChannels[4], 126, 85, 10, 4); // Soulder
  //Serial.println("DONE");
}

void setSeedPose() {
  moveServoSmoothly(servoChannels[4], 98, 108, 10, 4); // Soulder
   moveServoSmoothly(servoChannels[0], 173, 100, 10, 0); // Elbow
  moveServoSmoothly(servoChannels[2], 41, 41, 10, 2);  // Wrist

  moveServoSmoothly(servoChannels[1], 6, 50, 10, 1);  // Elbow2

  moveServoSmoothly(servoChannels[0], 100, 40, 10, 0); // Elbow

  
  //moveServoSmoothly(servoChannels[2], 41, 41, 10, 2);  // Wrist
  moveServoSmoothly(servoChannels[3], 90, 40, 3, 3);  // Claw

  moveServoSmoothly(servoChannels[0], 40, 30, 10, 0); // Elbow
  moveServoSmoothly(servoChannels[4], 88, 170, 10, 4); // Soulder
  Serial.println("DONE");
}




void releaseRightBin(){
  moveServoSmoothly(servoChannels[15], 38, 100, 40, 15);
  moveServoSmoothly(servoChannels[15], 100, 38, 10, 15);
  Serial.println("DONE");
}

void releaseLeftBin(){
  moveServoSmoothly(servoChannels[14], 38, 100, 40, 14);
  moveServoSmoothly(servoChannels[14], 100, 38, 10, 14);
  Serial.println("DONE");
}

void releaseBothBins(){
  moveServoSmoothly(servoChannels[15], 38, 100, 40, 15);
  moveServoSmoothly(servoChannels[14], 38, 100, 40, 14);
  moveServoSmoothly(servoChannels[15], 100, 38, 10, 15);
  moveServoSmoothly(servoChannels[14], 100, 38, 10, 14);
  Serial.println("DONE");
}