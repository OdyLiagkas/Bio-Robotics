/**
 * This sketch receives simple serial commands (e.g., "G0", "G1") to trigger
 * predefined arm movements and supports direct control of individual servos.
 * Make sure using 115200 baud
 */

// ====== Library Includes ======
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <FastLED.h>

// ====== LED Constants ======
#define NUM_LEDS 45
#define DATA_PIN 45
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

void setup() {
  Serial.begin(115200);
 FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
 FastLED.setBrightness(50);
  Wire.begin();
  Wire.setClock(400000);
  if (!pwm.begin()) {
    Serial.println("PCA9685 Dead");
    while (1);
  } else {
    pwm.setPWMFreq(50);
    Serial.println("PCA9685 Ready");
  }
  processGripperCommand("GC");
}

void loop() {                   // make arduino listen to the Pi, and when it detects one thing, make it do LED things and (if) movement of arm
   
   // Turn on white LEDs
    for(int i = 0; i<45; i++){
   leds[i] = CRGB::White;
    }
    FastLED.show();

  if (Serial.available() > 0) {
    static String input = "";
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      input.trim();
      if (input.length() > 0) {
        //processGripperCommand(input);   ADD DIFFERENT OUTCOMES BASED ON good, bad, gs, bs LEDS and grabbing
        if(input == "Good"){
    for(int i = 0; i<45; i++){
   leds[i] = CRGB::Green;
    }
    FastLED.show();            
        }
        else if(input == "Bad"){
    for(int i = 0; i<45; i++){
   leds[i] = CRGB::Red;
    }
    FastLED.show();
        }
        else if(input == "GS"){
    for(int i = 0; i<22; i++){
   leds[i] = CRGB::Green;
    }
    FastLED.show();
        }
        else if(input=="BS"){
    for(int i = 0; i<22; i++){
   leds[i] = CRGB::Red;
    }
    FastLED.show();
        }

        delay(500); // time the leds are going to be green/red
        processGripperCommand("GR");processGripperCommand("G2");processGripperCommand("GC");processGripperCommand("G0");processGripperCommand("GC");
        for(int i = 0; i<45; i++){leds[i] = CRGB::White;}FastLED.show();

        input = "";
        // Print DONE to confirm the completion of the action
        Serial.println("DONE");
        
      }
    } 
    else {
      input += ch;
    }
  }
}

// Routes incoming serial commands to the appropriate function.
void processGripperCommand(String cmd) {
  if (cmd == "G0") { setDefaultPose(); } // Arm extended vertically & claw set to closed
  else if (cmd == "G1") { setgGripperPick(); } // pointed upwards w/ claw closed
  else if (cmd == "G2") { releasegood(); } // Arm pulled inside frame w/ claw closed
  else if (cmd == "GL") { rotateBase(45); } // Claw wide open
  else if (cmd == "GC") { rotateBase(90); } // Claw slightly (med) opened 
  else if (cmd == "GR") { rotateBase(135); } // Claw fully closed
  // REVISED: Added handler for the ZERO command to support Python handshake
  else if (cmd == "ZERO") {
    Serial.println("BASELINE_RESET");
  }
  else { Serial.println("?"); }
}

// Moves a servo for scanning purposes.
void rotateBase(int targetAngle) {
  // Assumes the scanner servo starts at a 90-degree center point
  moveServoSmoothly(servoChannels[3], 90, targetAngle, 10, 4);
  Serial.print("TIME_MS:0, DONE"); // Simplified response
  Serial.println();
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
  moveServoSmoothly(servoChannels[0], 70, 100, 10, 0); // Base
  moveServoSmoothly(servoChannels[1], 60, 70, 10, 1);  // Shoulder
  moveServoSmoothly(servoChannels[2], 60, 90, 10, 2);  // Elbow
  moveServoSmoothly(servoChannels[3], 90, 90, 10, 3); // Claw
  Serial.println("DONE");
}

void setgGripperPick() {
  moveServoSmoothly(servoChannels[0], 100, 80, 10, 0); // Base
  moveServoSmoothly(servoChannels[1], 70, 50, 10, 1);  // Shoulder
  moveServoSmoothly(servoChannels[2], 90, 70, 10, 2);  // Elbow
  moveServoSmoothly(servoChannels[3], 130, 175, 10, 3); // Claw
  Serial.println("DONE");
}

void releasegood() {
  moveServoSmoothly(servoChannels[0], 100, 130, 10, 0); // Base
  moveServoSmoothly(servoChannels[1], 70, 100, 10, 1);  // Shoulder
  moveServoSmoothly(servoChannels[2], 90, 120, 10, 2);  // Elbow
  moveServoSmoothly(servoChannels[3], 170, 160, 3, 3);  // Claw
  Serial.println("DONE");
}

// New arm code function: Start Pose 
// Arm straight down, claw open, claw at 90° relative to arm.
void setStartPose() {
  // Adjust angles based on your mechanical calibration
  moveServoSmoothly(servoChannels[0], 90, 90, 10, 0);   // Base: centered
  moveServoSmoothly(servoChannels[1], 70, 120, 10, 1);  // Shoulder: lowered
  moveServoSmoothly(servoChannels[2], 90, 150, 10, 2);  // Elbow: extended down
  moveServoSmoothly(servoChannels[3], 130, 90, 10, 3);  // Claw: open (adjust as needed)

  Serial.println("START_POSE_DONE");
}
