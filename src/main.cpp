
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int numServos = 5;
const int servoPins[numServos] = {0, 1, 2, 3, 4};
const int servoMin = 150; // Minimum servo pulse length
const int ServoMid = 375; // Midpoint servo pulse length
const int servoMax = 600; // Maximum servo pulse length

// Servo limits
const int servoLimits[numServos][2] = {{0, 180}, {10, 90}, {80, 140}, {80, 130}, {50, 150}}; // Adjust limits for each servo

const char *servoNames[numServos] = {"Servo Base", "Servo Swing", "Servo Arm One", "Servo Arm Two", "Servo Gripper"};

// Servo positions for smooth control
int currentServoPos[numServos] = {90, 90, 90, 90, 90}; // Initial positions
int targetServoPos[numServos] = {90, 90, 90, 90, 90};

// Link lengths (assume two-link planar arm for simplicity)
float link1 = 10.0; // Length of arm link 1
float link2 = 10.0; // Length of arm link 2

// Speed control variable
unsigned long speedDelay = 10; // Speed control (lower is faster)
unsigned long lastUpdate = 0;


void setServoTarget(int servoNum, int targetPos)
{
  // Ensure target position is within servo limits
  targetPos = constrain(targetPos, servoLimits[servoNum][0], servoLimits[servoNum][1]);
  targetServoPos[servoNum] = targetPos;

}


// Updated function to smoothly move servos
void moveServoSmooth()
{
  for (int i =0; i < numServos; i++) // Check each servo at each iteration 
  {
    // Compare curent servo position to target and skip if servo already at target
    if (currentServoPos[i] != targetServoPos[i]) 
    {
      int diff = targetServoPos[i] - currentServoPos[i];

      // Check if the difference is small enough to set directly
      if (abs(diff) <= 1) 
      {
        currentServoPos[i] = targetServoPos[i];
      } 
      else 
      {
        currentServoPos[i] += (diff > 0) ? 1 : -1; // If difference is larger then move one step to the target position
      }

      int pulse = map(currentServoPos[i], 0, 180, servoMin, servoMax); // Set pulse width based on current position
      pwm.setPWM(servoPins[i], 0, pulse); // Update each servo position
    }
  }
}

// Inverse Kinematics for planar arm
void inverseKinematics(float x, float y)
{
  float d = sqrt(x * x + y * y); // Distance from base to end-effector
  if (d > (link1 + link2) || d < fabs(link1 - link2))
  {
    Serial.println("Target unreachable"); // Safety check: out of reach
    return;
  }

  float angle2 = acos((x * x + y * y - link1 * link1 - link2 * link2) / (2 * link1 * link2)); // Elbow angle
  float angle1 = atan2(y, x) - atan2(link2 * sin(angle2), link1 + link2 * cos(angle2));       // Shoulder angle

  // Convert angles to degrees
  int servoAngle1 = (int)(angle1 * 180.0 / PI);
  int servoAngle2 = (int)(angle2 * 180.0 / PI);

  // Move servos (Servo Arm One and Servo Arm Two) to new positions
  setServoTarget(2, servoAngle1);
  setServoTarget(3, servoAngle2);
}

// Safety mechanism to prevent out-of-bounds or dangerous movements
bool isSafePosition(float x, float y)
{
  // Add safety zones where the arm is restricted
  if (x < 0 || y < 0 || x > (link1 + link2) || y > 10)
  { // Example soft limits
    Serial.println("Unsafe position!");
    return false;
  }
  return true;
}

void setup()
{
  Serial.begin(115200); // Set up Serial communication at 115200 baud rate
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);

  Serial.println("Ready to receive commands. Use format: S<num> <angle>");
 
for (int i = 0; i < numServos; i++) {
  pwm.setPWM(servoPins[i], 0, ServoMid);  // 300 ticks for all
}

 
}

void loop()
{
  // Read serial input to control the servos
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.startsWith("S"))
    {
      int servoNum = input.substring(1, 2).toInt();
      int servoValue = input.substring(3).toInt();

      if (servoNum >= 0 && servoNum < numServos)
      {
        setServoTarget(servoNum, servoValue);
        Serial.print("Moving Servo ");
        Serial.print(servoNum);
        Serial.print(" to ");
        Serial.println(servoValue);
      }
      else
      {
        Serial.println("Invalid servo number.");
      }
    }
    else
    {
      Serial.println("Invalid command. Use format: S<num> <angle>");
    }
  }


  
    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdate >= speedDelay)
    {
      lastUpdate = currentMillis;
      moveServoSmooth(); // Update all servos towards their target positions
    }

}