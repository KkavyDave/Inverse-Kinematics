#include <Braccio.h>
#include <Servo.h>

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;

// Function prototype for inverse kinematics for 3 DOF
void inv_kinematics(float valuex, float valuey, float gamma);

// Link lengths (in cm)
const float Length1 = 12.5; // Shoulder - elbow
const float Length2 = 12.5; // Elbow - wrist
const float Length3 = 7.15; // Wrist to end-effector

void setup() {
  Serial.begin(9600);
  delay(1000);
  Braccio.begin();
}

void loop() {
  // Enter end-effector coordinates and orientation
  float valuex = -15, valuey = 15, gamma = 60;
  // Call inv_kinematics function and move the robotic arm
  inv_kinematics(valuex, valuey, gamma);
  delay(1000);
}

void inv_kinematics(float valuex, float valuey, float gamma) {
  // Calculate position of P3
  float valuea = valuex - (Length3 * cos(radians(gamma)));
  float valueb = valuey - (Length3 * sin(radians(gamma)));
  float valueC = sqrt(pow(valuea, 2) + pow(valueb, 2));

  // Check if position is within reachable workspace
  if ((L1 + L2) >= valueC) {
    // Calculate angles by using cosine rules at L1 and L2
    float alpha = degrees(acos((pow(Length1, 2) + pow(valueC, 2) - pow(Length2, 2)) / (2 * Length1 * valueC)));
    float beta = degrees(acos((pow(Length1, 2) + pow(Length2, 2) - pow(valueC, 2)) / (2 * Length1 * Length2)));

    // Joint angles for elbow-down configuration
    float theta1 = 180 - beta;
    float theta2 = degrees(atan2(valueb, valuea)) - alpha;
    float theta3 = gamma – (theta2 + theta1);
    // Set servo angles and move the Braccio arm
    Braccio.ServoMovement(20, 0, int(theta1), 90 + int(theta2), 90 + int(theta3), 0, 73);
  }
}

