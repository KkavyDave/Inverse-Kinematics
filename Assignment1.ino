#include <Braccio.h> 
#include<Servo.h>
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_rot;
Servo wrist_ver;
Servo gripper;
// Link lengths (in cm) for the Braccio arm
const float L1 = 12.5; // (shoulder to elbow)
const float L2 = 12.5; // (elbow to wrist)
const float L3 = 7.15; // (wrist to end-effector)

void setup() {
  Serial.begin(9600); // Initialize serial communication
  delay(1000);
  Braccio.begin(); // Initialize Braccio
}

void loop() {
  // Enter end-effector coordinates and orientation
  float valuex = 10, valuey = 20, gamma=90;
  // Call inverse kinematics function and move the Braccio arm
  moveBraccio(valuex, valuey, gamma);
  delay(1000);

void moveBraccio(float valuex, float valuey, float gamma) {
  // Calculate position of P3
  float valuea = valuex - (L3 * cos(radians(gamma)));
  float valueb = valuey - (L3 * sin(radians(gamma)));
  float valueC = sqrt(pow(valuea, 2) + pow(valueb, 2));

  // Check if position is within reachable workspace
  if ((L1 + L2) > valueC) {
    // Calculate angles
    float alpha = degrees(acos((pow(L1, 2) + pow(valueC, 2) - pow(L2, 2)) / (2 * L1 * valueC)));
    float beta = degrees(acos((pow(L1, 2) + pow(L2, 2) - pow(valueC, 2)) / (2 * L1 * L2)));

    // Joint angles for elbow-down configuration
    float theta1 = 180 - beta;
    float theta2 = degrees(atan2(valueb, valuea)) - alpha;
    float theta3 = gamma – theta2 – theta1;
    // Set servo angles and move the Braccio arm
    Braccio.ServoMovement(20 ,0,int(theta1),90+int(theta2),90+int(theta3),0,73); 
  }
}
