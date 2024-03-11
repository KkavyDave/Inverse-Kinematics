Use a USB cable to connect the Arduino UNO to the PC and upload the provided code by selecting 'Tools -> Board -> Arduino UNO' and choosing the correct serial port in 'Tools -> Port'.

This code is an Arduino sketch for controlling a robotic arm known as Braccio using inverse kinematics. The robotic arm has three degrees of freedom (DOF) controlled by servo motors at the base, shoulder, elbow, wrist rotation, wrist vertical, and gripper.

Libraries Included:

Braccio.h: This library provides functions to control the Braccio robotic arm.
Servo.h: This library allows control of servo motors.

moveBraccio() Function:

This function calculates the servo angles required to position the end-effector of the robotic arm at the specified coordinates and orientation using inverse kinematics.
It takes three parameters: valuex, valuey, and gamma, which represent the x and y coordinates of the end-effector and the orientation angle respectively.

Inverse Kinematics Calculation:

The function first calculates the position of point P3 using trigonometry based on the input coordinates and orientation.
It then checks if the desired position is within the reachable workspace of the robotic arm.
If the position is reachable, it calculates the joint angles (theta1, theta2, and theta3) required for the elbow-down configuration of the arm.

Working Video Link : https://youtu.be/JGDx4rtl6qw 

