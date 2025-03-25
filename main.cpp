/*|SCARA Simulator|------------------------------------------------------------
#
# Project: ROBT 1270 - SCARA Simulator Basic Control
# Program: scara_main.c
#
# Description:
#   This program demonstrates basic control over the SCARA Robot Simulator. The
# simulator moves with only commands to control the motor angles. To move to a
# desired (x, y) coordinate requires the implementation of inverse kinematics.
# Other available commands are listed below.
#
# Other Programs:
#   ScaraRobotSimulator.exe (Version 4.3)
#
# Simulator Commands:
#  - PEN_UP
#  - PEN_DOWN
#  - PEN_COLOR <r> <g> <b>
#  - CYCLE_PEN_COLORS ON/OFF
#  - ROTATE_JOINT ANG1 <deg1> ANG2 <deg2>
#  - CLEAR_TRACE
#  - CLEAR_REMOTE_COMMAND_LOG
#  - CLEAR_POSITION_LOG
#  - SHUTDOWN_SIMULATION
#  - MOTOR_SPEED HIGH/MEDIUM/LOW
#  - MESSAGE <"string">
#  - HOME
#  - END
#
# Other Information:
#  - IP Address: 127.0.0.1 Port 1270
#  - BCIT Blue: 10 64 109
#  - If using VS Code, add the following args to tasks.json g++ build task.
#     "-std=c++11"
#		"-lwsock32"
#		"-Wno-deprecated"
#  - Also change the "${file}" argument to "*.cpp". This is a .cpp wildcard
#  - that will grab other .cpp files in the folder.
#
# Author: <Name>
# Date Created: <Date>
# Last Modified: <Today>
# -----------------------------------------------------------------------------*/
#pragma warning(disable:4996)  // get rid of some microsoft-specific warnings.

/*|Includes|-------------------------------------------------------------------*/
#include <stdio.h>  // <list of functions used>
#include <math.h>   // <list of functions used>
#include "robot.h"  // <list of functions used> // NOTE: DO NOT REMOVE.

/*|Globals|--------------------------------------------------------------------*/
CRobot robot;     // the global robot Class instance.  Can be used everywhere
                  // robot.Initialize()
                  // robot.Send()
                  // robot.Close()

/*|CONSTANTS|------------------------------------------------------------------*/
#define L1                    350.0
#define L2                    250.0
#define MAX_ABS_THETA1_DEG    150.0
#define MAX_ABS_THETA2_DEG    170.0
#define MAX_STRING            256
#define PI                    3.14159265358979323846
#define LEFT_ARM_SOLUTION     0
#define RIGHT_ARM_SOLUTION    1

/*|Function Prototypes|--------------------------------------------------------*/
//double DegToRad(double);  // returns angle in radians from input angle in degrees
//double RadToDeg(double);  // returns angle in degrees from input angle in radians


int main(){
   // Variables
   char commandString[MAX_STRING];   // string for simulator commands
   double thetaDeg1,thetaDeg2;

   // Open a connection with the simulator
   if(!robot.Initialize()) return 0;
   
   // here are examples of how to send the robot commands. must use robot.
   robot.Send("CYCLE_PEN_COLORS OFF\n");  // DON'T FORGET \n AT THE END OF _EVERY_ COMMAND
   robot.Send("PEN_COLOR 0 0 255\n");
   robot.Send("ROTATE_JOINT ANG1 150.00 ANG2 90.00\n"); // here is an explicit way to move the robot when you know absolute values of angles
   robot.Send("PEN_COLOR 255 0 0\n");
   
   // And here is how you will move the robot more generally by calculating thetaDeg1 and thetaDeg2 variables. 
   // Note those are your variable names ...   scaraIK would be called first to calculate thetaDeg1 and thetaDeg2
   thetaDeg1=-45.0; // define some angles. Arbitrary.  
   thetaDeg2=-155.0;
   sprintf(commandString, "ROTATE_JOINT ANG1 %.2lf ANG2 %.2lf\n", thetaDeg1, thetaDeg2);
   robot.Send(commandString);

   robot.Send("PEN_UP\n");
   robot.Send("ROTATE_JOINT ANG1 150.00 ANG2 90.00\n\n");
   robot.Send("PEN_DOWN\n");
   robot.Send(commandString);
   robot.Send("PEN_COLOR 10 64 109\n");
   robot.Send("ROTATE_JOINT ANG1 -150.00 ANG2 90.00\n");
   robot.Send("PROCESS_MESSAGE OFF\n");
   robot.Send("MESSAGE Erasing Traces\n");
   robot.Send("CLEAR_TRACE\n");

   robot.Send("HOME\n");

   printf("\n\nWhat do the following commands do?\n");
   getchar();

   robot.Send("CLEAR_REMOTE_COMMAND_LOG\n");
   robot.Send("CLEAR_POSITION_LOG\n");
   robot.Send("MESSAGE Bye-Bye\n");
   robot.Send("SHUTDOWN_SIMULATION\n");
   robot.Send("END\n");

   printf("\n\nPress ENTER to end the program...\n");
   getchar();

   
   robot.Close(); // close remote connection
   return 0;
}


//---------------------------------------------------------------------------------------
// Returns angle in radians from input angle in degrees.  Robot only accepts degrees!!
//double DegToRad(double angDeg)
//{
   //write code here.  One line only
//}

//---------------------------------------------------------------------------------------
// Returns angle in radians from input angle in degrees
//double RadToDeg(double angRad)
//{
  //write code here.  One line only
//}