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

/*|CONSTANTS|------------------------------------------------------------------*/
#define L1                    350.0
#define L2                    250.0
#define MAX_ABS_THETA1_DEG    150.0
#define MAX_ABS_THETA2_DEG    170.0
#define MAX_STRING            256
#define PI                    3.14159265358979323846
#define LEFT_ARM_SOLUTION     0
#define RIGHT_ARM_SOLUTION    1
#define ESC                   27

/*|Globals|--------------------------------------------------------------------*/
CRobot robot;
bool ArmType = LEFT_ARM_SOLUTION;
char commandString[MAX_STRING];

/*|Function Prototypes|--------------------------------------------------------*/
int scaraFK (double, double, double*, double*);
int scaraIK (double, double, double*, double*, int);
void moveScaraIK(void);
void moveScaraFK(void);

int main(){

   // Open a connection with the simulator
   if(!robot.Initialize()) return 0;

   while (1) {
      //moveScaraFK();
      moveScaraIK();
   }
   printf("\n\nPress ENTER to end the program...\n");
   getchar();
   robot.Close(); // close remote connection
   return 0;
}

/**
 * @brief This function will calculate the x,y coordinates given two joint angles.
 *
 * @param _j1 Angle of joint 1 in degrees.
 * @param _j2 Angle of joint 2 in degrees.
 * @param _x The tool position along the x-axis. Pointer
 * @param _y The tool position along the y-axis. Pointer
 *
 * @return inRange (0) in range, (-1) out of range
 */
int scaraFK (double _j1, double _j2, double* _x, double* _y) {
   if (abs(_j1) > MAX_ABS_THETA1_DEG) return -1;
   if (abs(_j2) > MAX_ABS_THETA2_DEG) return -2;

   _j1 *= PI/180.0;
   _j2 *= PI/180.0;
   *_x = L1*cos(_j1)+L2*cos(_j1+_j2);
   *_y = L1*sin(_j1)+L2*sin(_j1+_j2);

   return 0;
}

void promptPen() {
   char pen;
   printf("Draw line? (Y/N):\n");
   scanf("%c", &pen);
   getchar();
   robot.Send(pen=='y' || pen == 'Y' ? "PEN_DOWN\n" : "PEN_UP\n");
}

/**
* @brief Calculate two joint angles given the x,y coordinates.
*
* @param _x - The tool position along the x-axis.
* @param _y - The tool position along the y-axis.
* @param _j1 - Angle of joint 1 in degrees. Pointer
* @param _j2 - Angle of joint 2 in degrees. Pointer
* @param arm - Selects which solution to try.
*
* @return - (0) in range, (-1) out of range
*/
int scaraIK (double _x, double _y, double* _j1, double* _j2, int arm) {

   const double L = sqrt(_x*_x + _y*_y);
   const double Min = sqrt(((L1*L1) + (L2*L2)) - (2 * L1 * L2 * cos(0.174532925)));

   if ( L > L1 + L2 ) return -1;
   if ( L < Min ) return -2;

   // Calculate joint angles
   const double beta = atan2(_y,_x);
   const double alpha = acos(((L2*L2) - (L*L) - (L1*L1)) / (-2*L*L1));

   // Use a sign multiplier based on arm configuration
   double sign = (arm == LEFT_ARM_SOLUTION) ? -1.0 : 1.0;
   *_j1 = beta + (sign * alpha);

   //*_j1 = beta + (arm ? alpha : -alpha);
   *_j2 = atan2(_y - (L1 * sin(*_j1)), _x - (L1 * cos(*_j1))) - *_j1;
   
   // Convert to degrees
   *_j1 *= 180/PI;
   *_j2 *= 180/PI;

   if (*_j2 < -MAX_ABS_THETA2_DEG) *_j2 += 360;
   if (*_j2 > MAX_ABS_THETA2_DEG) *_j2 -= 360;
   if (*_j1 < -MAX_ABS_THETA1_DEG) *_j1 += 360;
   if (*_j1 > MAX_ABS_THETA1_DEG) *_j1 -= 360;

   if (fabs(*_j1) > MAX_ABS_THETA1_DEG) return -1;
   if (fabs(*_j2) > MAX_ABS_THETA2_DEG) return -1;

   return 0;
}

/**
 *@brief function will ask the user for SCARA joint variables in degrees. Then ask the user for the pen position and display the X,Y position.
*
* @param void
*
* @return void
*/

void moveScaraFK(void) {
   char pose;
   double J1, J2, X, Y;
   int error;
   //Prompt for angles
   do {
      error = 0;
      printf("\n\nInput 2 angles in degrees (J1, J2):\n");
      scanf("%lf, %lf", &J1, &J2);
      printf("%lf,%lf\n",J1,J2);
      getchar();

      error = scaraFK(J1,J2,&X,&Y);
      switch (error) {
         case (-1):
            printf("J1 is out of bounds!\n");
            printf("Range for J1: ±%.2lf°\n",MAX_ABS_THETA1_DEG);
            continue;
         break;
         case (-2):
            printf("J2 is out of bounds!\n");
            printf("Range for J2: ±%.2lf°\n",MAX_ABS_THETA2_DEG);
            continue;
         break;
      }
   } while (error);

   promptPen();

   printf("Calculated coordinates: %.2lf,%.2lf\n",X,Y);
   sprintf(&commandString[0], "ROTATE_JOINT ANG1 %.2lf ANG2 %.2lf\n", J1, J2);
   robot.Send(commandString);
   robot.Send("PEN_UP\n");
}

/**
* @brief This function will ask the user for SCARA joint variables in degrees. Then ask the user for the pen position and display the X,Y position.
* @param void
* @return void
 */
void moveScaraIK(void) {
   char pose;
   double J1, J2, X, Y;
   int error;
   int RL = 0;
   //Prompt for angles
   do {
      error = 0;
      printf("\n\nInput a set of coordinates (X, Y):\n");
      scanf("%lf, %lf", &X, &Y);
      getchar();

      error = scaraIK(X,Y,&J1,&J2, ArmType);
      if (error) {
         error = scaraIK(X,Y,&J1,&J2, -ArmType);
         /*RL = 1;
         if (!scaraIK(X,Y,&J1,&J2, -ArmType)) {
            ArmType = -ArmType;
            RL = 2;
            error = -1;
         }*/
      }

      switch (error) {
         case (-1):
            printf("coordinates exceed maximum range!\n");
            printf("Max range: %.2lf mm\nMin range: %.2lf mm",L1+L2,L1-L2);
         continue;
         break;
         case (-2):
            printf("coordinates exceed minimum range!\n");
            printf("Max range: %.2lf mm\nMin range: %.2lf mm",L1+L2,L1-L2);
         continue;
         break;
      }

      //Prompt for pose type
      bool _pose;
      if (RL ==0) {
         do {
            _pose = false;
            printf("Select arm pose (L/R): ");
            scanf_s("%c",&pose,1);
            getchar();
            if (pose == 'R' || pose == 'r') {
               ArmType = RIGHT_ARM_SOLUTION;
               scaraIK(X,Y,&J1,&J2, ArmType);
               _pose = true;
            }
            if (pose == 'L' || pose == 'l') {
               ArmType = LEFT_ARM_SOLUTION;
               scaraIK(X,Y,&J1,&J2, ArmType);
               _pose = true;
            }
         } while (!_pose);
      }
   } while (error);
   promptPen();
   printf("Calculated angles: %.2lf,%.2lf\n",J1,J2);
   sprintf(&commandString[0], "ROTATE_JOINT ANG1 %.2lf ANG2 %.2lf\n", J1, J2);
   robot.Send(commandString);
   robot.Send("PEN_UP\n");
}