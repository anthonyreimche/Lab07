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
# Author: <n>
# Date Created: <Date>
# Last Modified: <Today>
# -----------------------------------------------------------------------------*/
#pragma warning(disable:4996)  // get rid of some microsoft-specific warnings.

/*|Includes|-------------------------------------------------------------------*/
#include <stdio.h>  // <list of functions used>
#include <math.h>   // <list of functions used>
#include "robot.h"  // <list of functions used> // NOTE: DO NOT REMOVE.
#include <windows.h> // For console colors
#include <string>   // For string operations
#include <iostream> // For improved input/output

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

// Console color definitions
#define COLOR_DEFAULT         7  // White (default)
#define COLOR_TITLE           11 // Cyan
#define COLOR_PROMPT          14 // Yellow
#define COLOR_SUCCESS         10 // Green
#define COLOR_ERROR           12 // Red
#define COLOR_INFO            15 // Bright White
#define COLOR_HIGHLIGHT       13 // Magenta
#define COLOR_INPUT           15 // Bright White
#define COLOR_BCIT_BLUE       9  // Blue (approximation of BCIT Blue)

/*|Globals|--------------------------------------------------------------------*/
CRobot robot;
bool ArmType = LEFT_ARM_SOLUTION;
char commandString[MAX_STRING];
HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); // Handle to console for color manipulation

/*|Function Prototypes|--------------------------------------------------------*/
int scaraFK (double, double, double*, double*);
int scaraIK (double, double, double*, double*, int);
void moveScaraIK(void);
void moveScaraFK(void);

// UI Helper functions
void setConsoleColor(int color);
void printTitle(const char* title);
void printSubtitle(const char* subtitle);
void printPrompt(const char* prompt);
void printError(const char* error);
void printSuccess(const char* message);
void printInfo(const char* info);
void printDivider();
void printCoordinates(double x, double y);
void printAngles(double j1, double j2);
void clearScreen();
void promptPen();
void displayWelcomeScreen();
void displayArmConfigurations(double j1, double j2);

int main(){
   // Set console title
   SetConsoleTitle("SCARA Robot Simulator");
   
   // Configure console for Unicode display
   SetConsoleOutputCP(CP_UTF8);
   SetConsoleCP(CP_UTF8);
   
   // Set console font to a TrueType font that supports Unicode
   CONSOLE_FONT_INFOEX cfi;
   cfi.cbSize = sizeof(cfi);
   cfi.nFont = 0;
   cfi.dwFontSize.X = 0;
   cfi.dwFontSize.Y = 16; // Font size
   cfi.FontFamily = FF_DONTCARE;
   cfi.FontWeight = FW_NORMAL;
   wcscpy(cfi.FaceName, L"Consolas"); // A good font for console that supports Unicode
   SetCurrentConsoleFontEx(hConsole, FALSE, &cfi);
   
   // Clear screen and display welcome
   clearScreen();
   displayWelcomeScreen();

   // Open a connection with the simulator
   printPrompt("Connecting to simulator...");
   if(!robot.Initialize()) {
      printError("Failed to connect to simulator!");
      printf("\n\nPress ENTER to end the program...\n");
      getchar();
      return 0;
   }
   printSuccess("Connected to simulator successfully!");
   
   // Main program loop
   while (1) {
      printDivider();
      printTitle("SCARA ROBOT CONTROL INTERFACE");
      printDivider();
      
      // Menu options
      setConsoleColor(COLOR_INFO);
      printf("\n  [1] Forward Kinematics (Angles to Coordinates)");
      printf("\n  [2] Inverse Kinematics (Coordinates to Angles)");
      printf("\n  [3] Clear Trace");
      printf("\n  [4] Home Position");
      printf("\n  [5] Exit");
      printf("\n\n");
      
      // Get user choice
      int choice = 0;
      printPrompt("Enter your choice (1-5): ");
      setConsoleColor(COLOR_INPUT);
      scanf("%d", &choice);
      getchar(); // Clear input buffer
      
      switch(choice) {
         case 1:
            moveScaraFK();
            break;
         case 2:
            moveScaraIK();
            break;
         case 3:
            printInfo("Clearing trace...");
            robot.Send("CLEAR_TRACE\n");
            printSuccess("Trace cleared!");
            break;
         case 4:
            printInfo("Moving to home position...");
            robot.Send("HOME\n");
            printSuccess("Robot is at home position!");
            break;
         case 5:
            printInfo("Shutting down...");
            robot.Send("END\n");
            robot.Close();
            printSuccess("Goodbye!");
            return 0;
         default:
            printError("Invalid choice! Please enter a number between 1 and 5.");
            break;
      }
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
   printPrompt("Draw line? (Y/N): ");
   setConsoleColor(COLOR_INPUT);
   scanf("%c", &pen);
   getchar();
   robot.Send(pen=='y' || pen == 'Y' ? "PEN_DOWN\n" : "PEN_UP\n");

   if(pen=='y' || pen == 'Y') {
      printSuccess("Pen down - drawing enabled");
   } else {
      printInfo("Pen up - drawing disabled");
   }
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

   *_j1 = beta + (arm  == RIGHT_ARM_SOLUTION ? alpha : -alpha);
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

void moveScaraIK(void) {
   char pose;
   double J1 = 0, J2 = 0, X = 0, Y = 0;
   double J1_left = 0, J2_left = 0, J1_right = 0, J2_right = 0;
   int error_left = 0, error_right = 0;

   // Display IK mode header
   printDivider();
   printSubtitle("INVERSE KINEMATICS MODE");
   printInfo("Convert end-effector coordinates to joint angles");
   printDivider();

   //Prompt for coordinates
   do {
      error_left = 0;
      error_right = 0;
      printPrompt("Input a set of coordinates (X, Y): ");
      setConsoleColor(COLOR_INPUT);
      scanf("%lf, %lf", &X, &Y);
      getchar();

      // Try both arm configurations
      error_left = scaraIK(X, Y, &J1_left, &J2_left, LEFT_ARM_SOLUTION);
      error_right = scaraIK(X, Y, &J1_right, &J2_right, RIGHT_ARM_SOLUTION);

      // Check if either configuration is valid
      if (error_left && error_right) {
         printError("Coordinates are out of reach for both arm configurations!");
         setConsoleColor(COLOR_INFO);
         printf("  ► Target coordinates: (%.2lf, %.2lf)\n", X, Y);
         printf("  ► Max range: %.2lf mm\n", L1+L2);
         printf("  ► Min range: %.2lf mm\n", L1-L2);
         continue;
      }

      // If both configurations are valid, prompt user to choose
      if (!error_left && !error_right) {
         bool valid_choice = false;
         do {
            printSuccess("Both arm configurations are possible:");
            setConsoleColor(COLOR_INFO);
            printf("\n  ┌────────────────────────────────────────────┐\n");
            printf("  │ [L] LEFT ARM:  J1 = %+6.2f°, J2 = %+6.2f° │\n", J1_left, J2_left);
            printf("  │ [R] RIGHT ARM: J1 = %+6.2f°, J2 = %+6.2f° │\n", J1_right, J2_right);
            printf("  └────────────────────────────────────────────┘\n\n");

            printPrompt("Select arm pose (L/R): ");
            setConsoleColor(COLOR_INPUT);
            scanf_s("%c", &pose, 1);
            getchar();

            if (pose == 'L' || pose == 'l') {
               J1 = J1_left;
               J2 = J2_left;
               ArmType = LEFT_ARM_SOLUTION;
               valid_choice = true;
            } else if (pose == 'R' || pose == 'r') {
               J1 = J1_right;
               J2 = J2_right;
               ArmType = RIGHT_ARM_SOLUTION;
               valid_choice = true;
            } else {
               printError("Invalid choice. Please enter L or R.");
            }
         } while (!valid_choice);
      } else {
         // Only one configuration is valid
         if (!error_left) {
            J1 = J1_left;
            J2 = J2_left;
            ArmType = LEFT_ARM_SOLUTION;
            printInfo("Using LEFT arm configuration (only valid option)");
         } else {
            J1 = J1_right;
            J2 = J2_right;
            ArmType = RIGHT_ARM_SOLUTION;
            printInfo("Using RIGHT arm configuration (only valid option)");
         }
      }

      // Display the selected configuration
      printSuccess("Valid configuration selected!");
      setConsoleColor(COLOR_HIGHLIGHT);
      printf("  ► Arm configuration: %s\n", ArmType == LEFT_ARM_SOLUTION ? "LEFT" : "RIGHT");
      printCoordinates(X, Y);
      printAngles(J1, J2);

      // Break out of the loop since we have a valid configuration
      break;

   } while (1);

   // Ask about pen
   promptPen();

   // Send command to robot
   sprintf(&commandString[0], "ROTATE_JOINT ANG1 %.2lf ANG2 %.2lf\n", J1, J2);
   printInfo("Sending command to robot...");
   robot.Send(commandString);
   printSuccess("Robot moved successfully!");
   robot.Send("PEN_UP\n");
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

   // Display FK mode header
   printDivider();
   printSubtitle("FORWARD KINEMATICS MODE");
   printInfo("Convert joint angles to end-effector coordinates");
   printDivider();

   //Prompt for angles
   do {
      error = 0;
      printPrompt("Input 2 angles in degrees (J1, J2): ");
      setConsoleColor(COLOR_INPUT);
      scanf("%lf, %lf", &J1, &J2);
      getchar();

      error = scaraFK(J1, J2, &X, &Y);
      switch (error) {
         case (-1):
            printError("J1 is out of bounds!");
            setConsoleColor(COLOR_INFO);
            printf("Range for J1: ±%.2lf°\n", MAX_ABS_THETA1_DEG);
            continue;
         break;
         case (-2):
            printError("J2 is out of bounds!");
            setConsoleColor(COLOR_INFO);
            printf("Range for J2: ±%.2lf°\n", MAX_ABS_THETA2_DEG);
            continue;
         break;
      }
   } while (error);

   // Display the calculated angles and coordinates
   printSuccess("Valid joint angles entered!");
   displayArmConfigurations(J1, J2);
   printCoordinates(X, Y);

   // Ask about pen
   promptPen();

   // Send command to robot
   sprintf(&commandString[0], "ROTATE_JOINT ANG1 %.2lf ANG2 %.2lf\n", J1, J2);
   printInfo("Sending command to robot...");
   robot.Send(commandString);
   printSuccess("Robot moved successfully!");
   robot.Send("PEN_UP\n");
}

// UI Helper Functions Implementation

/**
 * @brief Set the console text color
 * @param color The color code to set
 */
void setConsoleColor(int color) {
   SetConsoleTextAttribute(hConsole, color);
}

/**
 * @brief Print a title with appropriate formatting
 * @param title The title text to print
 */
void printTitle(const char* title) {
   setConsoleColor(COLOR_TITLE);
   printf("\n\n  %s\n", title);
   setConsoleColor(COLOR_DEFAULT);
}

/**
 * @brief Print a subtitle with appropriate formatting
 * @param subtitle The subtitle text to print
 */
void printSubtitle(const char* subtitle) {
   setConsoleColor(COLOR_HIGHLIGHT);
   printf("\n  %s\n", subtitle);
   setConsoleColor(COLOR_DEFAULT);
}

/**
 * @brief Print a prompt with appropriate formatting
 * @param prompt The prompt text to print
 */
void printPrompt(const char* prompt) {
   setConsoleColor(COLOR_PROMPT);
   printf("\n  %s", prompt);
   setConsoleColor(COLOR_DEFAULT);
}

/**
 * @brief Print an error message with appropriate formatting
 * @param error The error text to print
 */
void printError(const char* error) {
   setConsoleColor(COLOR_ERROR);
   printf("\n  ✗ ERROR: %s\n", error);
   setConsoleColor(COLOR_DEFAULT);
}

/**
 * @brief Print a success message with appropriate formatting
 * @param message The success text to print
 */
void printSuccess(const char* message) {
   setConsoleColor(COLOR_SUCCESS);
   printf("\n  ✓ %s\n", message);
   setConsoleColor(COLOR_DEFAULT);
}

/**
 * @brief Print an info message with appropriate formatting
 * @param info The info text to print
 */
void printInfo(const char* info) {
   setConsoleColor(COLOR_INFO);
   printf("\n  ℹ %s\n", info);
   setConsoleColor(COLOR_DEFAULT);
}

/**
 * @brief Print a divider line
 */
void printDivider() {
   setConsoleColor(COLOR_DEFAULT);
   printf("\n  ────────────────────────────────────────────────────────────\n");
}

/**
 * @brief Print coordinates with appropriate formatting
 * @param x The x coordinate
 * @param y The y coordinate
 */
void printCoordinates(double x, double y) {
   setConsoleColor(COLOR_INFO);
   printf("  ► Coordinates: (%.2lf, %.2lf) mm\n", x, y);
   setConsoleColor(COLOR_DEFAULT);
}

/**
 * @brief Print angles with appropriate formatting
 * @param j1 The j1 angle
 * @param j2 The j2 angle
 */
void printAngles(double j1, double j2) {
   setConsoleColor(COLOR_INFO);
   printf("  ► Joint angles: J1=%.2lf°, J2=%.2lf°\n", j1, j2);
   setConsoleColor(COLOR_DEFAULT);
}

/**
 * @brief Clear the console screen
 */
void clearScreen() {
   system("cls");
}

/**
 * @brief Display a welcome screen
 */
void displayWelcomeScreen() {
   setConsoleColor(COLOR_BCIT_BLUE);
   printf("\n\n");
   printf("   ███████╗ ██████╗ █████╗ ██████╗  █████╗     ██████╗  ██████╗ ██████╗  ██████╗ ████████╗\n");
   printf("   ██╔════╝██╔════╝██╔══██╗██╔══██╗██╔══██╗    ██╔══██╗██╔═══██╗██╔══██╗██╔═══██╗╚══██╔══╝\n");
   printf("   ███████╗██║     ███████║██████╔╝███████║    ██████╔╝██║   ██║██████╔╝██║   ██║   ██║   \n");
   printf("   ╚════██║██║     ██╔══██║██╔══██╗██╔══██║    ██╔══██╗██║   ██║██╔══██╗██║   ██║   ██║   \n");
   printf("   ███████║╚██████╗██║  ██║██║  ██║██║  ██║    ██║  ██║╚██████╔╝██████╔╝╚██████╔╝   ██║   \n");
   printf("   ╚══════╝ ╚═════╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝    ╚═╝  ╚═╝ ╚═════╝ ╚═════╝  ╚═════╝    ╚═╝   \n");
   printf("                                                                                            \n");
   printf("                          ███████╗██╗███╗   ███╗██╗   ██╗██╗      █████╗ ████████╗ ██████╗ ██████╗ \n");
   printf("                          ██╔════╝██║████╗ ████║██║   ██║██║     ██╔══██╗╚══██╔══╝██╔═══██╗██╔══██╗\n");
   printf("                          ███████╗██║██╔████╔██║██║   ██║██║     ███████║   ██║   ██║   ██║██████╔╝\n");
   printf("                          ╚════██║██║██║╚██╔╝██║██║   ██║██║     ██╔══██║   ██║   ██║   ██║██╔══██╗\n");
   printf("                          ███████║██║██║ ╚═╝ ██║╚██████╔╝███████╗██║  ██║   ██║   ╚██████╔╝██║  ██║\n");
   printf("                          ╚══════╝╚═╝╚═╝     ╚═╝ ╚═════╝ ╚══════╝╚═╝  ╚═╝   ╚═╝    ╚═════╝ ╚═╝  ╚═╝\n");
   printf("\n\n");

   setConsoleColor(COLOR_INFO);
   printf("                                  ROBT 1270 - SCARA Simulator Basic Control\n");
   printf("                                  ----------------------------------------\n\n");

   setConsoleColor(COLOR_DEFAULT);
   printf("  This program demonstrates control over the SCARA Robot Simulator using forward and inverse kinematics.\n");
   printf("  The simulator allows controlling joint angles to move to desired (x, y) coordinates.\n\n");

   setConsoleColor(COLOR_INFO);
   printf("  ► Arm Length 1 (L1): %.1f mm\n", L1);
   printf("  ► Arm Length 2 (L2): %.1f mm\n", L2);
   printf("  ► Max J1 Angle: ±%.1f degrees\n", MAX_ABS_THETA1_DEG);
   printf("  ► Max J2 Angle: ±%.1f degrees\n\n", MAX_ABS_THETA2_DEG);
   
   setConsoleColor(COLOR_PROMPT);
   printf("  Press Enter to continue...");
   setConsoleColor(COLOR_DEFAULT);
   getchar();
   clearScreen();
}

/**
 * @brief Display both left and right arm configurations with proper alignment
 * @param j1 The j1 angle
 * @param j2 The j2 angle
 */
void displayArmConfigurations(double j1, double j2) {
   double j1_right = j1 + 90.0;
   double j2_right = -j2;
   
   setConsoleColor(COLOR_INFO);
   printf("  ┌────────────────────────────────────────────┐\n");
   printf("  │ [L] LEFT ARM:  J1 = %+6.2f°, J2 = %+6.2f° │\n", j1, j2);
   printf("  │ [R] RIGHT ARM: J1 = %+6.2f°, J2 = %+6.2f° │\n", j1_right, j2_right);
   printf("  └────────────────────────────────────────────┘\n");
   setConsoleColor(COLOR_DEFAULT);
}