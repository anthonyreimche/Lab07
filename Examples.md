```
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
robot.Send("END\n");```