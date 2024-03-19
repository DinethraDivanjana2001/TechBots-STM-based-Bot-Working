/*
 * PIDController.c
 *
 *  Created on: Mar 19, 2024
 *      Author: yasir
 */


#include "PIDController.h"

// Private variables for PID control
static double prevError = 0;
static int Drive_constant = 500;

// Function to calculate PID control values
struct DriveValues PID_control(double Kp, double Kd, double Ki, int digital_IR[IR_ARRAY_LENGTH]) {
	//Calculate the amount of IRs turned on
	  int array_lit_amount = 0;
	  for (int i = 0; i < IR_ARRAY_LENGTH; i++)
	  {
		  array_lit_amount += digital_IR[i];
	  }

	  int IR_MID_VAL = ((IR_ARRAY_LENGTH+1)/2-1);

	  //Calculate the sum of Left side IRs with given weight
	  float left_sum = 0;
	  for (int i=-IR_MID_VAL; i<0;i++) left_sum += i*digital_IR[i+IR_MID_VAL];

	  //Calculate the sum of Right side IRs with given weight
	  float right_sum = 0;
	  for (int i=1; i<IR_MID_VAL;i++) right_sum += i*digital_IR[i+IR_MID_VAL];

	  //Calculate the current Error
	  int position = left_sum + right_sum;

	  int derivative = position - prevError;

	  int PID_constant = Kp * position + Kd * derivative;

	  prevError = position;

	  Drive_constant = 500;

	  int offset = 0; // For correcting motor speeds
	  int Left_drive = Drive_constant + offset - PID_constant;
	  int Right_drive = Drive_constant - offset + PID_constant;

	  const int MIN_VALUE = -1024;
	  const int MAX_VALUE = 1024;

	  if (Left_drive < MIN_VALUE) {
		  Left_drive = MIN_VALUE;
	  } else if (Left_drive > MAX_VALUE) {
		  Left_drive = MAX_VALUE;
	  }

	  if (Right_drive < MIN_VALUE) {
		  Right_drive = MIN_VALUE;
	  } else if (Right_drive > MAX_VALUE) {
		  Right_drive = MAX_VALUE;
	  }

	  // Example return statement, replace with actual calculated values
	  struct DriveValues result;
	  result.LEFT = Left_drive; // Example value
	  result.RIGHT = Right_drive; // Example value
	  return result;

}

// Supporting functions if any

