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
int ARRAY_WEIGHT[25] = {
		-4400,-4000,-3600,-3200,-2980,-1100,-400,-150,-54,-20,-8,-3,0,3,8,20,54,150,400,1100,2980,3200,3600,4000,4400

};//Y8 IR25

float test = 0;
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
//	  for (int i=-IR_MID_VAL; i<0;i++) left_sum += ARRAY_WEIGHT[i+IR_MID_VAL]*digital_IR[i+IR_MID_VAL];
	  left_sum = ARRAY_WEIGHT[0]*digital_IR[0]+ARRAY_WEIGHT[1]*digital_IR[1]+ARRAY_WEIGHT[2]*digital_IR[2]+
			  ARRAY_WEIGHT[3]*digital_IR[3]+ARRAY_WEIGHT[4]*digital_IR[4]+ARRAY_WEIGHT[5]*digital_IR[5]+
			  ARRAY_WEIGHT[6]*digital_IR[6]+ARRAY_WEIGHT[7]*digital_IR[7]+ARRAY_WEIGHT[8]*digital_IR[8]+
			  ARRAY_WEIGHT[9]*digital_IR[9]+ARRAY_WEIGHT[10]*digital_IR[10]+ARRAY_WEIGHT[11]*digital_IR[11];
test = left_sum;
	  //Calculate the sum of Right side IRs with given weight
	  float right_sum = 0;

	  right_sum = ARRAY_WEIGHT[13]*digital_IR[13]+ARRAY_WEIGHT[14]*digital_IR[14]+
			  ARRAY_WEIGHT[15]*digital_IR[15]+ARRAY_WEIGHT[16]*digital_IR[16]+ARRAY_WEIGHT[17]*digital_IR[17]+
			  ARRAY_WEIGHT[18]*digital_IR[18]+ARRAY_WEIGHT[19]*digital_IR[19]+ARRAY_WEIGHT[20]*digital_IR[20]+
			  ARRAY_WEIGHT[21]*digital_IR[21]+ARRAY_WEIGHT[22]*digital_IR[22]+ARRAY_WEIGHT[23]*digital_IR[23]+
			  ARRAY_WEIGHT[24]*digital_IR[24];
//	  for (int i=1; i<IR_MID_VAL;i++) right_sum += ARRAY_WEIGHT[i]*digital_IR[i+IR_MID_VAL];

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

