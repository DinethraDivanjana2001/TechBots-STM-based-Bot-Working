/*
 * PICController.h
 *
 *  Created on: Mar 19, 2024
 *      Author: yasir
 */

#ifndef INC_PIDCONTROLLER_H_
#define INC_PIDCONTROLLER_H_

#define IR_ARRAY_LENGTH 25

struct DriveValues {
    int LEFT;
    int RIGHT;
};

struct DriveValues PID_control(double Kp, double Kd, double Ki, int digital_IR[IR_ARRAY_LENGTH]);


#endif /* INC_PIDCONTROLLER_H_ */
