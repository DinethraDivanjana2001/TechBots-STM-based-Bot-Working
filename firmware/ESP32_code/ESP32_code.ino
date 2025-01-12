#include <WiFi.h>

///////////////////////////////////////////////////////////////////////

#define INA_LEFT 37
#define INB_LEFT 36
#define PWM_LEFT 35

#define INA_RIGHT 40
#define INB_RIGHT 39
#define PWM_RIGHT 38

#define ENC_LA 6
#define ENC_LB 5

#define ENC_RA 15
#define ENC_RB 7

#define LEFT_MOTOR 1
#define RIGHT_MOTOR -1

#define FORWARD 0
#define BACKWARD 1

#define IR_ARRAY_LENGTH 23

#define mux_L_sig 16
#define mux_L_s3 17
#define mux_L_s2 18
#define mux_L_s1 21
#define mux_L_s0 47

#define mux_R_sig 19
#define mux_R_s3 20
#define mux_R_s2 10
#define mux_R_s1 11
#define mux_R_s0 12

#define posMaxSpeed 400

#define left 0
#define right 1
///////////////////////////////////////////////////////////////////////

// PID parametaers for Line Following
bool mode = 0; // 0 - line , 1 - bend

int endOfCircle = 2;
double Kp = 1.25;
double Ki = 0.0001;
double Kd = 0.18;
double now_time = 0; 

float left_sum;
float right_sum;
float right_sensors;
float left_sensors;

int Drive_constant = 400;

int prev_position = 0;
int prev_history_sum = 0;

// Initialize PID variables
double prevError = 0;
double integral = 0;
double derivative = 0;

const int leftMotorChannel = 0;
const int rightMotorChannel = 1;
const int frequency = 10000;
const int resolution = 10;

int IR_array[IR_ARRAY_LENGTH];
int Ir_thresholds[] = { 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000 };

int Left_drive = 0;
int Right_drive = 0;

volatile int leftCount = 0;
volatile int rightCount = 0;

int setpoint_L = 312;
int setpoint_R = -312;

// PID parameters for Line Following
double Kp_L = 4;
double Ki_L = 0.000;
double Kd_L = 0.00005;

// PID parameters for Line Following
double Kp_R = 4;
double Ki_R = 0.000;
double Kd_R = 0.00005;

// Initialize PID variables
double prevError_L = 0;
double integral_L = 0;
double derivative_L = 0;

// Initialize PID variables
double prevError_R = 0;
double integral_R = 0;
double derivative_R = 0;

uint32_t nowTime;
bool bendDirection;
int level = 0;
int maxMotorSpeed = 1000;
///////////////////////////////////////////////////////////////////////

void readIRArray();
int readRightMux(int channel);
int readLeftMux(int channel);
void speed(int leftSpeed, int rightSpeed);
void motor(int motor, int direction);

void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(ENC_LB) == HIGH) {
    leftCount++;
  } else {
    leftCount--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(ENC_RB) == HIGH) {
    rightCount++;
  } else {
    rightCount--;
  }
}

///////////////////////////////////////////////////////////////////////

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  pinMode(INA_LEFT, OUTPUT);
  pinMode(INB_LEFT, OUTPUT);
  pinMode(INA_RIGHT, OUTPUT);
  pinMode(INB_RIGHT, OUTPUT);

  ledcSetup(leftMotorChannel, frequency, resolution);
  ledcAttachPin(PWM_LEFT, leftMotorChannel);

  ledcSetup(rightMotorChannel, frequency, resolution);
  ledcAttachPin(PWM_RIGHT, rightMotorChannel);

  pinMode(mux_R_s3, OUTPUT);
  pinMode(mux_R_s2, OUTPUT);
  pinMode(mux_R_s1, OUTPUT);
  pinMode(mux_R_s0, OUTPUT);

  pinMode(mux_L_s3, OUTPUT);
  pinMode(mux_L_s2, OUTPUT);
  pinMode(mux_L_s1, OUTPUT);
  pinMode(mux_L_s0, OUTPUT);

  pinMode(ENC_LA, INPUT);
  pinMode(ENC_LB, INPUT);
  pinMode(ENC_RA, INPUT);
  pinMode(ENC_RB, INPUT);
  
  attachInterrupt(ENC_LA, leftEncoderISR, RISING);
  attachInterrupt(ENC_RA, rightEncoderISR, RISING);
  // level = 2; // Remove this. Only for testing purposes

  // calibrate();
}

///////////////////////////////////////////////////////////////////////

void readIRArray() {
  // Loop through channels 0 to 15
  for (int channel = 0; channel < 16; channel++) {
    // int rightValue = readRightMux(channel);
    // Serial.print(" ");
    // Serial.print(rightValue);
    IR_array[channel] = readRightMux(channel) < Ir_thresholds[channel];
  }

  for (int channel = 0; channel <= 7; channel++) {
    // int leftValue = readLeftMux(channel);
    // Serial.print(" ");
    // Serial.print(leftValue);
    IR_array[channel + 16] = readLeftMux(channel) < Ir_thresholds[channel];
  }

  // Serial.println();
}

///////////////////////////////////////////////////////////////////////

// Function to read analog value from left mux
int readLeftMux(int channel) {
  digitalWrite(mux_L_s0, bitRead(channel, 0));
  digitalWrite(mux_L_s1, bitRead(channel, 1));
  digitalWrite(mux_L_s2, bitRead(channel, 2));
  digitalWrite(mux_L_s3, bitRead(channel, 3));

  return analogRead(mux_L_sig);
}

///////////////////////////////////////////////////////////////////////

// Function to read analog value from right mux
int readRightMux(int channel) {
  digitalWrite(mux_R_s0, bitRead(channel, 0));
  digitalWrite(mux_R_s1, bitRead(channel, 1));
  digitalWrite(mux_R_s2, bitRead(channel, 2));
  digitalWrite(mux_R_s3, bitRead(channel, 3));

  return analogRead(mux_R_sig);
}

///////////////////////////////////////////////////////////////////////

void speed(int leftSpeed, int rightSpeed) {
  // Set speed for left motor
  ledcWrite(leftMotorChannel, leftSpeed);

  // Set speed for right motor
  ledcWrite(rightMotorChannel, rightSpeed);
}

///////////////////////////////////////////////////////////////////////

void motor(int motor, int direction) {
  if (motor == LEFT_MOTOR) {
    if (direction == FORWARD) {
      digitalWrite(INA_LEFT, HIGH);
      digitalWrite(INB_LEFT, LOW);
    } else if (direction == BACKWARD) {
      digitalWrite(INA_LEFT, LOW);
      digitalWrite(INB_LEFT, HIGH);
    }
  } else if (motor == RIGHT_MOTOR) {
    if (direction == FORWARD) {
      digitalWrite(INA_RIGHT, HIGH);
      digitalWrite(INB_RIGHT, LOW);
    } else if (direction == BACKWARD) {
      digitalWrite(INA_RIGHT, LOW);
      digitalWrite(INB_RIGHT, HIGH);
    }
  }
}

void applyBrakes(int motor) {
  if (motor == LEFT_MOTOR) {
    digitalWrite(INA_LEFT, HIGH); // Set one terminal high
    digitalWrite(INB_LEFT, HIGH); // Set the other terminal high
  } else if (motor == RIGHT_MOTOR) {
    digitalWrite(INA_RIGHT, HIGH); // Set one terminal high
    digitalWrite(INB_RIGHT, HIGH); // Set the other terminal high
  }
}

double Getabs(double num){
  if (num > 0){
    return num;
  } else{
    return -num;
  }
}

////////////////////////////////////takeBend////////////////////////////
void takeBend(bool direction){
  if(millis()< 500){
    return;
  }
  applyBrakes(RIGHT_MOTOR);
  applyBrakes(LEFT_MOTOR);
  delay(200);
  motor(LEFT_MOTOR,FORWARD);
  motor(RIGHT_MOTOR,FORWARD);
  if(level == endOfCircle){
    speed(500,800);
    level++;  
  }
  else{
    speed(500,500);
  }
  delay(200);

  // Serial.print(leftCount);
  // Serial.print(" : ");
  // Serial.print(rightCount);
  // Serial.print(" : ");
  if (direction){
    setpoint_L = 320;
    setpoint_R = -320;
  }
  else{
    setpoint_L = -320;
    setpoint_R = 320;
  }
  leftCount = 0;
  rightCount = 0;


  double nowBendTime = millis();
  while(millis() - nowBendTime < 700){

  int leftPos = setpoint_L - leftCount;

  derivative_L = leftPos - prevError_L;
  // added by nirosh
  integral_L = integral_L + leftPos;
  //
  double PID_constant_L = Kp_L * leftPos + Ki_L * integral_L + Kd_L * derivative_L;

  // Serial.print(" pid integral l : ");
  // Serial.print(integral_L);
  // Serial.print(" : ");

  prevError_L = leftPos;

  if (PID_constant_L > posMaxSpeed) PID_constant_L = posMaxSpeed;
  if (PID_constant_L < -posMaxSpeed) PID_constant_L = -posMaxSpeed;

  if (integral_L > 400/Ki_L) integral_L = 400/Ki_L;
  if (integral_L < -400/Ki_L) integral_L = -400/Ki_L;


  // Serial.print(PID_constant_L);
  // Serial.print(" : ");

  ///////////////////////////////////////////////////////////

  int rightPos = setpoint_R - rightCount;

  derivative_R = rightPos - prevError_R;
  // aded by nirosh
  integral_R = integral_R + rightPos;
  //
  double PID_constant_R = Kp_R * rightPos + Ki_R * integral_R + Kd_R * derivative_R;

  prevError_R = rightPos;

  if (PID_constant_R > posMaxSpeed) PID_constant_R = posMaxSpeed;
  if (PID_constant_R < -posMaxSpeed) PID_constant_R = -posMaxSpeed;

  if (integral_R > 400/Ki_R) integral_R = 400/Ki_R;
  if (integral_R < -400/Ki_R) integral_R = -400/Ki_R;

  // Serial.println(PID_constant_R);

  if (PID_constant_L > 0){
    motor(LEFT_MOTOR, FORWARD);      // Set left motor direction to backward
  }else{
    motor(LEFT_MOTOR, BACKWARD);      // Set left motor direction to backward
  }

  if (PID_constant_R > 0){
    motor(RIGHT_MOTOR, FORWARD);      // Set left motor direction to backward
  }else{
    motor(RIGHT_MOTOR, BACKWARD);      // Set left motor direction to backward
  }

  // applyBrakes(RIGHT_MOTOR);

  // Serial.println(Getabs(PID_constant_L));
  speed(Getabs(PID_constant_L),Getabs(PID_constant_R));
  }
  applyBrakes(RIGHT_MOTOR);
  applyBrakes(LEFT_MOTOR);
  delay(100);
  leftCount = 0;
  rightCount = 0;
  prevError_L = 0;
  integral_L = 0;
  derivative_L = 0;
  prevError_R = 0;
  integral_R = 0;
  derivative_R = 0;
  prevError = 0;
  integral = 0;
  derivative = 0;
}
void lineFollow(){
  readIRArray();

  //////////decisions on jections and bends////////////
  if (left_sensors + right_sensors > 16){
    // level++;
    applyBrakes(RIGHT_MOTOR);
    applyBrakes(LEFT_MOTOR);
    delay(250);
    speed(500,500);
    motor(LEFT_MOTOR, FORWARD);
    motor(RIGHT_MOTOR, FORWARD);
    delay(250);
  }
  else if (IR_array[0] && (IR_array[1] + IR_array[2] + IR_array[3] + IR_array[4] + IR_array[5] + IR_array[6] + IR_array[7]) >= 5 && (IR_array[21] + IR_array[20] + IR_array[19] + IR_array[18] + IR_array[17]) < 5){
    // Serial.println("Left");
    takeBend(left);
  }
  else if (IR_array[22] && (IR_array[21] + IR_array[20] + IR_array[19] + IR_array[18] + IR_array[17] + IR_array[16] + IR_array[15]) >= 5 && (IR_array[0] + IR_array[1] + IR_array[2] + IR_array[3] + IR_array[4]) < 5){
    // Serial.println("Right");
    takeBend(right);
  }

  int array_lit_amount = 0;
  for (int i = 0; i < IR_ARRAY_LENGTH; i++) {
    array_lit_amount += IR_array[i];
  }

  // The following two lines should change with respect to IR array length.

  // // For 8 IR array.
  left_sensors = IR_array[0] + IR_array[1] + IR_array[2] + IR_array[3] + IR_array[4] + IR_array[5] + IR_array[6] + IR_array[7] + IR_array[8] + IR_array[9] + IR_array[10];
  right_sensors = IR_array[12] + IR_array[13] + IR_array[14] + IR_array[15] + IR_array[16] + IR_array[17] + IR_array[18] + IR_array[19] + IR_array[20] + IR_array[21] + IR_array[22];

  left_sum = -54 * IR_array[0] - 39 * IR_array[1] - 36 * IR_array[2] - 33 * IR_array[3] - 29 * IR_array[4] - 24 * IR_array[5] - 20 * IR_array[6] - 15 * IR_array[7] - 12 * IR_array[8] - 5.4 * IR_array[9] - 2.7 * IR_array[10];
  right_sum = 2.7 * IR_array[12] + 5.4 * IR_array[13] + 12 * IR_array[14] + 15 * IR_array[15] + 20 * IR_array[16] + 24 * IR_array[17] + 29 * IR_array[18] + 33 * IR_array[19] + 36 * IR_array[20] + 39 * IR_array[21] + 54 * IR_array[22];
  //Serial.println(left_sum + right_sum);

  int position = left_sum + right_sum;

  derivative = position - prevError;
  // aded by nirosh
  integral = integral + position;
  //
  int PID_constant = Kp * position + Ki * integral + Kd * derivative;

  prevError = position;

  int offset = 0;  // For correcting motor speeds
  Left_drive = Drive_constant + offset - PID_constant;
  Right_drive = Drive_constant - offset + PID_constant;

  // Limiting to 0 - 255 range
  Left_drive = min(max(Left_drive, 0), maxMotorSpeed);
  Right_drive = min(max(Right_drive, 0), maxMotorSpeed);

  prev_position = position;

  speed(Right_drive, Left_drive);  // Set right motor speed to 150
  motor(RIGHT_MOTOR, FORWARD);     // Set right motor direction to forward
  motor(LEFT_MOTOR, FORWARD);      // Set left motor direction to backward

  // for (int i = 0; i < 22; i++) {
  //   Serial.print(IR_array[i]);
  //   Serial.print(" : ");
  // }

  // Serial.print(Left_drive);
  // Serial.print(" : ");
  // Serial.print(Right_drive);
  // Serial.print(" : ");
  // Serial.println();

}

void calibrate() {
  motor(RIGHT_MOTOR, BACKWARD);
  motor(LEFT_MOTOR, FORWARD);

  speed(500,500);

  // make sensor_max_values array of length 8 equal to sensor calibration array
  int sensor_max_values[IR_ARRAY_LENGTH] = { 0 };

  // sensor min values array
  int sensor_min_values[IR_ARRAY_LENGTH] = { 0 };

  int now_time_for_calibration = millis();
  // Serial.print("now time for calibration: ");
  // Serial.print(now_time_for_calibration);
  // Serial.print("  millis: ");
  // Serial.println(millis());

  while (millis() - now_time_for_calibration < 2500) {
    // Serial.println("Caliberating IR");

    bool allZeros = true;
    for (int i = 0; i < IR_ARRAY_LENGTH; i++) {
      if (sensor_max_values[i] != 0) {
        allZeros = false;
        break;  // No need to continue checking if we find a non-zero value
      }
    }

    if (allZeros) {
      for (int i = 0; i < IR_ARRAY_LENGTH; i++) {
        sensor_max_values[i] = readRightMux(i);
        sensor_min_values[i] = readRightMux(i);
      }
    }

    for (int i = 0; i < IR_ARRAY_LENGTH; i++) {
      if (readRightMux(i + IR_ARRAY_LENGTH) > sensor_max_values[i]) {
        sensor_max_values[i] = readRightMux(i);
      }
      if (readRightMux(i + IR_ARRAY_LENGTH) < sensor_min_values[i]) {
        sensor_min_values[i] = readRightMux(i);
      }
    }
  }

  for (int i = 0; i < IR_ARRAY_LENGTH; i++) {
    Ir_thresholds[i] = (sensor_max_values[i] + sensor_min_values[i]) / 2;
  }

  delay(1000);

  applyBrakes(RIGHT_MOTOR);
  applyBrakes(LEFT_MOTOR);

  delay(2000);
}
//////////////////////////////////LOOP/////////////////////////////////////

void loop() {


  nowTime = micros();
  //////////////////////////////////////////////////////////////////////////////////////

    lineFollow();

  
  //////////////////////////////////////////////////////////////////////////////////////
  while(true){
    if(micros() - nowTime > 2000) break;  
  }
  
}
