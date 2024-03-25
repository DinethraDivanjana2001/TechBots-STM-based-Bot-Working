#include <EEPROM.h>
#include "Adafruit_VL53L0X.h"

// ========== LINE_COLOR DEFINITION ==========
const String LINE_COLOR = "WHITE";

int LEVEL = 1;

// Define the size of the array and the EEPROM address to start writing data
const int arraySize = 8;      // Adjust this based on your array size
const int eepromAddress = 0;  // EEPROM address where you want to start writing

const int inputCalib = 7;  // Pin 7 is connected to your input device

float cylinder_variance = 0;

int col = 1;

// PID parameters for Line Following
double Kp = 5;
double Ki = 0;
double Kd = 0.1;

// PID parameters for Line Following
double Kp_circle = 3;
double Ki_circle = 0;
double Kd_circle = 0.3
;

double now_time = 0; 

float left_sum;
float right_sum;
float right_sensors;
float left_sensors;

int Drive_constant = 100;

// Initialize PID variables
double prevError = 0;
double integral = 0;
double derivative = 0;

int Ir_thresholds[] = { 300, 300, 100, 100, 100, 105, 400, 400 };

// define right motor enable pin is 3 and left 2
#define GREEN_LED 6
#define BLUE_LED 5

#define ENR 3
#define ENL 2

#define Motor_Right_Forward 26
#define Motor_Right_Backward 28
#define Motor_Left_Forward 22
#define Motor_Left_Backward 24

// Motor and Direction Definitions
#define RIGHT_MOTOR 1
#define LEFT_MOTOR 2
#define FORWARD 1
#define BACKWARD 2

#define magnet 9

#define Buzzer 11

// IR_array 8 space empty array
#define IR_ARRAY_LENGTH 12
int IR_array[IR_ARRAY_LENGTH];
int calibrate_array_size = IR_ARRAY_LENGTH - 4;

int Threshold = 100;

int prev_position = 0;
int prev_history_sum = 0;


long count = 0;      // Number of observations
float mean = 0;      // Running mean
float M2 = 0;        // Sum of squares of differences from the current mean
float variance = 0;  // Variance

// address we will assign if dual sensor is present
#define LOX1_ADDRESS 0x33
#define LOX2_ADDRESS 0x30

// set the pins to shutdown
#define SHT_LOX1 10
#define SHT_LOX2 39

// objects for the vl53l0x
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

// this holds the measurement
VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;
uint16_t side_tof;
uint16_t front_tof;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  pinMode(Motor_Right_Forward, OUTPUT);
  pinMode(Motor_Right_Backward, OUTPUT);
  pinMode(Motor_Left_Forward, OUTPUT);
  pinMode(Motor_Left_Backward, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A2, INPUT);
  pinMode(A1, INPUT);
  pinMode(A3, INPUT);

  pinMode(inputCalib, INPUT);

  pinMode(magnet, OUTPUT);
  pinMode(Buzzer, OUTPUT);

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  

  Serial.println(F("Shutdown pins inited..."));

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println(F("Both in reset mode...(pins are low)"));
  setID();

  for (int i = 0; i < calibrate_array_size; i++) {
    // Ir_thresholds[i] = EEPROM.read(i)*4;
    EEPROM.get(i * sizeof(int), Ir_thresholds[i]);
  }

  Serial.print("int IR_thresholds[] = {");
  for (int i = 0; i < calibrate_array_size; i++) {
    Serial.print(Ir_thresholds[i]);
    Serial.print(", ");
  }
  Serial.println("};    ");

  calibrate();

  trigMagnet(false);

  beep(5);

  while (true) {
    speed(RIGHT_MOTOR, 100);      // Set right motor speed to 150
    motor(RIGHT_MOTOR, FORWARD);  // Set right motor direction to forward

    speed(LEFT_MOTOR, 100);      // Set left motor speed to 200
    motor(LEFT_MOTOR, FORWARD);  // Set left motor direction to backward

    readIRArray();
    if (sensorSum() < 7) {
      Serial.println("ENTERED PID");
      delay(500);
      break;
    }
  }
}

int temp = 0;
int Left_drive = 0;
int Right_drive = 0;

bool task1_color = false;

void loop() {

  //  if(digitalRead(inputCalib) == 1) calibrate();

  if (LEVEL == 1) TASK1();
  if (LEVEL == 2) TASK2();

  if (LEVEL == 3) TASK3();
  if (LEVEL == 4) TASK4();
  if (LEVEL == 5) TASK5();

  // speed(RIGHT_MOTOR, 100); // Set right motor speed to 150
  // motor(RIGHT_MOTOR, FORWARD); // Set right motor direction to forward

  // speed(LEFT_MOTOR, 100); // Set left motor speed to 200
  // motor(LEFT_MOTOR, FORWARD); // Set left motor direction to backward
  // delay(1000);

  // trigMagnet(true);
  // speed(RIGHT_MOTOR, 00); // Set right motor speed to 150
  // motor(RIGHT_MOTOR, FORWARD); // Set right motor direction to forward

  // speed(LEFT_MOTOR, 000); // Set left motor speed to 200
  // motor(LEFT_MOTOR, FORWARD); // Set left motor direction to backward
  // delay(1000);

  // speed(RIGHT_MOTOR, 100); // Set right motor speed to 150
  // motor(RIGHT_MOTOR, BACKWARD); // Set right motor direction to forward

  // speed(LEFT_MOTOR, 100); // Set left motor speed to 200
  // motor(LEFT_MOTOR, BACKWARD); // Set left motor direction to backward
  // delay(2000);

  // trigMagnet(false);
  // speed(RIGHT_MOTOR, 00); // Set right motor speed to 150
  // motor(RIGHT_MOTOR, FORWARD); // Set right motor direction to forward

  // speed(LEFT_MOTOR, 000); // Set left motor speed to 200
  // motor(LEFT_MOTOR, FORWARD); // Set left motor direction to backward
  // delay(1000);
  // delay(1000);
  // Example usage
  // lineFollowVarience();

  // if (IR_array[11] == 1){
  //   speed(255,255);
  //   motor(LEFT_MOTOR, FORWARD);
  //   motor(RIGHT_MOTOR, FORWARD);
  //   delay(500);
  //   speed(0,255);
  //   motor(LEFT_MOTOR, BACKWARD);
  //   motor(RIGHT_MOTOR, FORWARD);
  //   delay(1000);
  //   speed(2,0);
  //   speed(1,0);
  //   delay(1000);
  // }



  // // You can adjust speed and direction as needed
  // delay(1000); // Delay for 1 second
}

// Function to control motor speed
void speed(int motor, int speed) {
  if (motor == RIGHT_MOTOR) {
    analogWrite(ENR, speed);
  } else if (motor == LEFT_MOTOR) {
    analogWrite(ENL, speed);
  }
}

void TASK1() {

  lineFollow();

  Serial.print("TASK1");

  if (task1_color == false and IR_array[11] == 1 and IR_array[10] == 1) {
    speed(RIGHT_MOTOR, 0);
    speed(LEFT_MOTOR, 0);
    delay(1000);
    beep(10);
    for (int i =0; i<3; i++) Serial.println("WALL");
    
    while(true):
      if(Serial.available()){
        String msg = Serial.readChar();
        break;
      }
    if (msg == "GREEN"){
      col = 0;
    }
    else{
      col = 2;
    }
    task1_color = true;
    Drive_constant = 200;

    left_turn();
  }

  if (task1_color == true and sensorSum() >= 11) {
    speed(1,0);
    speed(2,0);
    LEVEL++;
    beep(15);
    left_turn();
  }
  speed(RIGHT_MOTOR, Left_drive);  // Set right motor speed to 150
  motor(RIGHT_MOTOR, FORWARD);     // Set right motor direction to forward

  speed(LEFT_MOTOR, Right_drive);  // Set left motor speed to 200
  motor(LEFT_MOTOR, FORWARD);      // Set left motor direction to backward
}


void TASK2() {

  lineFollow();
  Serial.print("TASK2");

  if (sensorSum() > 10) {
    speed(1, 0);
    speed(2, 0);
    beep(10);
    Serial.println("JUNCTION");
    Serial.println("JUNCTION");
    Serial.println("JUNCTION");

    while(true){
      if (Serial.availble()){}
    }
    if (col == 1){
      left_turn();
    }else{
      right_turn();
    }
    LEVEL++;
  }

  speed(RIGHT_MOTOR, Left_drive);  // Set right motor speed to 150
  motor(RIGHT_MOTOR, FORWARD);     // Set right motor direction to forward

  speed(LEFT_MOTOR, Right_drive);  // Set left motor speed to 200
  motor(LEFT_MOTOR, FORWARD);      // Set left motor direction to backward
}


bool got_turn = false;
bool is_cylinder = false;
void TASK3() {

  lineFollow();
  Serial.print("TASK3");

  if (sensorSum() > 10 and got_turn == false) {
    got_turn = true;
    speed(1, 0);
    speed(2, 0);
    beep(10);
    right_turn();
    now_time = millis()/1000;
    // LEVEL++;
  }

  if (got_turn) {
    lineFollowVarience();

    if (millis() - now_time > 4000){
      if (IR_array[0] == 1 and IR_array[1] == 1) {
        speed(1, 0);
        speed(2, 0);
        beep(10);
        right_turn();
        delay(100);
        LEVEL++;
        if (cylinder_variance < variance) {
          is_cylinder = true;
          digitalWrite(GREEN_LED, HIGH);
        } else {
          is_cylinder = false;
          digitalWrite(BLUE_LED, HIGH);
        }
      }
    }
  }

  speed(RIGHT_MOTOR, Left_drive);  // Set right motor speed to 150
  motor(RIGHT_MOTOR, FORWARD);     // Set right motor direction to forward

  speed(LEFT_MOTOR, Right_drive);  // Set left motor speed to 200
  motor(LEFT_MOTOR, FORWARD);      // Set left motor direction to backward
}

bool task4_bend = false;
void TASK4() {

  lineFollow();
  Serial.print("TASK4");

  if (sensorSum() > 10 and task4_bend == false) {
    task4_bend = true;
    speed(1, 0);
    speed(2, 0);
    beep(10);
    left_turn();
    LEVEL++;
  }

  speed(RIGHT_MOTOR, Left_drive);  // Set right motor speed to 150
  motor(RIGHT_MOTOR, FORWARD);     // Set right motor direction to forward

  speed(LEFT_MOTOR, Right_drive);  // Set left motor speed to 200
  motor(LEFT_MOTOR, FORWARD);      // Set left motor direction to backward
}

bool task5_bend_1 = false;
bool task5_bend_2 = false;
bool task5_bend_3 = false;

void TASK5() {

  Serial.println("Testing");

  // lox2.rangingTest(&measure2, false);  // pass in 'true' to get debug data printout!
  // if (measure2.RangeStatus != 4) {
  //   front_tof = measure2.RangeMilliMeter;
  //   Serial.print(front_tof);
  // } else {
  //   Serial.print(F("Out of range"));
  // }

  lineFollow();
  Serial.print("TASK5");

  if (sensorSum() > 10 and task5_bend_1 == false) {
    task5_bend_1 = true;

    trigMagnet(true);
    speed(1, 0);
    speed(2, 0);
    beep(10);
    left_turn();

    speed(1,150);
    speed(2,150);
    motor(LEFT_MOTOR, FORWARD); 
    motor(RIGHT_MOTOR, FORWARD);

    delay(500);


   speed(LEFT_MOTOR, 200);      // Set left motor speed to 200
   speed(RIGHT_MOTOR, 200);      // Set left motor speed to 200
    motor(RIGHT_MOTOR, FORWARD);  // Set left motor direction to backward

    motor(LEFT_MOTOR, BACKWARD);  // Set left motor direction to backward

    delay(1300);


  }

  speed(RIGHT_MOTOR, Left_drive);  // Set right motor speed to 150
  motor(RIGHT_MOTOR, FORWARD);     // Set right motor direction to forward

  speed(LEFT_MOTOR, Right_drive);  // Set left motor speed to 200
  motor(LEFT_MOTOR, FORWARD);      // Set left motor direction to backward
}

// Function to control motor direction
void motor(int motor, int direction) {
  if (motor == RIGHT_MOTOR) {
    if (direction == FORWARD) {
      digitalWrite(Motor_Right_Forward, HIGH);
      digitalWrite(Motor_Right_Backward, LOW);
    } else if (direction == BACKWARD) {
      digitalWrite(Motor_Right_Forward, LOW);
      digitalWrite(Motor_Right_Backward, HIGH);
    }
  } else if (motor == LEFT_MOTOR) {
    if (direction == FORWARD) {
      digitalWrite(Motor_Left_Forward, HIGH);
      digitalWrite(Motor_Left_Backward, LOW);
    } else if (direction == BACKWARD) {
      digitalWrite(Motor_Left_Forward, LOW);
      digitalWrite(Motor_Left_Backward, HIGH);
    }
  }
}

int sensorSum() {
  int litIR = 0;
  for (int i = 0; i < 12; i++) {
    litIR += IR_array[i];
  }
  return litIR;
}

void lineFollowVarience() {
  readIRArray();

  motor(RIGHT_MOTOR, FORWARD);
  motor(LEFT_MOTOR, FORWARD);


  int array_lit_amount = 0;
  for (int i = 0; i < IR_ARRAY_LENGTH; i++) {
    array_lit_amount += IR_array[i];
  }

  // The following two lines should change with respect to IR array length.

  // For 8 IR array.
  float right_sum = IR_array[2] + IR_array[3] + IR_array[4] + IR_array[5];
  float left_sum = IR_array[6] + IR_array[7] + IR_array[8] + IR_array[9];

  // For 12 extended IR array.
  left_sum = -34 * IR_array[0] - 27 * IR_array[1] - 21 * IR_array[2] - 17 * IR_array[3] - 10 * IR_array[4] - 3 * IR_array[5];
  right_sum = 3 * IR_array[6] + 10 * IR_array[7] + 17 * IR_array[8] + 21 * IR_array[9] + 27 * IR_array[10] + 34 * IR_array[11];

  int position = left_sum + right_sum;

  // prev_error_history[stack_pointer] = position;
  // stack_pointer++;
  // if (stack_pointer == 10)
  // {
  //     stack_pointer = 0;
  // }

  derivative = position - prevError;
  // aded by nirosh
  integral = integral + position;
  //
  int PID_constant = Kp_circle * position + Ki_circle * integral + Kd_circle * derivative;

  prevError = position;

  // Drive_constant = 200;

  int offset = -30;  // For correcting motor speeds
  Left_drive = Drive_constant + offset + PID_constant;
  Right_drive = Drive_constant - offset - PID_constant;

  // Limiting to 0 - 255 range
  Left_drive = min(max(Left_drive, 0), 255);
  Right_drive = min(max(Right_drive, 0), 255);

  read_dual_sensors();
  updateWelford(side_tof);

  // To avoid division by zero, ensure we have at least two observations to calculate variance
  if (count > 1) {
    variance = M2 / (count - 1);  // Sample variance
    // For population variance, use: variance = M2 / count;

    Serial.print("Mean: ");
    Serial.print(mean);
    Serial.print(", Variance: ");
    Serial.println(variance);
  }
  prev_position = position;
}

/*
    Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
    Keep sensor #1 awake by keeping XSHUT pin high
    Put all other sensors into shutdown by pulling XSHUT pins low
    Initialize sensor #1 with lox.begin(new_i2c_address) Pick any number but 0x29 and it must be under 0x7F. Going with 0x30 to 0x3F is probably OK.
    Keep sensor #1 awake, and now bring sensor #2 out of reset by setting its XSHUT pin high.
    Initialize sensor #2 with lox.begin(new_i2c_address) Pick any number but 0x29 and whatever you set the first sensor to
 */
void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and resetting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    // while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    // while(1);
  }
}

void read_dual_sensors() {

  lox1.rangingTest(&measure1, false);  // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false);  // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print(F("1: "));
  if (measure1.RangeStatus != 4) {  // if not out of range
    side_tof = measure1.RangeMilliMeter;
    Serial.print(side_tof);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.print(F(" "));

  // print sensor two reading
  Serial.print(F("2: "));
  if (measure2.RangeStatus != 4) {
    front_tof = measure2.RangeMilliMeter;
    Serial.print(front_tof);
  } else {
    Serial.print(F("Out of range"));
  }

  Serial.println();
}


void updateWelford(int newValue) {
  if(newValue < 30){
    count++;
    float delta = newValue - mean;
    mean += delta / count;
    float delta2 = newValue - mean;
    M2 += delta * delta2;
  }
}


void lineFollow() {
  readIRArray();

  motor(RIGHT_MOTOR, FORWARD);
  motor(LEFT_MOTOR, FORWARD);


  int array_lit_amount = 0;
  for (int i = 0; i < IR_ARRAY_LENGTH; i++) {
    array_lit_amount += IR_array[i];
  }

  // The following two lines should change with respect to IR array length.

  // For 8 IR array.
  right_sensors = IR_array[2] + IR_array[3] + IR_array[4] + IR_array[5];
  left_sensors = IR_array[6] + IR_array[7] + IR_array[8] + IR_array[9];

  // For 12 extended IR array.
  left_sum = -27 * IR_array[0] - 21 * IR_array[1] - 16 * IR_array[2] - 11 * IR_array[3] - 6 * IR_array[4] - 3 * IR_array[5];
  right_sum = 3 * IR_array[6] + 6 * IR_array[7] + 11 * IR_array[8] + 16 * IR_array[9] + 21 * IR_array[10] + 27 * IR_array[11];

  int position = left_sum + right_sum;

  // prev_error_history[stack_pointer] = position;
  // stack_pointer++;
  // if (stack_pointer == 10)
  // {
  //     stack_pointer = 0;
  // }

  derivative = position - prevError;
  // aded by nirosh
  integral = integral + position;
  //
  int PID_constant = Kp * position + Ki * integral + Kd * derivative;

  prevError = position;

  // Drive_constant = 200;

  int offset = -30;  // For correcting motor speeds
  Left_drive = Drive_constant + offset + PID_constant;
  Right_drive = Drive_constant - offset - PID_constant;

  // Limiting to 0 - 255 range
  Left_drive = min(max(Left_drive, 0), 255);
  Right_drive = min(max(Right_drive, 0), 255);

  // Serial.println(Left_drive, Right_drive);

  // // if ((array_lit_amount == 0 or (IR_array[0] == 0 and IR_array[1] == 0 and IR_array[2] == 1 and IR_array[3] == 0 and IR_array[4] == 0 and IR_array[5] == 0 and IR_array[6] == 0 and IR_array[7] == 0))and mode != "bend")
  // if (array_lit_amount == 0 and mode != "bend")
  // {
  //     Left_drive = 0;
  //     Right_drive = 0;
  //     analogWrite(ENL, Left_drive);
  //     analogWrite(ENR, Right_drive);
  //     digitalWrite(Debug_led, HIGH);
  //     delay(50);
  //     digitalWrite(Debug_led, LOW);

  //     readIRArray();

  //     for (int i = 0; i < IR_ARRAY_LENGTH; i++) {
  //         array_lit_amount += IR_array[i];
  //     }

  //     if (array_lit_amount == 0 and mode != "bend")
  //     {
  //         mode = "deviated";
  //         // buzzer_beep();
  //         // delay(20);
  //         // buzzer_beep();
  //         // delay(20);
  //         // buzzer_beep();
  //         // delay(20);
  //         // buzzer_beep();
  //         // delay(20);
  //         // buzzer_beep();
  //         // delay(20);
  //     }
  // }

  // if (mode == "normal" and bend_done == false){
  //   //bend detection
  //   if(right_sum >= 8.5 && left_sum > -4){
  //     //breaking
  //     motor(RIGHT_MOTOR, BACKWARD);
  //     motor(LEFT_MOTOR, BACKWARD);
  //     now_time = millis();
  //     mode = "bend";

  //     bend_done = true;

  //     bend_start = true;

  //   }
  // }

  // Parking
  // if (array_lit_amount == 12){
  //     Switch = 0;
  // }

  // if (mode == "bend")
  // {
  //     if (bend_start == true)
  //     {
  //         bend_start = false;
  //         bend_condition = false;

  //         while (digitalRead(right_bend_sensor) == 1)
  //         {
  //             digitalWrite(buzzer_pin, HIGH);
  //             Serial.print("in loop:  ");
  //             Serial.println(digitalRead(right_bend_sensor));
  //             motor(RIGHT_MOTOR, FORWARD);
  //             motor(LEFT_MOTOR, FORWARD);
  //             Left_drive = 140;
  //             Right_drive = 140;
  //             analogWrite(ENL, Left_drive);
  //             analogWrite(ENR, Right_drive);
  //         }
  //         analogWrite(ENL, 0);
  //         analogWrite(ENR, 0);
  //         // delay(800);
  //         array_lit_amount = 0;
  //     }
  //     // analogWrite(ENL,0);
  //     // analogWrite(ENR,0);
  //     // delay(1000);

  //     digitalWrite(buzzer_pin, LOW);
  //     // buzzer_beep();

  //     Serial.println(array_lit_amount);

  //     motor(RIGHT_MOTOR, BACKWARD);
  //     motor(LEFT_MOTOR, FORWARD);

  //     Left_drive = 155;
  //     Right_drive = 155;
  //     Serial.println("in turning mode");

  //     // detecting if the line contacts the sensor
  //     if (bend_condition == false)
  //     {
  //         if (array_lit_amount >= 2)
  //         {

  //             bend_condition = true;
  //         }
  //     }
  //     else
  //     {
  //         // detecting if the line comes to the middle
  //         if (IR_array[3] == 1 or IR_array[4] == 1)
  //         {

  //             Serial.print("bend over");
  //             mode = "normal";

  //             // motor(RIGHT_MOTOR, BACKWARD);
  //             // motor(LEFT_MOTOR, BACKWARD);

  //             bend_condition = false;

  //             buzzer_beep();
  //             delay(100);
  //             buzzer_beep();

  //             analogWrite(ENL, 0);
  //             analogWrite(ENR, 0);
  //             delay(500);
  //         }
  //     }
  // }

  // //  // Deviation detection
  // if (mode == "deviated")
  // {
  //     for (int p = 0; p < 10; p++)
  //     {
  //         prev_history_sum += prev_error_history[p];
  //     }

  //     if (prev_history_sum > 0)
  //     {
  //         motor(RIGHT_MOTOR, BACKWARD);
  //         motor(LEFT_MOTOR, FORWARD);
  //         Left_drive = 230;
  //         Right_drive = 230;
  //         analogWrite(ENL, Left_drive);
  //         analogWrite(ENR, Right_drive);
  //         delay(300);
  //     }
  //     else if (prev_history_sum < 0)
  //     {
  //         motor(RIGHT_MOTOR, FORWARD);
  //         motor(LEFT_MOTOR, BACKWARD);
  //         Left_drive = 230;
  //         Right_drive = 230;
  //         analogWrite(ENL, Left_drive);
  //         analogWrite(ENR, Right_drive);
  //         delay(300);
  //     }
  //     else
  //     {
  //         motor(RIGHT_MOTOR, BACKWARD);
  //         motor(LEFT_MOTOR, BACKWARD);
  //         Left_drive = 200;
  //         Right_drive = 200;
  //         analogWrite(ENL, Left_drive);
  //         analogWrite(ENR, Right_drive);
  //         delay(300);
  //     }
  //     prev_history_sum = 0;
  //     mode = "normal";
  // }

  prev_position = position;
}

void readIRArray() {
  for (int i = 0; i < IR_ARRAY_LENGTH - 4; i++) {
    // Serial.print(analogRead(i + 8));
    Serial.print(" ");
    if (LINE_COLOR == "WHITE") {
      temp = analogRead(i + 8) <= Ir_thresholds[i];
    } else if (LINE_COLOR == "BLACK") {
      temp = analogRead(i + 8) > Ir_thresholds[i];
    }
    // IR_array[IR_ARRAY_LENGTH - 1 - i] = temp;

    // IR_array[i] = temp; // This is correct if pins are connected in order.
    IR_array[i + 2] = temp;

    // This part is needed because the pins are connected not in order.
    // if (i < 8) {
    //     IR_array[i + 2] = temp;
    // }
    // else {
    //     if (i == 8) IR_array[0] = temp;
    //     else if (i == 9) IR_array[1] = temp;
    //     else if (i == 10) IR_array[10] = temp;
    //     else if (i == 11) IR_array[11] = temp;
    // }
  }

  IR_array[0] = LINE_COLOR == "BLACK" ? digitalRead(A2) : 1 - digitalRead(A2);
  IR_array[1] = LINE_COLOR == "BLACK" ? digitalRead(A3) : 1 - digitalRead(A3);
  IR_array[10] = LINE_COLOR == "BLACK" ? digitalRead(A1) : 1 - digitalRead(A1);
  IR_array[11] = LINE_COLOR == "BLACK" ? digitalRead(A0) : 1 - digitalRead(A0);


  // for (int i = 0; i < IR_ARRAY_LENGTH; i++) {
  //   Serial.print(IR_array[i]);
  //   Serial.print(" ");
  // }
  // Serial.println();
}

void calibrate() {
  motor(RIGHT_MOTOR, BACKWARD);
  motor(LEFT_MOTOR, FORWARD);

  // Stop motors when calibrating
  analogWrite(ENL, 255);
  analogWrite(ENR, 255);

  // make sensor_max_values array of length 8 equal to sensor calibration array
  int sensor_max_values[calibrate_array_size] = { 0 };

  // sensor min values array
  int sensor_min_values[calibrate_array_size] = { 0 };

  int now_time_for_calibration = millis();
  Serial.print("now time for calibration: ");
  Serial.print(now_time_for_calibration);
  Serial.print("  millis: ");
  Serial.println(millis());

  while (millis() - now_time_for_calibration < 5000) {
    Serial.println("Caliberating IR");

    bool allZeros = true;
    for (int i = 0; i < calibrate_array_size; i++) {
      if (sensor_max_values[i] != 0) {
        allZeros = false;
        break;  // No need to continue checking if we find a non-zero value
      }
    }

    if (allZeros) {
      for (int i = 0; i < calibrate_array_size; i++) {
        sensor_max_values[i] = analogRead(i);
        sensor_min_values[i] = analogRead(i);
      }
    }

    for (int i = 0; i < calibrate_array_size; i++) {
      if (analogRead(i + 8) > sensor_max_values[i]) {
        sensor_max_values[i] = analogRead(i);
      }
      if (analogRead(i + 8) < sensor_min_values[i]) {
        sensor_min_values[i] = analogRead(i);
      }
    }
  }
  Serial.print("int IR_thresholds[] = {");
  for (int i = 0; i < calibrate_array_size; i++) {
    Serial.print(sensor_max_values[i]);
    Serial.print(", ");
  }
  Serial.print("};    ");
  Serial.print("int IR_thresholds[] = {");
  for (int i = 0; i < calibrate_array_size; i++) {
    Serial.print(sensor_min_values[i]);
    Serial.print(", ");
  }
  Serial.print("};    ");

  for (int i = 0; i < calibrate_array_size; i++) {
    Ir_thresholds[i] = (sensor_max_values[i] + sensor_min_values[i]) / 5 + sensor_min_values[i];
  }

  Serial.print("int IR_thresholds[] = {");
  for (int i = 0; i < calibrate_array_size; i++) {
    Serial.print(Ir_thresholds[i]);
    Serial.print(", ");
  }
  Serial.println("};    ");

  // save the array to eeprom
  for (int i = 0; i < calibrate_array_size; i++) {
    // EEPROM.write(i, byte(Ir_thresholds[i]/4));
    EEPROM.put(i * sizeof(int), Ir_thresholds[i]);
  }

  delay(1000);

  analogWrite(ENL, 0);
  analogWrite(ENR, 0);

  delay(4000);
}

void trigMagnet(bool state) {
  digitalWrite(magnet, !state);
}

void beep(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(Buzzer, HIGH);
    delay(50);
    digitalWrite(Buzzer, LOW);
    delay(50);
  }
}

void left_turn() {
  speed(RIGHT_MOTOR, 200);      // Set right motor speed to 150
  motor(RIGHT_MOTOR, FORWARD);  // Set right motor direction to forward

  speed(LEFT_MOTOR, 200);      // Set left motor speed to 200
  motor(LEFT_MOTOR, FORWARD);  // Set left motor direction to backward

  delay(650);

  motor(LEFT_MOTOR, BACKWARD);  // Set left motor direction to backward

  delay(650);
}

void right_turn() {
  speed(RIGHT_MOTOR, 200);      // Set right motor speed to 150
  motor(RIGHT_MOTOR, FORWARD);  // Set right motor direction to forward

  speed(LEFT_MOTOR, 200);      // Set left motor speed to 200
  motor(LEFT_MOTOR, FORWARD);  // Set left motor direction to backward

  delay(650);

  motor(RIGHT_MOTOR, BACKWARD);  // Set left motor direction to backward

  delay(650);
}
