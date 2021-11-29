// Bot 4 Esp Program to Control the movement of the bot and the Servo Motor
// Important Note : Esp8266 is by default active low state 
// It means the HIGH State = 0 and LOW  State = 1

// Declaring the Header Files required for the Program
#include <MPU6050_tockn.h>
#include <Servo.h>
#include <Wire.h>

// Creating Object for Servo and MPU Sensor
MPU6050 mpu6050(Wire);
Servo servo;

// Defining the Pins for Motor Drivers
int ena = D0;
int in1 = D3;
int in2 = D4;
int in3 = D5;
int in4 = D7;
int enb = D8;

// Defining the Pin for Servo Motor Control
int sm = D6;

// Declare Speed Control Values
int linear = 500;
int turn = 400;

// Declare Variable to Store the Value of MPU 6050
float z = 0;
float z_ang = 0;
float z_cal = 0;
float dev_1 = 0;
float dev_2 = 0;
int c=0;

// Function to get the angle from MPU Sensor
float mpu(bool calibrate=false) {
    mpu6050.update();
    // Getting Calibration Value on setup
    if (calibrate==true) {
        z = mpu6050.getAngleZ();
        Serial.println("MPU Calibration Done");
        Serial.print("Calibration Value : ");
        Serial.println(z);
        //delay(1000);
    }
    // Get Yaw Values from MPU
    else {
        z = mpu6050.getAngleZ();
        Serial.print("angleZ : ");
        Serial.println(z);
    }
    return z;
}

// Function to Move the bot in different Directions
void movement(int direction, float angle=0) {
    if (direction == 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
    }
    if (direction == 1) {
      c=c+1;
      z_ang = mpu();
      if(c<=1)
      {
        dev_1 = z_ang + 10;
        dev_2 = z_ang - 10;
      }
      if((z_ang <= dev_1) || (z_ang >= dev_2)){
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);
      analogWrite(ena, linear);
      analogWrite(enb, linear);
      Serial.println("Forward");
      }
      if(z_ang > dev_1){
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          analogWrite(ena, turn);
          analogWrite(enb, turn);
          Serial.println("Right");
      }
      if(z_ang < dev_2){
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, turn);
        analogWrite(enb, turn);
        Serial.println("Left");
        }
      
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
    }
    if (direction == 2) {
        z_ang = mpu();
        z_cal = (-(angle)+z_ang);
        while(z_ang >= z_cal) {
            z_ang = mpu();
            Serial.print("Z angle : ");
            Serial.println(z_ang);
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            analogWrite(ena, turn);
            analogWrite(enb, turn);
            Serial.println("Right");
        }
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
    }
    if (direction == 3) {
        z_ang = mpu();
        z_cal = ((angle)+z_ang);
        while(z_ang <= z_cal) {
            z_ang = mpu();
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(ena, turn);
            analogWrite(enb, turn);
            Serial.println("Left");
        }
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
    }
    if (direction == 4) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, linear);
        analogWrite(enb, linear);
        Serial.println("Backward");
        delay(1000);
    }
    if (direction == 5) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, turn);
        analogWrite(enb, turn);
        Serial.println("Fine Right");
        delay(100);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        delay(500);
    }
    if (direction == 6) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, turn);
        analogWrite(enb, turn);
        Serial.println("Fine Left");
        delay(100);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        delay(500);
    }
    if (direction == 7) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, turn);
        analogWrite(enb, turn);
        Serial.println("180 - degree Turn");
        delay(730);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
        delay(500);
    }
}

// Function to control Servo Motor
void servo_control(int pos) {
    if (pos == 0) servo.write(0);
    if (pos == 1) servo.write(180);
}

void setup() {
    // Set Up esp8266 as Output or Input
    pinMode(ena, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(enb, OUTPUT);

    // Attach Servo Motor Signal Pin
    servo.attach(sm);

    // On Boot Set Speed Control Pins to zero
    analogWrite(ena, 0);
    analogWrite(enb, 0);

    // Use ESP8266 serial to monitor the process
    // Note: Change your bps at the serial monitor to the below mentioned Value
    Serial.begin(115200);

    //Initialize the I2C Communication
    Wire.begin();
    // Initialize the MPU 6050 Sensor
    mpu6050.begin();
    z_cal  = mpu(true);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(1);
    movement(0);
}

// Loop the neccessary Functions
void loop() {
    //servo_control(0);
//    movement(1);
//    delay(1000);
//    movement(0);
//    delay(1000);
//    movement(2, 45);
//    delay(1000);
//    movement(3, 45);
//    delay(1000);
//    movement(2, 90);
//    delay(1000);
//    movement(3, 90);
//    delay(1000);
//    movement(2, 180);
//    delay(1000);
//    movement(3, 180);
//    delay(1000);
//    movement(1);
//    delay(2000);
//    movement(0);
//    delay(1000);
}
