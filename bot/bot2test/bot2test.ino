// Bot 2 Esp Program to Control the movement of the bot and the Servo Motor
// Important Note : Esp8266 is by default active low state 
// It means the HIGH State = 0 and LOW  State = 1

// Declaring the Header Files required for the Program
#include <MPU6050_tockn.h>
#include <MobaTools.h>
#include <Wire.h>

// Creating Object for Servo and MPU Sensor
MPU6050 mpu6050(Wire);
MoToServo servo;

// Defining the Pins for Motor Drivers
int ena = D0;
int in1 = D3;
int in2 = D4;
int in3 = D5;
int in4 = D6;
int enb = D7;

// Defining the Pin for Servo Motor Control
int sm = D8;

// Declare Speed Control Values
int SPD = 500;
int SPD1 = 500;
int forw = 550;
// Declare Variable to Store the Value of MPU 6050
float z = 0;
float z_ang = 0;
float z_cal = 0;
float zg = 0;

// Function to get the angle from MPU Sensor
float mpu(bool calibrate=false) {
    mpu6050.update();
    // Getting Calibration Value on setup
    if (calibrate==true) {
        z = mpu6050.getAngleZ();
        Serial.println("MPU Calibration Done");
        Serial.print("Calibration Value : ");
        Serial.println(z);
        delay(1000);
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
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
    }
    if (direction == 1) {
        zg = mpu();
      for(int i=0;i<100;i++)
      {
        z_ang = mpu();
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        if(z_ang==zg)
        {
          analogWrite(ena, SPD);
          analogWrite(enb, SPD);
        }
        if(z_ang<zg)
        {
          analogWrite(ena, SPD);
          analogWrite(enb, forw);
        }
        if(z_ang>zg)
        {
          analogWrite(ena, forw);
          analogWrite(enb, SPD);
        }
        zg = z_ang;
        Serial.println("Forward");
      }
    }
    if (direction == 2) {
        z_ang = mpu();
        z_cal = (-(angle+10)+z_ang);
        while(z_ang >= z_cal) {
            z_ang = mpu();
            // Serial.print("Z angle : ");
            // Serial.println(z_ang);
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            analogWrite(ena, SPD);
            analogWrite(enb, SPD);
            Serial.println("Right");
        }
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
    }
    if (direction == 3) {
        z_ang = mpu();
        z_cal = ((angle+10)+z_ang);
        while(z_ang <= z_cal) {
            z_ang = mpu();
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(ena, SPD);
            analogWrite(enb, SPD1);
            Serial.println("Left");
        }
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
    }
    if (direction == 4) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, SPD);
        analogWrite(enb, SPD1);
        Serial.println("Backward");
        delay(1000);
    }
    if (direction == 5) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, SPD);
        analogWrite(enb, SPD1);
        Serial.println("Fine Right");
        delay(100);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        delay(500);
    }
    if (direction == 6) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, SPD);
        analogWrite(enb, SPD1);
        Serial.println("Fine Left");
        delay(100);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        delay(500);
    }
    if (direction == 7) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, SPD);
        analogWrite(enb, SPD1);
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
    delay(3000);
    // Use ESP8266 serial to monitor the process
    // Note: Change your bps at the serial monitor to the below mentioned Value
    Serial.begin(115200);

    //Initialize the I2C Communication
    Wire.begin();
    // Initialize the MPU 6050 Sensor
    mpu6050.begin();
    z_cal  = mpu(true);
}

// Loop the neccessary Functions
void loop() {
    movement(1);
    movement(1);
    movement(2, 45);
    delay(1000);
    servo_control(1);
    delay(1000);
    movement(3, 45);
    delay(500);
    movement(2, 90);
    delay(1000);
    servo_control(1);
    delay(1000);
    movement(3, 90);
    delay(500);
    movement(0);
    delay(2000);
}
