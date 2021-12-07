// Bot 4 Esp Program to Control the movement of the bot and the Servo Motor
// To connect esp with ROS run this below mentioned command
// rosrun rosserial_python serial_node.py tcp
// Important Note : Esp8266 is by default active low state 
// It means the HIGH State = 0 and LOW  State = 1

// Declaring the Header Files required for the Program
#include <ESP8266WiFi.h>
#include <MPU6050_tockn.h>
#include <Servo.h>
#include <Wire.h>

// Including ROS Libraries for Subscribing to the ROS Topics
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#define ROSSERIAL_ARDUINO_TCP

// Creating Object for Servo and MPU Sensor
MPU6050 mpu6050(Wire);
Servo servo;

// Declaring wifi credentials
const char* ssid = "Jiji_Tomy";
const char* password = "bijubijoy928";

// Setting the rosserial socket server IP address
// Use hostname -I in terminal to get the IP
// Note : Varies for different wifi connection
//IPAddress server(192,168,225,28);// Hari
IPAddress server(192,168,225,59);// Bijoy

// Set the rosserial socket server port
const uint16_t serverPort = 44181;

// Creating a ROS Node
ros::NodeHandle n4;

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
int linear = 150;
int linear1 = 150;
int turn = 270;

// Declare Variable to Store the Value of MPU 6050
float z = 0;
float z_ang = 0;
float z_cal = 0;
float zg = 0;

// Function to get the angle from MPU Sensor
float mpu() {
    mpu6050.update();
    z = mpu6050.getAngleZ();
    // Serial.print("angleZ : ");
    // Serial.println(z);
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
      for(int i=0;i<150;i++)
      {
        z_ang = mpu();
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        
        if(z_ang<zg)
        {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          analogWrite(ena, 0);
          analogWrite(enb, 200);
        }
        if(z_ang>zg)
        {
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
          analogWrite(ena, 200);
          analogWrite(enb, 0);
        }
        else
        {
          analogWrite(ena, linear);
          analogWrite(enb, linear1);
        }
        Serial.println("Forward");
      }
      
          analogWrite(ena, 0);
          analogWrite(enb, 0);
          delay(50);
    }
    if (direction == 2) {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            analogWrite(ena, linear);
            analogWrite(enb, linear);
            Serial.println("Clock-Wise Rotation");
    }
    if (direction == 3) {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(ena, linear);
            analogWrite(enb, linear);
            Serial.println("Anti Clock-Wise Rotation");
    }
    if (direction == 4) {
      zg = mpu();
      for(int i=0;i<200;i++)
      {
        z_ang = mpu();
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        if(z_ang<zg)
        {
          analogWrite(ena, linear);
          analogWrite(enb, 100);
        }
        if(z_ang>zg)
        {
          analogWrite(ena, 100);
          analogWrite(enb, linear1);
        }
        else
        {
          analogWrite(ena, linear);
          analogWrite(enb, linear1);
        }
        zg = z_ang;
        Serial.println("Reverse");
      }
    }
    if (direction == 5) {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(ena, turn);
            analogWrite(enb, linear);
            Serial.println("Clock-Wise Rotation");
    }
    if (direction == 6) {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(ena, linear);
            analogWrite(enb, turn);
            Serial.println("Anti Clock-Wise Rotation");
    }
}

// Function to control Servo Motor
void servo_control(int pos) {
    if (pos == 0) servo.write(0);
    if (pos == 1) servo.write(180);
}

// Callback function for control signal
void controlCb(const std_msgs::Int16MultiArray& con){
    Serial.println(con.data[0]);
    Serial.println(con.data[1]);
    Serial.println(con.data[2]);
    movement(con.data[0], con.data[1]);
    servo_control(con.data[2]);
}
// Subscribe to the ROS Topic
ros::Subscriber<std_msgs::Int16MultiArray> sub_con("bot1/control_signal", &controlCb);

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
    servo_control(0);

    // Use ESP8266 serial to monitor the process
    // Note: Change your bps at the serial monitor to the below mentioned Value
    Serial.begin(115200);

    // Connect the ESP8266 the the wifi AP
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    //Initialize the I2C Communication
    Wire.begin();
    // Initialize the MPU 6050 Sensor
    mpu6050.begin();
    z_cal  = mpu();

    // Set the connection to rosserial socket server
    n4.getHardware()->setConnection(server, serverPort);

    // Initialize ROS Node
    n4.initNode();

    // Subscribing the ROS Topics
    n4.subscribe(sub_con);
}

// Loop the neccessary Functions
void loop() {
    n4.spinOnce();
}
