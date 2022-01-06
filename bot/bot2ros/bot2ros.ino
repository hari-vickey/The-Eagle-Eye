// To connect esp with ROS run this below mentioned command
// rosrun rosserial_python serial_node.py tcp
// Important Note : Esp8266 is by default active HIGH state 

#define ROSSERIAL_ARDUINO_TCP

// Declaring the Header Files required for the Program
#include <ESP8266WiFi.h>
#include <MPU6050_tockn.h>
#include <Servo.h>
#include <Wire.h>

// Including ROS Libraries for Subscribing to the ROS Topics
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

// Creating Object for Servo and MPU Sensor
MPU6050 mpu6050(Wire);
Servo servo;

// Declaring wifi credentials
const char* ssid = "hari";
const char* password = "password";

// Setting the rosserial socket server IP address
// Use hostname -I in terminal to get the IP
// Note : Varies for different wifi connection
IPAddress server(192,168,43,246);// Hari
//IPAddress server(192,168,225,59);// Bijoy

// Set the rosserial socket server port
const uint16_t serverPort = 11454;

// Creating a ROS Node
ros::NodeHandle n2;

// Defining the Pins for Motor Drivers
int ena = D5;
int in1 = D0;
int in2 = D3;
int in3 = D4;
int in4 = D7;
int enb = D6;

// Defining the Pin for Servo Motor Control
int sm = D8;

// Declare Speed Control Values
int l1 = 200;
int r1 = 200;
int l2;
int r2;

// Defining Variables
int count = 1;

// Declare Variable to Store the Value of MPU 6050
float z = 0, zg = 0, z_ang = 0, z_cal = 0;

// Function to get the angle from MPU Sensor
float mpu() {
    mpu6050.update();
    z = mpu6050.getAngleZ();
    return z;
}

// Function to Move the bot in different Directions
void movement(int direction, float angle=0) {
    if (direction == 0) {
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
       count = 1;
    }
    if (direction == 1) {
        if (count == 1) zg = mpu();
        else if (count == 500) count = 1;
        count++;
        z_ang = mpu();
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, l1);
        analogWrite(enb, r1);
       if(z_ang == zg)
       {
            analogWrite(ena, l1);
            analogWrite(enb, r1);
       }
       else if(z_ang > zg)
       {
            r2 = r1 - (5*(z_ang - zg));
            analogWrite(ena, l1);
            analogWrite(enb, r2);
       }
       else if(z_ang < zg)
       {
            l2 = l1 - (5*(z_ang - zg));
            analogWrite(ena, l2);
            analogWrite(enb, r1);
       }
        Serial.println("forward");
    }
    if (direction == 3) {
        z_ang = mpu();
        z_cal = (-(angle)+z_ang);
        while(z_ang >= z_cal) {
            z_ang = mpu();
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, HIGH);
            analogWrite(ena, l1);
            analogWrite(enb, r1);
            Serial.println("Clock-Wise Rotation");
        }
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
        count = 1;
    }
    if (direction == 2) {
        z_ang = mpu();
        z_cal = ((angle)+z_ang);
        while(z_ang <= z_cal) {
            z_ang = mpu();
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
            digitalWrite(in3, HIGH);
            digitalWrite(in4, LOW);
            analogWrite(ena, l1);
            analogWrite(enb, r1);
            Serial.println("Anti Clock-Wise Rotation");
        }
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
        count = 1;
    }
    if (direction == 4) {
        if (count == 1) zg = mpu();
        else if (count == 50) count = 1;
        count++;
        z_ang = mpu();
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, l1);
        analogWrite(enb, r1);
        if(z_ang == zg)
        {
            analogWrite(ena, l1);
            analogWrite(enb, r1);
        }
        else if(z_ang < zg)
        {
            r2 = r1 - (5*(z_ang - zg));
            analogWrite(ena, l1);
            analogWrite(enb, r2);
        }
        else if(z_ang > zg)
        {
            l2 = l1 - (5*(z_ang - zg));
            analogWrite(ena, l2);
            analogWrite(enb, r1);
        }
        Serial.println("Reverse");
    }
    if (direction == 5) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, l1);
        analogWrite(enb, 150);
        Serial.println("Clock-Wise Rotation");
        delay(8);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
        count = 1;
    }
    if (direction == 6) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, 150);
        analogWrite(enb, r1);
        Serial.println("Anti Clock-Wise Rotation");
        delay(8);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
        count = 1;
    }
    if (direction == 7) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, l1);
        analogWrite(enb, 150);
        Serial.println("Clock-Wise Rotation");
        delay(8);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
        count = 1;
    }
    if (direction == 8) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(ena, 150);
        analogWrite(enb, r1);
        Serial.println("Anti Clock-Wise Rotation");
        delay(8);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
        count = 1;
    }
    if (direction == 9) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, l1);
        analogWrite(enb, 150);
        Serial.println("Clock-Wise Rotation");
        delay(8);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
        count = 1;
    }
    if (direction == 10) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(ena, 150);
        analogWrite(enb, r1);
        Serial.println("Anti Clock-Wise Rotation");
        delay(8);
        analogWrite(ena, 0);
        analogWrite(enb, 0);
        Serial.println("Stop");
        count = 1;
    }
    
}

// Function to control Servo Motor
void servo_control(int pos) {
    if (pos == 0) {
        servo.write(0);
    }
    if (pos == 1) {
        servo.write(180);
    }
}

// Callback function for control signal
void controlCb(const std_msgs::Int16MultiArray& con){
    Serial.println(con.data[0]);
    Serial.println(con.data[1]);
    Serial.println(con.data[2]);
    movement(con.data[0], con.data[1]);
    if(con.data[2] == 1) servo_control(con.data[2]);
    if(con.data[2] == 0) servo_control(con.data[2]);
}

// Subscribe to the ROS Topic
ros::Subscriber<std_msgs::Int16MultiArray> sub_con("bot2/control_signal", &controlCb);

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
    z_cal = mpu();
    zg = mpu();
    // Set the connection to rosserial socket server
    n2.getHardware()->setConnection(server, serverPort);

    // Initialize ROS Node
    n2.initNode();

    // Subscribing the ROS Topics
    n2.subscribe(sub_con);
}

// Loop the neccessary Functions
void loop() {
    n2.spinOnce();
}
