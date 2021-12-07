# The_Eagle_Eye
## Bot Programs

This folder contains all the codes for each individual bot which can be utilized for testing purposes and also to be utilized in the ros programs for autonomous navigation.

Each Bot consist of the following Components integrated with it for successful navigation and package delivery. The Circuit diagram will be added soon.

### Components In the Bot

1. NodeMCU V1.0
2. 7805 Voltage Regulator
3. MPU6050 Gyroscopic sensor
4. Servo Motor
5. L298n Motor Driver
6. L shaped BO Motor
7. Li-ion(11V, 2600/2200maH) Battery

## Working 

**NodeMCU** acts as a processing unit for all the bots which is utilized to connect the bot to the ROS server running on the remote PC. So, that the bot can receive commands from them and converts them to digital signal to move the bot and actuate the servo motor.

The **7805 Voltage Regulator** is used to covert 11V input to 5V output which is then provided as input to the NodeMCU, MPU6050, servo motor.

**MPU6050** sensor used to make the bot move precisely in any direction and also to take accurate rotation for any given degree from the server.

**Servo Motor** will act as the barrier to the package from falling from the bot along with the guide ways attached to the bot. Also, by default the servo will be raised up for preventing the package to fall from the bot and rotates 180 degree to make the package fall from the bot.

**L298n Motor Driver** is used mainly to control the direction of the rotation of the motors. But, this component also used for controlling motor speed. As we have utilized the speed control for precise movements.

**L Shaped BO Motor** is used to reduce the actual shaft output speed of the 5V DC Motor to 100 rpm for our application. Also, we have chosen this motor because it's offer to compactable length.

The **Li-ion** Battery is made of three individual Li-ion Cells of 3.7V 2600/2200maH Capacity. The main advantage of this battery is the capability of them to offer current and recharging facility.

