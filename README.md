# 2ArmPid
This project was aimed to test and deploy PID control Law on a Two Arm PID Balance. The goal of this project was to level a wooden rod with the help of BLDC motors and propellors.
This was done to simulate the roll and pitch stability of a Quadcopter.
![gif of model](2-Arm-Pid.gif?raw=True "Gif of Arm")
The angle wrt to the horizontal was found out using MPU 9250 by fusing both gyroscope and accelerometer values by using a complimentry filter.

# Result
The Video of the working model is [linked here.](https://www.youtube.com/watch?v=0rf5-bUTrOY)

# References 
https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library

Refer this for imu interfacing.
