#include <Wire.h>
#include <Servo.h>
#define SerialPort Serial
#include <SparkFunMPU9250-DMP.h>

// declaring variables


MPU9250_DMP imu;
double roll , pitch;
float phi = 0;
float theta = 0;
long int pre_ts=0;
float phi_n_1, theta_n_1;
float phi_dot, theta_dot, phi_quat;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float Q[4] = {1,0,0,0} ; 
  
float Q_dot [4] ;
float Q_pre [4] = {1,0,0,0} ;
float filtered_pitch;
float elapsedTime, time, timePrev;

//servo initializations

Servo right_prop;
Servo left_prop;

int MAPl,MAPr;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;

//pid constants
double kp=1.8 ;//3.55
double ki=0.005;//0.003
double kd=0.9;//2.05

double throttle = 1300;    //initial value of throttle to the motors in ms
const float desired_angle = 0; //This is the angle in which we want the
                         //balance to stay steady

void setup()
{
{
  {
  SerialPort.begin(9600);

 if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(3000);
    }
  }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

  imu.setGyroFSR(250); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(2); // Set accel to +/-2g
  imu.setLPF(10); // Set LPF corner frequency to 5Hz
  imu.setSampleRate(10); // Set sample rate to 10Hz
  imu.setCompassSampleRate(50); // Set mag rate to 10Hz
 }
  Wire.begin();
  
  right_prop.attach(3);  //attatch the right motor to pin 3(pwm pins)
  left_prop.attach(5);  //attatch the left motor to pin 5

  pre_ts=millis();
  left_prop.write(30);
  right_prop.write(30);
  delay(3000);//Give some delay, 7s, to have time to connect the propellers and let everything start up/ 
  
}

}
void loop() 
{
    timePrev = pre_ts;  // the previous time is stored before the actual time read
    elapsedTime = (millis() - timePrev) / 1000; 
    if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    filtered_pitch=getIMUData(millis() - pre_ts); //function called 
   
    
    pre_ts=millis();
  }
   //error=filtered_pitch+0.0;
  
   //Serial.println(error); 
    
  ////////////////////////////P I D////////////////////////////////////
//Calculating the error between the desired angle and the real measured angle/
//error = filtered_pitch - desired_angle;
    
//Next the proportional value of the PID is just a proportional constant *multiplied by the error/

pid_p = kp*filtered_pitch;

//The integral part should only act if we are close to the desired position but we want to fine tune the error i.e  if error is between -2 and 2 degree./
if(-3 <filtered_pitch <3)
{
  pid_i = pid_i+(ki*filtered_pitch);  
}

/* The derivate acts upon the speed of the error.As we know the speed is the amount of error that produced in a certain amount oftime divided by that time. */

pid_d = kd*((filtered_pitch - previous_error)/elapsedTime);

//The final PID values is the sum of each of this 3 parts/
PID = pid_p + pid_i + pid_d;

//min value of PWM signal is 1000us and the max is 2000./
if(PID < -1000)
{
  PID=-1000;
}
if(PID > 1000)
{
  PID=1000;
}

/*Finnaly we calculate the PWM width. We sum the desired throttle and the PID value */
pwmLeft = throttle + PID-175;
pwmRight = throttle - PID;


//Just to ensure again/

//Right
if(pwmRight < 1000)
{
  pwmRight= 1000;
}
if(pwmRight > 2000)
{
  pwmRight=2000;
}

//Left
if(pwmLeft < 1000)
{
  pwmLeft= 1000;
}
if(pwmLeft > 2000)
{
  pwmLeft=2000;
}

MAPl= map(pwmLeft,1000,2000,40,50);
MAPr= map(pwmRight,1000,2000,40,50);
MAPr=constrain(MAPr,40,50);
MAPl=constrain(MAPl,40,50);
left_prop.write(MAPl);
right_prop.write(MAPr);
previous_error = filtered_pitch; //store the previous error.
Serial.println(filtered_pitch); 


}

float getIMUData(long int dt)
{  
  
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx)/57.3;
  float gyroY = imu.calcGyro(imu.gy)/57.3;
  float gyroZ = imu.calcGyro(imu.gz)/57.3;


// NORMALIZE ACCELE VALUES
    float naccel = sqrt(accelX*accelX +accelY*accelY+accelZ*accelZ);


// normalization of acc values
    accelX =accelX/naccel;
    accelY =accelY/naccel;
    accelZ =accelZ/naccel;



//Euler angle from accel
 
   pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
   roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));

// angle from gyrometer
   //roll =  (0.98*(roll+(gyroX)*dt/1000.0f) +0.02*(accelX))*57.3;
   //pitch = (0.98*(pitch + (gyroY)*dt/1000.0f)+0.02*(accelY))*57.3;


// quaternions

  Q_dot[0] = -0.5* ( (gyroX*Q_pre[1]) +(gyroY*Q_pre[2]) + (Q_pre[3]*gyroZ));
  Q_dot[1] =  0.5* ( (gyroX*Q_pre[0]) +(gyroZ*Q_pre[2]) - (Q_pre[3]*gyroY));
  Q_dot[2] =  0.5* ( (gyroY*Q_pre[0]) -(gyroZ*Q_pre[1]) + (Q_pre[3]*gyroX));
  Q_dot[3] =  0.5* ( (gyroZ*Q_pre[0]) +(gyroY*Q_pre[1]) - (Q_pre[2]*gyroX));

Q[0] =Q_pre[0]+ (Q_dot[0]*dt/1000.0);
Q_pre[0] = Q[0];
Q[1] =Q_pre[1]+ (Q_dot[1]*dt/1000.0);
Q_pre[1] = Q[1];
Q[2] =Q_pre[2]+ (Q_dot[2]*dt/1000.0);
Q_pre[2] = Q[2];
Q[3] =Q_pre[3]+ (Q_dot[3]*dt/1000.0);
Q_pre[3] = Q[3];

double n = (sqrt((Q[0]*Q[0]) + (Q[1]*Q[1]) + (Q[2]*Q[2]) + (Q[3]*Q[3])));
float Q0 = Q[0]/n;
float Q1 = Q[1]/n;
float Q2 = Q[2]/n;
float Q3 = Q[3]/n;

phi_quat = atan2 (2*((Q0*Q1)+(Q2*Q3)), ((0.5f-Q1*Q1-Q2*Q2)));
float theta_quat = asin( 2*((Q0*Q2)- (Q1*Q3)));


phi_quat = (0.98*(phi_quat+gyroX*dt/1000.0f) +0.02*(accelX))*57.3;
theta_quat=(0.98*(theta_quat+gyroY*dt/1000.0f) +0.02*(accelY))*57.3;

  
int filtered_roll = 0.99*(roll+roll*dt/1000.0f)+0.01*(phi_quat);
 
int filtered_pitch = 0.99*(pitch+pitch*dt/1000.0f)+0.01*(theta_quat);


//Serial.println(String(filtered_roll)+','+   String(filtered_pitch));
return pitch*57.3 ;

}
