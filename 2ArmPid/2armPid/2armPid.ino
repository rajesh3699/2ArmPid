 #include <Wire.h>
 #include <Servo.h>
#define SerialPort Serial
#include <SparkFunMPU9250-DMP.h>
MPU9250_DMP imu;
double roll , pitch, yaw;
float phi = 0;
float theta = 0;
float psi = 0;
long int pre_ts=0;
float phi_n_1, theta_n_1, psi_n_1;
float phi_dot, theta_dot, psi_dot, phi_quat;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
float Q[4] = {1,0,0,0} ; 
  
float Q_dot [4] ;
float Q_pre [4] = {1,0,0,0}  ;
//Servo intialization

Servo Servo1;
Servo Servo2;
//Servo Servo3;
int roll_servo;
int pitch_servo;
//int yaw_servo;

//feedback intialization

int feedbackPin = A0;
int feedback;
float voltage;

//RC intialization

const int chA = 8;
const int chB = 9;
const int chC = 10;
const int chD = 11;

int ch1;
int ch2;
int ch3;
int ch4;







void setup()
{
  Servo1.attach(5);
  Servo2.attach(6);
  
 
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
//Wire.begin();
//for (int cal_int = 0; cal_int < 1000 ; cal_int ++){ 
  //read_IMUDATA();
  pre_ts=millis();
}

}
void loop() 
{
    if ( imu.dataReady() )
  {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    printIMUData(millis()-pre_ts);
    pre_ts=millis();
  }
}

void printIMUData(long int dt)
{  
  
  float accelX = imu.calcAccel(imu.ax);
  float accelY = imu.calcAccel(imu.ay);
  float accelZ = imu.calcAccel(imu.az);
  float gyroX = imu.calcGyro(imu.gx)/57.3;
  float gyroY = imu.calcGyro(imu.gy)/57.3;
  float gyroZ = imu.calcGyro(imu.gz)/57.3;
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);

// NORMALIZE ACCELE VALUES
 float naccel = sqrt(accelX*accelX +accelY*accelY+accelZ*accelZ);
   
    accelX =accelX/naccel;
    accelY =accelY/naccel;
    accelZ =accelZ/naccel;

//Normailze mag values
 float   nmag = sqrt(magX*magX + magY*magY + magZ*magZ);
   
    magX =magX/ nmag;
    magY =magY/ nmag;
    magZ =magZ/ nmag;

  //Euler angle from accel
 
   pitch = atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))));
   roll = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))));

  
// yaw from mag
   float Yh = (magY * cos(roll)) + (magZ * sin(roll));
   float Xh = (magX * cos(pitch))+(magY * sin(roll)*sin(pitch)) + (magZ * cos(roll) * sin(pitch));

    yaw =  atan2(Yh, Xh);
    

   roll =  (0.98*(roll+(gyroX)*dt/1000.0f) +0.02*(accelX))*57.3;
   pitch = (0.98*(pitch + gyroY*dt/1000.0f)+0.02*(accelY))*57.3;
   yaw =((yaw+ gyroZ*dt/1000.0f)+0.02*(magZ))*57.3;

 

// quaternions

 Q_dot[0] = -0.5* ( (gyroX*Q_pre[1]) +(gyroY*Q_pre[2]) + (Q_pre[3]*gyroZ));
  Q_dot[1] = 0.5* ( (gyroX*Q_pre[0]) +(gyroZ*Q_pre[2]) - (Q_pre[3]*gyroY));
   Q_dot[2] = 0.5* ( (gyroY*Q_pre[0]) -(gyroZ*Q_pre[1]) + (Q_pre[3]*gyroX));
    Q_dot[3] = 0.5* ( (gyroZ*Q_pre[0]) +(gyroY*Q_pre[1]) - (Q_pre[2]*gyroX));

Q[0] =Q_pre[0]+ (Q_dot[0]*dt/1000.0);
Q_pre[0] = Q[0];
Q[1] =Q_pre[1]+ (Q_dot[1]*dt/1000.0);
Q_pre[1] = Q[1];
Q[2] =Q_pre[2]+ (Q_dot[2]*dt/1000.0);
Q_pre[2] = Q[2];
Q[3] =Q_pre[3]+ (Q_dot[3]*dt/1000.0);
Q_pre[3] = Q[3];

 double n = (sqrt((Q[0]*Q[0]) + (Q[1]*Q[1]) + (Q[2]*Q[2]) + (Q[3]*Q[3])));
float Q0 =Q[0] / n;
float Q1 =Q[1] / n;
float Q2 = Q[2]/n;
float Q3 = Q[3]/n;

phi_quat = atan2 (2*((Q0*Q1)+(Q2*Q3)), ((0.5f-Q1*Q1-Q2*Q2)));
float theta_quat = asin( 2*((Q0*Q2)- (Q1*Q3)));
float psi_quat = atan2(2*((Q0*Q3)+(Q1*Q2)),((0.5f-Q2*Q2+Q3*Q3)));
 
phi_quat = (0.98*(phi_quat+gyroX*dt/1000.0f) +0.02*(accelX))*57.3;
theta_quat = (0.98*(theta_quat+gyroY*dt/1000.0f) +0.02*(accelY))*57.3;
psi_quat = (0.98*(psi_quat+gyroZ*dt/1000.0f) +0.02*(magZ))*57.3;
  

 int filtered_roll = 0.99*(roll+roll*dt/1000.0f)+0.01*(phi_quat);
 
 int filtered_pitch = 0.99*(pitch+pitch*dt/1000.0f)+0.01*(theta_quat);

  int filtered_yaw = 0.5*(yaw+yaw*dt/1000.0f)+0.5*(psi_quat);

 Serial.println(String(filtered_roll)+','+String(filtered_pitch));
  
    //switch the stabilize mode and control mode

  ch4 = pulseIn(11, HIGH, 25000);
    
     if
    (ch4 <= 1500)
     
    { 
        // stabilize mode
           
      filtered_roll = map(filtered_roll, -90, 90, 0, 180);
      filtered_roll = constrain(filtered_roll, 0,180 );
                  Servo1.write(filtered_roll );
            
    
               
      filtered_pitch = map(filtered_pitch, -90, 90, 0, 180); 
      filtered_pitch = constrain(filtered_pitch, 0,180 );
           Servo2.write(filtered_pitch);

        
    }
               
                // control mode
else

{
  
  {
      filtered_roll = map(filtered_roll, -90, 90, 0, 180);
     
      filtered_roll = constrain(filtered_roll, 0,180 );
      
       Servo1.write(filtered_roll );
       
   

      filtered_pitch = map(filtered_pitch, -90, 90, 0, 180); 
    
      filtered_pitch = constrain(filtered_pitch, 0,180 );
      Servo2.write(filtered_pitch  );
      
    
    }
   
      
   ch1 = pulseIn (8, HIGH,25000);
   ch1 = map(ch1, 1000,2000,-90,90);
   Servo1.write(ch1+ filtered_roll);

   ch2 = pulseIn (9, HIGH,25000);
   ch2 = map(ch2, 1000,2000,-90,90);
   Servo2.write(ch2 + filtered_pitch);

   

     }
    }
