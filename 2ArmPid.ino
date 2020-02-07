#include <Servo.h>//Using servo library to control ESC

Servo esc; //Creating a servo class with name as esc
 int val=1300; //Creating a variable val

void setup()

{

esc.attach(9); //Specify the esc signal pin,Here as D8

esc.writeMicroseconds(1000); //initialize the signal to 1000

Serial.begin(9600);

}

void loop()

{
esc.writeMicroseconds(1300); //using val as the signal to esc

}
