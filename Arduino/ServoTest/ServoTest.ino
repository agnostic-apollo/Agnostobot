
#include <Servo.h>

int front_pan_servo_pin = 2;
int front_tilt_servo_pin = 3;
int left_pan_servo_pin = 4;
int left_tilt_servo_pin = 5;
int right_pan_servo_pin = 6;
int right_tilt_servo_pin = 7;
int back_pan_servo_pin = 8;
int back_tilt_servo_pin = 9;

Servo front_pan_servo;
Servo front_tilt_servo;
Servo left_pan_servo;
Servo left_tilt_servo;
Servo right_pan_servo;
Servo right_tilt_servo;
Servo back_pan_servo;
Servo back_tilt_servo;
         
int angle = 0;    
 
void setup() 
{ 
  front_pan_servo.attach(front_pan_servo_pin);
  front_tilt_servo.attach(front_tilt_servo_pin);
  left_pan_servo.attach(left_pan_servo_pin);
  left_tilt_servo.attach(left_tilt_servo_pin);
  right_pan_servo.attach(right_pan_servo_pin);
  right_tilt_servo.attach(right_tilt_servo_pin);
  back_pan_servo.attach(back_pan_servo_pin);
  back_tilt_servo.attach(back_tilt_servo_pin);

} 
  
void loop() 
{

  for(angle = 45; angle < 135; angle += 15)
  {                                  
    front_pan_servo.write(angle);
    front_tilt_servo.write(90);
    left_pan_servo.write(angle);
    left_tilt_servo.write(90);
    right_pan_servo.write(angle);
    right_tilt_servo.write(90);
    back_pan_servo.write(angle);
    back_tilt_servo.write(90);

    delay(200);                       
  } 

  delay(1000);

  for(angle = 45; angle < 135; angle += 15)
  {                                  
    front_pan_servo.write(90);
    front_tilt_servo.write(angle);
    left_pan_servo.write(90);
    left_tilt_servo.write(angle);
    right_pan_servo.write(90);
    right_tilt_servo.write(angle);
    back_pan_servo.write(90);
    back_tilt_servo.write(angle);

    delay(200);                       
  } 

}
