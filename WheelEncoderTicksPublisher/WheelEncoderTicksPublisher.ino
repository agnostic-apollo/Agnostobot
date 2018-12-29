/* 
 WheelEncoderTicksPublisher - Implements a ROS node for publishing ticks of two quadature motor encoders as ros messages
Uses "Encoder Library" for reading encoder ticks 
http://www.pjrc.com/teensy/td_libs_Encoder.html
https://github.com/PaulStoffregen/Encoder
*/

// Needed on Leonardo to force use of USB serial.
//#define USE_USBCON

#include "Encoder.h"

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability


// Pins for motor encoders
//only first pin of each encoder is currently attached to interrupt capable pins 
const int leftEncoderPin_A = 2;
const int leftEncoderPin_B = 4;
const int rightEncoderPin_A = 3;
const int rightEncoderPin_B = 5;

ros::NodeHandle_<ArduinoHardware, 6, 0, 0, 150> nh;

std_msgs::Int16 leftEncoderMsg;
ros::Publisher leftEncoderPublisher("left_encoder", &leftEncoderMsg);

std_msgs::Int16 rightEncoderMsg;
ros::Publisher  rightEncoderPublisher("right_encoder", &rightEncoderMsg);

long leftEncoderTicks = 0;
long rightEncoderTicks = 0;


// The delay between updates in milliseconds
int updateDelay=15;

bool PUBLISH_ENCODER_DATA=false;
bool SERIAL_PRINT_DATA=true;

Encoder encoderLeft(leftEncoderPin_A, leftEncoderPin_B);
Encoder encoderRight(rightEncoderPin_A, rightEncoderPin_B);

void setup() {

  if(PUBLISH_ENCODER_DATA){
    nh.initNode();
  
    nh.advertise(leftEncoderPublisher);
    nh.advertise(rightEncoderPublisher);
  
    // Wait until the node has initialized before getting parameters.
    while(!nh.connected()) {
      nh.spinOnce();
    }
  }
  
  if(SERIAL_PRINT_DATA){
    Serial.begin(57600);
  }

}

// Every loop, publish the encoder and wheel rates.
void loop()
{
  delay(updateDelay);

  long newLeftEncoderTicks, newRightEncoderTicks;
  
  newLeftEncoderTicks = encoderLeft.read();
  newRightEncoderTicks = encoderRight.read();
  
  if (newLeftEncoderTicks != leftEncoderTicks || newRightEncoderTicks != rightEncoderTicks) {
    
    if(PUBLISH_ENCODER_DATA){
      leftEncoderMsg.data = (int) newLeftEncoderTicks;
      rightEncoderMsg.data = (int) newRightEncoderTicks;
      leftEncoderPublisher.publish(&leftEncoderMsg);
      rightEncoderPublisher.publish(&rightEncoderMsg);
    }

    if(SERIAL_PRINT_DATA){
      Serial.print("Left = ");
      Serial.print(newLeftEncoderTicks);
      Serial.print(", Right = ");
      Serial.print(newRightEncoderTicks);
      Serial.println();
    }
    
    leftEncoderTicks = newLeftEncoderTicks;
    rightEncoderTicks = newRightEncoderTicks;
  }

}
