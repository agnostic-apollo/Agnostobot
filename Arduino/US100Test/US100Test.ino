/*
 * PingSerialExample.ino - Example Arduino script for using PingSerial library with US-100 ultrasonic distance meter
 * Created by Anthony Toole, 17 March 2016
 * Released under MIT License, see LICENSE file.
 * This example script is a basic example of how to use PingSerial library to talk to a US-100 distance module.
 * In this example the US-100 is connected to the hardware serial port Serial, and a SoftwareSerial port is used for debugging (if required).
 */
#include <PingSerial.h>
#include <SoftwareSerial.h> // Arduino issue: a library can't include other libraries, the sketch has to do this. Avoid by not using Arduino!

int front_ultrasonic_tx_pin = 18;
int front_ultrasonic_rx_pin = 19;
int left__ultrasonic_tx_pin = 16;
int left_ultrasonic_rx_pin = 17;
int right_ultrasonic_tx_pin = 14;
int right_ultrasonic_rx_pin = 15;
int back_ultrasonic_tx_pin = 25;
int back_ultrasonic_rx_pin = 13;


PingSerial front_ultrasonic(1, 0, US100_HARDWARE_SERIAL, 0, 2100);
PingSerial left_ultrasonic(2, 0, US100_HARDWARE_SERIAL, 0, 2100);
PingSerial right_ultrasonic(3, 0, US100_HARDWARE_SERIAL, 0, 2100);
PingSerial back_ultrasonic(back_ultrasonic_rx_pin, back_ultrasonic_tx_pin, US100_SOFTWARE_SERIAL, 0, 2100);

bool ping_enabled = true;
unsigned int pingSpeed = 2000; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer = 0;     // Holds the next ping time.
int data_available;
int32_t distance;
int32_t temperature;

void setup() {
  front_ultrasonic.begin();
  left_ultrasonic.begin();
  right_ultrasonic.begin();
  back_ultrasonic.begin();
  Serial.begin(9600);
}

void loop() {

  /*
   * Note: none of this code is blocking (no calls to delay() for example)
   * so your Arduino can do other things while measurements are being made.
   * Quite useful for any real world examples!
   */
  distance = get_distance(front_ultrasonic);
  if (distance != -1 && distance!= -2) {
      Serial.print("\nFront Distance: ");
      Serial.println(distance);
  }

  temperature = get_temperature(front_ultrasonic);
  if (temperature != -1 && temperature!= -2) {
      Serial.print("Front Temperature: ");
      Serial.println(temperature);
  }


  distance = get_distance(left_ultrasonic);
  if (distance != -1 && distance!= -2) {
      Serial.print("Left Distance: ");
      Serial.println(distance);
  }

  temperature = get_temperature(left_ultrasonic);
  if (temperature != -1 && temperature!= -2) {
      Serial.print("Left Temperature: ");
      Serial.println(temperature);
  }


  distance = get_distance(right_ultrasonic);
  if (distance != -1 && distance!= -2) {
      Serial.print("Right Distance: ");
      Serial.println(distance);
  }

  temperature = get_temperature(right_ultrasonic);
  if (temperature != -1 && temperature!= -2) {      
      Serial.print("Right Temperature: ");
      Serial.println(temperature);
  }


  distance = get_distance(back_ultrasonic);
  if (distance != -1 && distance!= -2) {
      Serial.print("Back Distance: ");
      Serial.println(distance);
  }

  temperature = get_temperature(back_ultrasonic);
  if (temperature != -1 && temperature!= -2) {      
      Serial.print("Back Temperature: ");
      Serial.println(temperature);
  }

  if (ping_enabled && (millis() >= pingTimer)) {   // pingSpeed milliseconds since last ping, do another ping.
      pingTimer = millis() + pingSpeed;      // Set the next ping time.
      
      //front_ultrasonic.request_temperature();
      front_ultrasonic.request_distance();
      left_ultrasonic.request_distance();
      right_ultrasonic.request_distance();
      back_ultrasonic.request_distance();
  }

  delay(1000);

}


// Get Distance
int32_t get_distance(PingSerial range_sensor)
{
  if(!range_sensor.is_distance_requested()) //if distance has not been requested  
    return -2;
  else
  {
    int data_available = range_sensor.data_available();

    if (data_available == US100_DISTANCE_AVAILABLE) //if distance has been requested and distance data is available return the distance
      return range_sensor.get_distance();
    else //if distance has been requested but distance data is not available return -1
      return -1;
  }
}

// Get Temperature
int32_t get_temperature(PingSerial range_sensor)
{
  if(!range_sensor.is_temperature_requested()) //if temperature has not been requested  
    return -2;
  else
  {
    int data_available = range_sensor.data_available();

    if (data_available == US100_TEMPERATURE_AVAILABLE)  //if temperature has been requested and temperature data is available return the temperature
      return range_sensor.get_temperature();
    else //if temperature has been requested but temperature data is not available return -1
      return -1;
  }
}