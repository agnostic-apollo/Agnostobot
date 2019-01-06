#ifndef __PINGSERIAL_H__
#define __PINGSERIAL_H__

/*
 * PingSerial.h - Library for interacting with Serial enabled ultrasonic distance modules (currently just US-100).
 * Created by Anthony Toole, 17 March 2016
 * Released under MIT License, see LICENSE file.
 */
#define __STDC_LIMIT_MACROS // need UINT16_MAX
#include <stdint.h>

#include <SoftwareSerial.h>

#if defined (ARDUINO) && ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
  #include <pins_arduino.h>
#endif

#if defined (__AVR__)
  #include <avr/io.h>
  #include <avr/interrupt.h>
#endif

/*
 * Returned by data_available() to indicate which values have been requested previously and are now available
 */
const int US100_DISTANCE_AVAILABLE = 0;
const int US100_TEMPERATURE_AVAILABLE = 1;
const int US100_DATA_NOT_AVAILABLE = 2;

/*
 * Yeah, just because.
 */
#define TRUE (1 == 1)
#define FALSE (!TRUE)

const int US100_HARDWARE_SERIAL = 0;
const int US100_SOFTWARE_SERIAL = 1;

//these values can be sent to the constructor if you dont want the distance value capped if distance return is out of range
const uint16_t DEFAULT_MIN_MM_DISTANCE = 0;
const uint16_t DEFAULT_MAX_MM_DISTANCE = UINT16_MAX;

/*
 * If you want debug enabled, define PS_ENABLE_DEBUG in your sketch
 * If you can't use Serial (eg. on Leonardo, where you want Serial1) then
 * you've got to edit this library header file to change this.
 *
 * If anyone has neater ideas, answers on a postcard, please.
 */
//#define PS_ENABLE_DEBUG
#ifdef PS_ENABLE_DEBUG
#define DBG_PRINT(args) {Serial.print(args);}
#define DBG_PRINTLN(args) {Serial.println(args);}
#else
#define DBG_PRINT(args) {}
#define DBG_PRINTLN(args) {}
#endif

class PingSerial {
  public:
 
      /*
       * If the sensor is connected to a hardware serial port then pass that to the constructor, else
       * pass in the pin details and a SoftwareSerial port will be created.
       *
       * Max distance sensor can handle is 5m=5000mm, so a uint16 is plenty (max 65,536mm).
       */
      PingSerial();
      PingSerial(HardwareSerial& serialport, uint16_t min_mm_distance = 0, uint16_t max_mm_distance = 500);
      PingSerial(SoftwareSerial& serialport, uint16_t min_mm_distance = 0, uint16_t max_mm_distance = 500);
      PingSerial(int param1, int param2, int serial_type, uint16_t min_mm_distance = 0, uint16_t max_mm_distance = 500);

      // begin() must be called from setup loop, primarily to open and set the baud rate of the serial conncetion
      bool begin(void);

      // Reads data from serial connection to US-100.
      // Returns value with US100_DISTANCE or US100_TEMPERATURE flags set indicating which values are available
      // (not necessarily just received, the flags are cleared only when the values are retrieved). 
      int data_available(void);

      // Get the stored values, if data_available() indicates a value is available.  Returns zero on error (which could be a valid temperature).
      uint16_t get_distance(void);
      uint16_t get_temperature(void);

      // Send request to US-100 for distance or temperature.  This may not be sent immediately, but will be queued if required.
      // (it won't be sent if another request is pending, nor if a request has only just been received as US-100 seems to like a brief pause)
      void request_distance(void);
      void request_temperature(void);

      // Prints some useful internal state, see DBG_PRINT for details on how to enable this.
      void display_debugging(bool clear=FALSE);

      //returns true if waiting for distance data or if the request is queued
      bool is_distance_requested(void);
      //returns true if waiting for temperature data or if the request is queued
      bool is_temperature_requested(void);


  private:
      void Init(uint16_t min_mm_distance, uint16_t max_mm_distance);

      // Wrappers to call underlying function on hardware or software serial object
      int read(uint8_t count); // adds 'count' argument; if not 1 then all read bytes will be discarded
      size_t write(uint8_t b);

      // One of these will be valid.
      HardwareSerial *_hw_serial = NULL;
      SoftwareSerial *_sw_serial = NULL;


      // Internal state, mostly for debugging purposes (see display_debugging())
      unsigned long _op_started = 0;
      uint16_t      _max_op_duration_ms = 0;
      uint16_t      _op_timeout_ms = 0;
      uint16_t      _timeout_count = 0;

      // Stored values
      uint16_t _distance = 0; // Distance is always positive, max (255 * 256 + 255) = 65535
      uint16_t _temperature = 0; // Temperature returned is (byte) - 45 (so ranges from -45 to 256-45)

      // If values are out of range then distance returned will capped to these value, initializing to 0 or UINT16_MAX respectively but will be overwriten by contructor
      uint16_t _min_mm_distance = DEFAULT_MIN_MM_DISTANCE;
      uint16_t _max_mm_distance = DEFAULT_MAX_MM_DISTANCE;

      // Indicates if a request has been sent to US-100, and neither response received or timed out
      bool _distance_pending = FALSE;
      bool _temperature_pending = FALSE;

      // Indicates if a value has been received from US-100.  Cleared once read.
      bool _distance_avail = FALSE;
      bool _temperature_avail = FALSE;

      // Indicates if a request was made while an existing operation is in progress
      bool _distance_request_queued = FALSE;
      bool _temperature_request_queued = FALSE;
};
#endif
