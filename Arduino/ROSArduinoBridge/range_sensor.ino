/***************************************************************
   Range Sensor - by Hassan Khalid

 *************************************************************/

#ifdef USE_RANGE_SENSOR

  #ifdef USE_SERIAL_US100

  // Constructor
  SERIAL_US100::SERIAL_US100()
  {

  }


  // Init
  bool SERIAL_US100::initRangeSensor(
          int param1,
          int param2,
          int serial_type,
          uint16_t min_mm_distance,
          uint16_t max_mm_distance)
  {
    range_sensor = PingSerial(param1, param2, serial_type, min_mm_distance, max_mm_distance);
    return range_sensor.begin();

  }


  // Request Distance
  void SERIAL_US100::request_distance()
  {
    range_sensor.request_distance();
  }

  // Get Distance
  int32_t SERIAL_US100::get_distance()
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

  // Request Temperature
  void SERIAL_US100::request_temperature()
  {
    range_sensor.request_temperature();
  }

  // Get Temperature
  int32_t SERIAL_US100::get_temperature()
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

  #endif

#endif
