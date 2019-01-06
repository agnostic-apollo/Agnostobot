#ifndef RANGE_SENSOR_H
#define RANGE_SENSOR_H


#ifdef USE_SERIAL_US100
#include <limits.h>

#define N_SERIAL_US100_RANGE_SENSORS 4

int serial_us100_param1 [N_SERIAL_US100_RANGE_SENSORS] = { 1, 2, 3, 13};
int serial_us100_param2 [N_SERIAL_US100_RANGE_SENSORS] = { 0, 0, 0, 25};
int serial_us100_serial_type [N_SERIAL_US100_RANGE_SENSORS] = { US100_HARDWARE_SERIAL, US100_HARDWARE_SERIAL, US100_HARDWARE_SERIAL, US100_SOFTWARE_SERIAL };
uint16_t serial_us100_min_mm_distance [N_SERIAL_US100_RANGE_SENSORS] = { 0, 0, 0, 0 };
uint16_t serial_us100_max_mm_distance [N_SERIAL_US100_RANGE_SENSORS] = { 2100, 2100, 2100, 2100 };

class SERIAL_US100
{
  public:
    SERIAL_US100();
    bool initRangeSensor(
        int param1,
        int param2,
        int serial_type,
        uint16_t min_mm_distance,
        uint16_t max_mm_distance);
    void request_distance();
    void request_temperature();
    int32_t get_distance();
    int32_t get_temperature();

  private:
    PingSerial range_sensor;
};

SERIAL_US100 serial_us100_range_sensors [N_SERIAL_US100_RANGE_SENSORS];
#endif

#endif
