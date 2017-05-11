#ifndef __ADAFRUIT_SIMPLE_AHRS_H__
#define __ADAFRUIT_SIMPLE_AHRS_H__

#include <Adafruit_Sensor.h>
#include "Adafruit_Sensor_Set.h"

// Simple sensor fusion AHRS using an accelerometer and magnetometer.
class Adafruit_Simple_AHRS
{
public:
  Adafruit_Simple_AHRS(Adafruit_Sensor* accelerometer, Adafruit_Sensor* magnetometer);
  Adafruit_Simple_AHRS(Adafruit_Sensor_Set& sensors);
  Adafruit_Simple_AHRS(Adafruit_Sensor* accelerometer, Adafruit_Sensor* gyro, bool mock);
  bool getOrientation(sensors_vec_t* orientation);
  bool getRollPitchYaw(sensors_vec_t* orientation);

private:
  Adafruit_Sensor* _accel;
  Adafruit_Sensor* _mag;
  Adafruit_Sensor* _gyro;
};

#endif
