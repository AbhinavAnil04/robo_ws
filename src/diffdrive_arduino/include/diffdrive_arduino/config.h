#ifndef DIFFDRIVE_ARDUINO_CONFIG_H
#define DIFFDRIVE_ARDUINO_CONFIG_H

#include <string>

struct Config
{
  std::string left_wheel_name = "rear_left_wheel_joint";
  std::string right_wheel_name = "rear_right_wheel_joint";
  float loop_rate = 30;  // Hz
  std::string device = "/dev/ttyUSB0";
  int baud_rate = 57600;
  int timeout = 1000;   // ms
  int enc_counts_per_rev = 3436;
};

#endif // DIFFDRIVE_ARDUINO_CONFIG_H
