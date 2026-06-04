#ifndef LD06FORARDUINO_H
#define LD06FORARDUINO_H

#include <Arduino.h>
#include <vector>

#define TOTAL_DATA_BYTE 47   // trame LD06 standard

class LD06forArduino {
public:
  void Init(int rxPin);
  void read_lidar_data();

  std::vector<float> angles;
  std::vector<int> distances;
  std::vector<char> confidences;

private:
  void calc_lidar_data(std::vector<char>& values);

  char start_byte;
  int data_length;
  float Speed;
  float FSA;
  float LSA;
  float angle_step;
  int time_stamp;
  int CS;
};

#endif