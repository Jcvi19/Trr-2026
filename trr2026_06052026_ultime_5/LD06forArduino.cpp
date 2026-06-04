#include "LD06forArduino.h"

static std::vector<char> tmpChars;

void LD06forArduino::Init(int rxPin) {
  Serial2.begin(230400, SERIAL_8N1, rxPin);
}

void LD06forArduino::read_lidar_data() {

  if (!Serial2.available()) return;

  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == 0x54 && tmpChars.empty()) {
      tmpChars.push_back(c);
      continue;
    }

    if (!tmpChars.empty()) {
      tmpChars.push_back(c);

      if (tmpChars.size() == 2 && tmpChars[1] != 0x2C) {
        tmpChars.clear();
        return;
      }

      if (tmpChars.size() >= TOTAL_DATA_BYTE) {
        calc_lidar_data(tmpChars);
        tmpChars.clear();
        return;
      }
    }
  }
}

void LD06forArduino::calc_lidar_data(std::vector<char>& values) {

  data_length = values[1] & 0x1F;
  Speed = float(values[3] << 8 | values[2]) / 100.0;
  FSA   = float(values[5] << 8 | values[4]) / 100.0;
  LSA   = float(values[values.size() - 4] << 8 | values[values.size() - 5]) / 100.0;

  angle_step = (LSA >= FSA)
               ? (LSA - FSA) / (data_length - 1)
               : (LSA + 360 - FSA) / (data_length - 1);

  angles.clear();
  distances.clear();
  confidences.clear();

  for (int i = 0; i < data_length; i++) {
    float angle = FSA + i * angle_step;
    if (angle >= 360) angle -= 360;

    int index = 8 + i * 3;

    distances.push_back(values[index - 1] << 8 | values[index - 2]);
    confidences.push_back(values[index]);
    angles.push_back(angle);
  }
}