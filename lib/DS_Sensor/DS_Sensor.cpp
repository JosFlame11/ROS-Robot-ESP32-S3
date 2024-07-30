#include "DS_Sensor.h"

DS_Sensor::DS_Sensor(int pin) {
  _pin = pin;
  pinMode(_pin, INPUT_PULLDOWN);
}

float DS_Sensor::getDistance() {
  float voltage = readVoltage();
  float distance = 12.08 * pow(voltage, -1.058);
  return distance;
}

float DS_Sensor::readVoltage() {
  int sensorValue = analogRead(_pin);
  float voltage = sensorValue * (3.3 / 4095);
  return voltage;
}
