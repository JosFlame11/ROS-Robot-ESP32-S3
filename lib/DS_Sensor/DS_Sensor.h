#ifndef DS_SENRSOR_H
#define DS_SENRSOR_H

#include <Arduino.h>

class DS_Sensor {
  public:
    DS_Sensor(int pin);
    float getDistance();

  private:
    int _pin;
    float readVoltage();
};

#endif
