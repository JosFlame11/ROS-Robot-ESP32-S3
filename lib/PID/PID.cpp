#include "Arduino.h"
#include "PID.h"

PID::PID(double* input, double* output, double* setpoint, double kp, double ki, double kd) {
  _input = input;
  _output = output;
  _setpoint = setpoint;
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _lastTime = 0;
  _integral = 0;
  _lastInput = *_input;
}

// void PID::setSetpoint(double setpoint) {
//   _setpoint = setpoint;
// }

void PID::setKonstants(double kp, double ki, double kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

bool PID::calculate() {
  unsigned long now = micros();
  double dt = (now - _lastTime) / 1e6;
  double error = *_setpoint - *_input;
  _integral += error * dt;
  double derivative = (_lastInput - *_input) / dt;
  *_output = (_kp * error + _ki * _integral + _kd * derivative);
  _lastTime = now;
  _lastInput = *_input;
  return true;
}