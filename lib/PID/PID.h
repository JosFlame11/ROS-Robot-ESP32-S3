#ifndef PID_h
#define PID_h

class PID {
  public:
    PID(double* input, double* output, double* setpoint, double kp, double ki, double kd);
    // void setSetpoint(double setpoint);
    void setKonstants(double kp, double ki, double kd);
    bool calculate();

  private:
    double* _input;
    double* _output;
    double* _setpoint;
    double _kp;
    double _ki;
    double _kd;
    unsigned long _lastTime;
    double _integral;
    double _lastInput;
};

#endif