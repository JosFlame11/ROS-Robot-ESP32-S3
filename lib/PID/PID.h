#ifndef PID_h
#define PID_h

class PID {
  public:
    PID(double* input, double* output, double* setpoint, double kp, double ki, double kd);
    // Esta funcion ya no es necesaria
    // void setSetpoint(double setpoint);
    // Por si se quieren cambiar las constantens, para que sea PID adaptativo
    void setKonstants(double kp, double ki, double kd);
    // Esta funcion se debe llamar cada ves que se quiera actualizar el output
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