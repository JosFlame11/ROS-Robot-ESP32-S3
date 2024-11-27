#ifndef STATE_FEEDBACK_CONTROLLER_H
#define STATE_FEEDBACK_CONTROLLER_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

class StateFeedbackController {
public:
    StateFeedbackController(
        BLA::Matrix<3, 3, double>& A, BLA::Matrix<3, 1, double>& B, BLA::Matrix<1, 3, double>& C,
        double D, BLA::Matrix<1, 3, double>& Kx, double Ki, BLA::Matrix<3, 1, double>& L,
        double samplingTime, double* measuredOutput, double* referenceInput, double* controlOutput);

    void update();

private:
    // Matrices for state-space equations
    BLA::Matrix<3, 3, double> A_;
    BLA::Matrix<3, 1, double> B_;
    BLA::Matrix<1, 3, double> C_;
    double D_;
    BLA::Matrix<1, 3, double> Kx_;
    double Ki_;
    BLA::Matrix<3, 1, double> L_;

    // State variables
    BLA::Matrix<3, 1, double> x_hat_; // Estimated state
    BLA::Matrix<3, 1, double> x_dot_; // State derivative
    double xi_;                // Integral state
    double y_hat_;             // Estimated output

    // Control input/output pointers
    double* measuredOutput_;
    double* referenceInput_;
    double* controlOutput_;

    // Sampling time
    double Ts_;
};

#endif // STATE_FEEDBACK_CONTROLLER_H
