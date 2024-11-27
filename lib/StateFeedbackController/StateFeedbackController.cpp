#include "StateFeedbackController.h"

// Constructor
StateFeedbackController::StateFeedbackController(
    BLA::Matrix<3, 3, double>& A, BLA::Matrix<3, 1, double>& B, BLA::Matrix<1, 3, double>& C,
    double D, BLA::Matrix<1, 3, double>& Kx, double Ki, BLA::Matrix<3, 1, double>& L,
    double samplingTime, double* measuredOutput, double* referenceInput, double* controlOutput)
    : A_(A), B_(B), C_(C), D_(D), Kx_(Kx), Ki_(Ki), L_(L), Ts_(samplingTime),
      x_hat_({0, 0, 0}), x_dot_({0, 0, 0}), xi_(0), y_hat_(0), 
      measuredOutput_(measuredOutput), referenceInput_(referenceInput), controlOutput_(controlOutput) {}

// Update method to process the control loop
void StateFeedbackController::update() {
    // Calculate the error
    double error = *referenceInput_ - *measuredOutput_;

    // Update integral state
    xi_ += error * Ts_;

    // Compute estimated output
    y_hat_ = (C_ * x_hat_)(0, 0) + D_ * (*controlOutput_);

    // Observer update: dx_hat = A_ * x_hat_ + B_ * u_ + L_ * (measuredOutput - y_hat_)
    BLA::Matrix<3, 1, double> observerCorrection = L_ * (*measuredOutput_ - y_hat_);
    x_dot_ = A_ * x_hat_ + B_ * (*controlOutput_) + observerCorrection;

    // Update state using Euler integration
    x_hat_ = x_hat_ + x_dot_ * Ts_;

    // Compute control input: u_ = -Kx_ * x_hat_ - Ki_ * xi_
    *controlOutput_ = -(Kx_ * x_hat_)(0, 0) - Ki_ * xi_;
}
