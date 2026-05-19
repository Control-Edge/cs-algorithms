#include "PIDController.hpp"

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd), integral_(0.0), prev_error_(0.0) {}


double PIDController::update(double input, double dt) {
    // Proportional term
    double p_term = kp_ * input;

    
    // Integral term
    integral_ += input * dt;
    double i_term = ki_ * integral_;
    
    if (dt == 0.0) {
        return p_term + i_term; // Avoid division by zero for derivative term
    }

    // Derivative term
    double derivative = (input - prev_error_) / dt;
    double d_term = kd_ * derivative;

    prev_error_ = input;
    double output = p_term + i_term + d_term;

    return output;
}
