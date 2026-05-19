#include "FirstOrder.hpp"

FirstOrderPlant::FirstOrderPlant(double tau)
    : tau_(tau), output_(0.0) {}

double FirstOrderPlant::update(double input, double dt) {
    // Discrete first-order dynamics: y[k] = y[k-1] + (dt/τ) * (u - y[k-1])
    output_ = output_ + (dt / tau_) * (input - output_);
    return output_;
}