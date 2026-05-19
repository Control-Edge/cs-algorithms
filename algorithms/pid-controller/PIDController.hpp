#pragma once

class PIDController
{
private:
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double prev_error_;

public:
    PIDController(double kp, double ki, double kd);
    double update(double input, double dt);
};