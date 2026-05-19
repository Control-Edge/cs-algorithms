#pragma once

class FirstOrderPlant
{
private:
    double tau_;
    double output_;

public:
    FirstOrderPlant(double tau);
    double update(double input, double dt);
};
