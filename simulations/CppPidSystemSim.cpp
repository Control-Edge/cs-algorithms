#include "PIDController.hpp"
#include "FirstOrder.hpp"

#include <cppmodel/Simulation.h>

class ClosedLoopSimulation : public CppModelBase::Simulation
{
    double previousTime = 0.0;
    double plantOutput = 0.0; // Initial output of the plant

    PIDController *pid;
    FirstOrderPlant *plant;

public:
    ClosedLoopSimulation(std::string name, double duration, double dt)
    {
        this->name = name;
        this->time = duration;
        this->stepSize = dt;

        double Kp = 2.7;
        double Ki = 2.6;
        double Kd = 0.1;
        double tau = 1.0;

        pid = new PIDController(Kp, Ki, Kd);
        plant = new FirstOrderPlant(tau);
    }

    void RunCyclic(double simulationTime) override
    {
        double dt = simulationTime - previousTime;
        previousTime = simulationTime;

        
        
        double simulationSetpoint = inputs.GetSafe("Setpoint", 1.0);


        double error = simulationSetpoint - plantOutput;

        inputs["Setpoint"] = simulationSetpoint;
        inputs["Error"] = error;

        double controlSignal = pid->update(error, dt);
        plantOutput = plant->update(controlSignal, dt);

        // outputs["ControlOutput"] = controlSignal;
        outputs["PlantOutput"] = plantOutput;
        outputs["Error"] = error;

        double settlingTime = 2.0;
        double settlingTolerance = 0.02;
        uint8_t isSettled = (simulationTime >= settlingTime);
        uint8_t isValid = (isSettled) ? (fabs(error) < settlingTolerance) : 1;
        outputs["CppModel.StepResult"] = isValid;
    }
};

int main()
{
    ClosedLoopSimulation simulation("CppPIDSystemSim", 20., 0.01);

    simulation.Simulate();

    return simulation.GetSimulationResult();
}
