#include "cppmodel/CModel.h"
#include "CPid.h"
#include "CFirstOrder.h"

#include <math.h>

double plantOutput = 0.0; // Initial output of the plant

double simulationTime = 20.0; // Total simulation time in seconds
double timeStep = 0.01; // Simulation time step in seconds

static CModelSimulation_ts *simulation;

static PIDState pidState = {0};
static FirstOrderState firstOrderState = {0};

void RunCyclic(CModelSimulation_ts *ctx, unsigned long currentTime)
{
    static unsigned long lastTime = 0;
    double dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    double Kp = CppModel_getParameterF64(ctx, "Kp", 2.7);
    double Ki = CppModel_getParameterF64(ctx, "Ki", 2.6);
    double Kd = CppModel_getParameterF64(ctx, "Kd", 0.1);
    double tau = CppModel_getParameterF64(ctx, "Tau", 1.0);
    double settlingTime = CppModel_getParameterF64(ctx, "SettlingTime", 2.0);
    double settlingTolerance = CppModel_getParameterF64(ctx, "SettlingTolerance", 0.02);

    double simulationSetpoint = CppModel_getInputF64(ctx, "Setpoint", 1.0);
    
    double error = simulationSetpoint - plantOutput;

    double controlSignal = pid_cyclic(Kp, Ki, Kd, error, &pidState, dt);
    plantOutput = first_order_cyclic(tau, dt, controlSignal, &firstOrderState);

    // CppModel_setOutputF64(ctx, "ControlOutput", controlSignal);
    CppModel_setOutputF64(ctx, "PlantOutput", plantOutput);
    CppModel_setOutputF64(ctx, "Error", error);

    uint8_t isSettled = (currentTime / 1000.0) >= settlingTime;
    uint8_t isWithinTolerance = fabs(error) < settlingTolerance;
    CppModel_setOutputU8(ctx, "CppModel.StepResult", isSettled ? !isWithinTolerance ? 0 : 1 : 1);
}

int main(void)
{
    simulation = CppModel_create("CPidSystemSim", simulationTime * 1000, timeStep * 1000);
    CppModel_setRunStepFunction(simulation, RunCyclic);

    CppModel_Simulate(simulation);

    return CppModel_getSimulationResult(simulation);
}
