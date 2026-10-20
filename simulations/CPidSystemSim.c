#include "cppmodel/CModel.h"
#include "CPid.h"
#include "CFirstOrder.h"

double plantOutput = 0.0; // Initial output of the plant

double simulationTime = 20.0; // Total simulation time in seconds
double timeStep = 0.01;       // Simulation time step in seconds

static CModelSimulation_ts *simulation;

static PIDState pidState = {0};
static FirstOrderState firstOrderState = {0};

void Init()
{
    pidState = (PIDState){0};
    firstOrderState = (FirstOrderState){0};
}

CMODEL_CYCLIC()
{
    static unsigned long lastTime = 0;
    double dt = (currentTime_ms - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime_ms;
    if (currentTime_ms == 0)
    {
        dt = timeStep; // Use default time step for the first iteration
        Init();
    }

    double Kp = CppModel_getParameterF64(self, "Kp", 2.7);
    double Ki = CppModel_getParameterF64(self, "Ki", 2.6);
    double Kd = CppModel_getParameterF64(self, "Kd", 0.01);
    double tau = CppModel_getParameterF64(self, "Tau", 1.0);
    double settlingTime = CppModel_getParameterF64(self, "SettlingTime", 2.0);
    double settlingTolerance = CppModel_getParameterF64(self, "SettlingTolerance", 0.02);

    double simulationSetpoint = CppModel_getInputF64(self, "Setpoint", 1.0);

    double toleranceMin = simulationSetpoint - settlingTolerance;
    double toleranceMax = simulationSetpoint + settlingTolerance;

    double error = simulationSetpoint - plantOutput;

    double controlSignal = pid_cyclic(Kp, Ki, Kd, error, &pidState, dt);
    plantOutput = first_order_cyclic(tau, dt, controlSignal, &firstOrderState);

    // CppModel_setOutputF64(self, "ControlOutput", controlSignal);
    CppModel_setOutputF64(self, "PlantOutput", plantOutput);
    // CppModel_setOutputF64(self, "Error", error);

    uint8_t isSettled = (currentTime_ms / 1000.0) >= settlingTime;
    uint8_t isWithinTolerance = (plantOutput >= toleranceMin) && (plantOutput <= toleranceMax);
    CppModel_setOutputU8(self, "CppModel.StepResult", isSettled ? !isWithinTolerance ? 0 : 1 : 1);

    CppModel_setOutputF64(self, "ToleranceMin", toleranceMin);
    CppModel_setOutputF64(self, "ToleranceMax", toleranceMax);
    CppModel_setOutputF64(self, "Settled", isSettled ? 1.0 : 0.0);
}

CMODEL_SIMULATE("CPidSystemSim", simulationTime * 1000, timeStep * 1000)