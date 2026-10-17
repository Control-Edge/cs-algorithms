/**
 * Line-follower cruise control simulation
 *
 * A small differential-drive car (CLineCarModel) follows a line through
 * straights and curves. The controller runs every 10 ms:
 *   - line position from 5 digital IR sensors (CLineCruise),
 *   - base speed from the cruise control (CLineCruise),
 *   - steering as a PD controller on the line position (CPid).
 *
 * Scenarios:
 *   1. Cruise control on, battery sagging 100% -> 80%: the car must never
 *      lose the line, must be clearly slower in the tight curve than on the
 *      straights, and must finish the track in time.
 *   2. Cruise control off, constant full speed, full battery: the car must
 *      lose the line - proves the track actually needs the cruise control.
 */

#include "cppmodel/CModel.h"
#include "CLineCruise.h"
#include "CPid.h"
#include "models/CLineCarModel.h"

#include <stdint.h>

#define STEP_MS 10UL      /* simulation step = controller period */
#define PLANT_SUBSTEPS 10 /* car model integrated at 1 ms */
#define SCENARIO_MS 12000UL
#define SCENARIO_COUNT 2
#define TIGHT_CURVE_SEGMENT 3

static LineCarModel car;
static LineCruiseState cruiseState;
static LinePositionState positionState;
static PIDState steerState;

static double linePosition;
static uint8_t lineDetected;
static double baseSpeed;

static uint8_t lineLost;
static double peakStraightSpeed;
static double peakTightCurveSpeed;
static unsigned long finishTime_ms;

static double clamp(double value, double low, double high)
{
    return value < low ? low : (value > high ? high : value);
}

static const LineCarSegment track[] = {
    {1.0, 0.0},          /* straight */
    {0.7854, 2.0},       /* gentle left, R = 0.5 m, 90 deg */
    {0.8, 0.0},          /* straight */
    {0.4712, 6.6667},    /* tight left, R = 0.15 m, 180 deg */
    {0.5, 0.0},          /* straight */
    {0.4712, 3.3333},    /* S-bend left, R = 0.3 m, 90 deg */
    {0.4712, -3.3333},   /* S-bend right, R = 0.3 m, 90 deg */
    {1.0, 0.0},          /* straight to finish */
};

static void InitScenario(void)
{
    car.config.wheelbase = 0.12;
    car.config.max_speed = 1.0;
    car.config.motor_tau = 0.1;
    car.config.sensor_ahead = 0.06;
    car.config.sensor_pitch = 0.015;
    car.config.line_width = 0.019;
    car.config.segment_count = sizeof(track) / sizeof(track[0]);
    for (int i = 0; i < car.config.segment_count; i++) {
        car.config.track[i] = track[i];
    }
    car.actuators.pwm_left = 0.0;
    car.actuators.pwm_right = 0.0;
    line_car_model_reset(&car);

    cruiseState = (LineCruiseState){0};
    positionState = (LinePositionState){0};
    steerState = (PIDState){0};
    linePosition = 0.0;
    lineDetected = 1;
    baseSpeed = 0.0;

    lineLost = 0;
    peakStraightSpeed = 0.0;
    peakTightCurveSpeed = 0.0;
    finishTime_ms = 0;
}

static void RunCyclic(CModelSimulation_ts *ctx, unsigned long currentTime)
{
    const int scenario = (int)(currentTime / SCENARIO_MS) + 1;
    const unsigned long localTime = currentTime % SCENARIO_MS;
    const uint8_t cruiseEnabled = scenario == 1;

    if (scenario > SCENARIO_COUNT) {
        /* CppModel also runs the step at exactly the end time */
        CppModel_setOutputU8(ctx, "CppModel.StepResult", 1);
        return;
    }
    if (localTime == 0) {
        InitScenario();
    }

    LineCruiseConfig cruise = {
        .max_speed = CppModel_getParameterF64(ctx, "MaxSpeed", 1.0),
        .min_speed = CppModel_getParameterF64(ctx, "MinSpeed", 0.35),
        .error_full_slow = CppModel_getParameterF64(ctx, "ErrorFullSlow", 0.6),
        .filter_tau = CppModel_getParameterF64(ctx, "FilterTau", 0.03),
        .accel_rate = CppModel_getParameterF64(ctx, "AccelRate", 1.0),
        .decel_rate = CppModel_getParameterF64(ctx, "DecelRate", 8.0),
    };
    const double kp = CppModel_getParameterF64(ctx, "SteerKp", 0.6);
    const double kd = CppModel_getParameterF64(ctx, "SteerKd", 0.015);
    const double batteryEnd = CppModel_getParameterF64(ctx, "BatteryEnd", 0.8);
    const double timeLimit_s = CppModel_getParameterF64(ctx, "TimeLimit", 11.0);
    const double curveSlowdown = CppModel_getParameterF64(ctx, "CurveSlowdown", 0.8);

    /* Environment: battery sag the controller cannot see */
    car.battery_factor = cruiseEnabled ? 1.0 - (1.0 - batteryEnd) * localTime / (double)SCENARIO_MS : 1.0;

    /* Controller */
    const double dt = STEP_MS / 1000.0;
    linePosition = line_position_cyclic(car.sensors.ir, LINE_CAR_SENSOR_COUNT, &lineDetected, &positionState);
    baseSpeed = cruiseEnabled ? line_cruise_cyclic(&cruise, linePosition, lineDetected, &cruiseState, dt)
                              : cruise.max_speed;
    const double steer = pid_cyclic(kp, 0.0, kd, linePosition, &steerState, dt);

    /* Stop at the finish line, and once the line is lost in scenario 2 */
    const uint8_t stop = car.finished || (!cruiseEnabled && lineLost);
    car.actuators.pwm_left = stop ? 0.0 : clamp(baseSpeed - steer, -1.0, 1.0);
    car.actuators.pwm_right = stop ? 0.0 : clamp(baseSpeed + steer, -1.0, 1.0);

    /* Plant, with bookkeeping for the assertions at every sub-step */
    double carSpeed = 0.0;
    for (int i = 0; i < PLANT_SUBSTEPS; i++) {
        line_car_model_cyclic(&car, dt / PLANT_SUBSTEPS);

        carSpeed = 0.5 * (car.speed_left + car.speed_right);
        uint8_t lineVisible = 0;
        for (int s = 0; s < LINE_CAR_SENSOR_COUNT; s++) {
            lineVisible |= car.sensors.ir[s];
        }
        if (!lineVisible && !car.finished) {
            lineLost = 1;
        }
        if (car.finished && finishTime_ms == 0) {
            finishTime_ms = localTime + STEP_MS;
        }
        if (!car.finished) {
            if (car.config.track[car.segment].curvature == 0.0 && carSpeed > peakStraightSpeed) {
                peakStraightSpeed = carSpeed;
            }
            if (car.segment == TIGHT_CURVE_SEGMENT && carSpeed > peakTightCurveSpeed) {
                peakTightCurveSpeed = carSpeed;
            }
        }
    }

    /* Assertions */
    uint8_t result = 1;
    if (cruiseEnabled) {
        if (lineLost) {
            result = 0;
        }
        if (localTime / 1000.0 > timeLimit_s && !car.finished) {
            result = 0;
        }
        if (car.finished && peakTightCurveSpeed > curveSlowdown * peakStraightSpeed) {
            result = 0;
        }
    } else if (localTime == SCENARIO_MS - STEP_MS && !lineLost) {
        result = 0;
    }

    CppModel_setOutputU8(ctx, "CppModel.StepResult", result);
    CppModel_setOutputU8(ctx, "internalStepNumber", (uint8_t)scenario);
    CppModel_setOutputF64(ctx, "Distance [m]", car.distance);
    CppModel_setOutputF64(ctx, "LateralOffset [mm]", car.lateral_offset * 1000.0);
    CppModel_setOutputF64(ctx, "LineOffsetAtSensors [mm]", car.line_offset_at_sensors * 1000.0);
    CppModel_setOutputF64(ctx, "LinePosition", linePosition);
    CppModel_setOutputU8(ctx, "LineDetected", lineDetected);
    CppModel_setOutputU8(ctx, "LineLost", lineLost);
    CppModel_setOutputF64(ctx, "BaseSpeed", baseSpeed);
    CppModel_setOutputF64(ctx, "PwmLeft", car.actuators.pwm_left);
    CppModel_setOutputF64(ctx, "PwmRight", car.actuators.pwm_right);
    CppModel_setOutputF64(ctx, "CarSpeed [m/s]", carSpeed);
    CppModel_setOutputF64(ctx, "Battery", car.battery_factor);
    CppModel_setOutputU8(ctx, "Segment", (uint8_t)car.segment);
    CppModel_setOutputU8(ctx, "Finished", car.finished);
    CppModel_setOutputF64(ctx, "FinishTime [s]", finishTime_ms / 1000.0);
}

int main(void)
{
    CModelSimulation_ts *simulation = CppModel_create("CLineCruiseSim", SCENARIO_COUNT * SCENARIO_MS, STEP_MS);
    CppModel_setRunStepFunction(simulation, RunCyclic);

    CppModel_Simulate(simulation);

    return CppModel_getSimulationResult(simulation);
}
