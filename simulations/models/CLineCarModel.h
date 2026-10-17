/**
 * Line-following robot car - plant model (C Header)
 *
 * Minimal differential-drive car driving along a line made of constant-
 * curvature segments. Only the behavior the line-follower controller needs:
 *   - each wheel follows its PWM command with a first-order lag,
 *   - the car's pose is tracked relative to the line (distance along it,
 *     lateral offset, heading error),
 *   - a row of digital IR sensors ahead of the axle reports which sensors
 *     are over the line.
 * No slip, inertia beyond the motor lag, or sensor noise.
 *
 * Conventions: lateral offset and line position are positive to the LEFT,
 * curvature is positive for a LEFT turn. Units: m, s, rad. PWM is -1..1.
 *
 * battery_factor scales motor speed (1.0 = full battery) and is set by the
 * simulation; the controller cannot measure it (no wheel encoders).
 */

#ifndef CLINE_CAR_MODEL_H
#define CLINE_CAR_MODEL_H

#include <stdint.h>

#define LINE_CAR_SENSOR_COUNT 5
#define LINE_CAR_MAX_SEGMENTS 16

/** One piece of track: a straight (curvature 0) or a circular arc */
typedef struct {
    double length;    /* m */
    double curvature; /* 1/m, positive = left turn */
} LineCarSegment;

typedef struct {
    double wheelbase;     /* m, distance between the wheels */
    double max_speed;     /* m/s, wheel speed at PWM 1.0 on a full battery */
    double motor_tau;     /* s, wheel speed response time constant */
    double sensor_ahead;  /* m, sensor array distance ahead of the axle */
    double sensor_pitch;  /* m, spacing between adjacent sensors */
    double line_width;    /* m */
    LineCarSegment track[LINE_CAR_MAX_SEGMENTS];
    int segment_count;
} LineCarModelConfig;

/** Commanded by the controller */
typedef struct {
    double pwm_left;  /* -1..1 */
    double pwm_right; /* -1..1 */
} LineCarModelActuators;

/** Reported back to the controller */
typedef struct {
    uint8_t ir[LINE_CAR_SENSOR_COUNT]; /* index 0 = rightmost, 1 = over the line */
} LineCarModelSensors;

typedef struct {
    LineCarModelConfig config;
    LineCarModelActuators actuators;
    LineCarModelSensors sensors;

    double battery_factor; /* injected by the simulation, 0..1 */

    double speed_left;     /* m/s */
    double speed_right;    /* m/s */
    double distance;       /* m along the line */
    double lateral_offset; /* m, axle center from the line, positive = left */
    double heading_error;  /* rad, car heading minus line heading */
    double line_offset_at_sensors; /* m, line position under the array, positive = left */
    int segment;           /* index of the current track segment */
    uint8_t finished;      /* 1 once the end of the track is reached */
} LineCarModel;

/** Put the car at the start of the line, centered and aligned, at rest */
void line_car_model_reset(LineCarModel *model);

/** Total track length, m */
double line_car_model_track_length(const LineCarModel *model);

/**
 * Advance the model by one step.
 *
 * @param model  Model, actuators read and sensors written in place
 * @param dt     Time step in seconds
 */
void line_car_model_cyclic(LineCarModel *model, double dt);

#endif /* CLINE_CAR_MODEL_H */
