/**
 * Line-follower cruise control - C Header
 *
 * Speed scheduling for a line-following robot car without wheel encoders.
 * The car's base motor power (PWM, 0..1) is lowered as the line-tracking
 * error grows - a large error means a curve the car is struggling to hold -
 * and raised again on straights. Decrease and increase are rate-limited
 * separately so the car brakes quickly into a curve and accelerates out of
 * it gently. Steering is not part of this algorithm: add a differential
 * correction (e.g. a PD controller on the line position) on top of the base
 * speed returned here.
 *
 * Line position convention: normalized to -1..1 across the sensor array,
 * positive = line is to the left of the array center.
 *
 * Assumptions:
 *   - Called once per control cycle with a constant-ish dt (e.g. 10 ms).
 *   - Speeds are open-loop PWM fractions, 0..1 (no speed feedback).
 */

#ifndef CLINE_CRUISE_H
#define CLINE_CRUISE_H

#include <stdint.h>

/** Tuning parameters */
typedef struct {
    double max_speed;       /* base PWM on a straight line, 0..1 */
    double min_speed;       /* base PWM in the tightest curve / line lost */
    double error_full_slow; /* |line position| at which min_speed is reached, 0..1 */
    double filter_tau;      /* s, low-pass time constant on |line position| */
    double accel_rate;      /* max base PWM increase per second */
    double decel_rate;      /* max base PWM decrease per second */
} LineCruiseConfig;

/** Persistent state passed into every cyclic call */
typedef struct {
    double filtered_error; /* low-pass filtered |line position| */
    double speed;          /* current base PWM output */
} LineCruiseState;

/** Persistent state of the line position estimator */
typedef struct {
    double last_position; /* last valid position, used to steer back when lost */
} LinePositionState;

/**
 * Line position from a row of digital reflectance sensors.
 *
 * Weighted average of the active sensors' indices, normalized to -1..1.
 * When no sensor sees the line, returns +/-1 towards the side the line was
 * last seen on, so the steering keeps turning back to it.
 *
 * @param sensors        Sensor readings, index 0 = rightmost, 1 = line seen
 * @param count          Number of sensors (>= 2)
 * @param line_detected  Set to 1 if any sensor sees the line, else 0
 * @param state          Pointer to persistent state (updated in place)
 * @return               Line position, -1..1 (positive = line to the left)
 */
double line_position_cyclic(const uint8_t *sensors, int count,
                            uint8_t *line_detected, LinePositionState *state);

/**
 * Cruise control cyclic function - call once per control cycle.
 *
 * @param config         Tuning parameters
 * @param line_position  Line position, -1..1
 * @param line_detected  1 if the line is currently seen; 0 forces min_speed
 * @param state          Pointer to persistent state (updated in place)
 * @param dt             Time step in seconds
 * @return               Base PWM for both motors, min_speed..max_speed
 */
double line_cruise_cyclic(const LineCruiseConfig *config, double line_position,
                          uint8_t line_detected, LineCruiseState *state,
                          double dt);

#endif /* CLINE_CRUISE_H */
