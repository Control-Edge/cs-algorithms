#include "CLineCruise.h"

#include <math.h>

double line_position_cyclic(const uint8_t *sensors, int count,
                            uint8_t *line_detected, LinePositionState *state)
{
    const double center = (count - 1) / 2.0;
    double sum = 0.0;
    int active = 0;

    for (int i = 0; i < count; i++) {
        if (sensors[i]) {
            sum += i - center;
            active++;
        }
    }

    *line_detected = active > 0;
    if (!active) {
        /* Line lost: keep pointing towards the side it was last seen on */
        return state->last_position >= 0.0 ? 1.0 : -1.0;
    }

    state->last_position = (sum / active) / center;
    return state->last_position;
}

double line_cruise_cyclic(const LineCruiseConfig *config, double line_position,
                          uint8_t line_detected, LineCruiseState *state,
                          double dt)
{
    if (dt <= 0.0) {
        return state->speed;
    }

    /* Curve demand: how hard the car is working to stay on the line */
    double alpha = config->filter_tau > 0.0 ? dt / (config->filter_tau + dt) : 1.0;
    state->filtered_error += alpha * (fabs(line_position) - state->filtered_error);

    /* Map demand linearly from max_speed (on the line) to min_speed */
    double ratio = state->filtered_error / config->error_full_slow;
    if (ratio > 1.0) {
        ratio = 1.0;
    }
    double target = config->max_speed - ratio * (config->max_speed - config->min_speed);
    if (!line_detected) {
        target = config->min_speed;
    }

    /* Brake fast into curves, accelerate gently out of them */
    double step = target - state->speed;
    double max_up = config->accel_rate * dt;
    double max_down = config->decel_rate * dt;
    if (step > max_up) {
        step = max_up;
    } else if (step < -max_down) {
        step = -max_down;
    }
    state->speed += step;

    return state->speed;
}
