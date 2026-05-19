#include "CPid.h"

double pid_cyclic(double kp, double ki, double kd, double input,
                  PIDState* state, double dt) {

    if (dt == 0.0) {
        return state->output; // Avoid division by zero
    }

    /* Proportional term */
    double p_term = kp * input;

    /* Integral term */
    state->integral += input * dt;
    double i_term = ki * state->integral;

    /* Derivative term */
    double derivative = (input - state->prev_error) / dt;
    double d_term = kd * derivative;

    state->prev_error = input;
    state->output = p_term + i_term + d_term;

    return state->output;
}

