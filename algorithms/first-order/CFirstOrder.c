
#include "CFirstOrder.h"

double first_order_cyclic(double tau, double dt,
                          double input, FirstOrderState *state)
{
    /* Discrete first-order dynamics: y[k] = y[k-1] + (dt/τ) * (u - y[k-1]) */
    state->output = state->output + (dt / tau) * (input - state->output);
    return state->output;
}
