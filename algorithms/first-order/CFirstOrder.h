
#ifndef CFIRST_ORDER_H
#define CFIRST_ORDER_H

/** Persistent state passed into every cyclic call */
typedef struct
{
    double output;
} FirstOrderState;

/**
 * First-order plant cyclic function – call once per control cycle.
 *
 * Discrete approximation: y[k] = y[k-1] + (dt/τ) * (u[k] - y[k-1])
 *
 * @param tau    Time constant in seconds
 * @param dt     Sample time in seconds
 * @param input  Plant input (control signal)
 * @param state  Pointer to persistent state (updated in place)
 * @return       Plant output
 */
double first_order_cyclic(double tau, double dt,
                          double input, FirstOrderState *state);

#endif /* CFIRST_ORDER_H */
