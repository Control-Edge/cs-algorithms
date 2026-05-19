# First Order C Algorithm

This algorithm implements a first-order object which can be used to model simple systems in control theory. It is defined by the following differential equation:

```
dx/dt = (u - x) / tau
```
Where:
- `x` is the state of the system
- `u` is the input to the system
- `tau` is the time constant of the system

The algorithm can be used to simulate the behavior of a first-order system in response to various inputs, and can be useful for testing control algorithms and understanding system dynamics.