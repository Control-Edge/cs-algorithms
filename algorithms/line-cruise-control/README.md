# Line-follower cruise control

Speed scheduling for a line-following robot car that has no wheel encoders. The car drives at full power on straights and slows down automatically in curves, where it would otherwise overshoot and lose the line.

Without speed feedback the cruise control is open-loop on motor power (PWM). It uses the line-tracking error as a measure of how hard the car is working to stay on the line: a small error means a straight, a large error means a curve.

```
demand  = lowpass(|line_position|, filter_tau)
target  = max_speed - min(1, demand / error_full_slow) * (max_speed - min_speed)
target  = min_speed                        if the line is lost
speed  += clamp(target - speed, -decel_rate*dt, +accel_rate*dt)
```

The asymmetric rate limit lets the car brake quickly into a curve and accelerate gently out of it.

The folder also contains `line_position_cyclic`, which turns a row of digital reflectance sensors into a line position from -1 to 1. When the line is lost, it reports the side where the line was last seen, so the steering keeps turning back towards it.

Steering is not part of this algorithm. Add a differential correction on top of the base speed, for example a PD controller on the line position (see `simulations/CLineCruiseSim.c`, which uses `CPid`):

```
pwm_left  = base_speed - steer
pwm_right = base_speed + steer
```
