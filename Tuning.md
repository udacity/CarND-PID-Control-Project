./pid -p 0.02 -i 0.01 -d 0.5 -tp 1.0 -ti 0.0 -td 4.0 --throttle 0.8

./pid -p 0.019 -i 0.05 -d 1.2 -tp 1000.0 -ti 0.0 -td 100.0 --throttle 0.6

./pid -p 0.055 -i 0.00001 -d 0.85 -tp 100000.0 -ti 0.0 -td 10000.0 --throttle 0.6

./pid -p 0.055 -i 0.00001 -d 0.85 -tp 100000.0 -ti 0.0 -td 10000.0 --throttle 0.6
  - вот это использовал с speed_factor
# The final version

Two PID controllers were used:

  * a PID controller for a steering angle: P=0.08, I=0.001, D=0.65
  * a PID controller for a throttle (it uses a break to stop a car in a critical sitation): P=1000000, I=0, D=100000

PID parameters were chosen manually according to the following observations:

  * a "huge" value of P parameter causes oscillation
  * a "small" value of P parameter does not allow to correct trajectory and a car gets pulled away from a track
  * the value of I parameter should be substantially smaller than the value of P parameter. I parameter "supresses" system bias.
  * D parameter prevents oscillation, which arises due to the overshoot

In addition, the maximum speed is limited. When a car moves too fast the controller does not have enough time to correct the trajectory.

# Note

You can redefine PID parameters, maximum speed and maximum throttle using command line parameters. See `pid -h`.
