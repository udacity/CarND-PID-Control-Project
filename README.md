# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

## Intro

The goal of this project is to implement a [PID controller](https://en.wikipedia.org/wiki/PID_controller) to drive a car successfully around the [Udacity self-driving car simulator](https://github.com/udacity/self-driving-car-sim).

A PID controller continuously calculates an error value as the difference between a desired point and a measured process variable and applies a correction based on (P)roportional, (I)ntegral, and (D)erivative terms.

## The P, I, D components in the implementation.

In the case of driving a car, the P component provides a steering angle that is proportional to the cross-track error (CTE). The car simulator returns the CTE value, and the P tries to adjust the error by making a counter steer, trying to remain to the center of the road. A high P value is a violent steer, and a low P value is a smoother one.

The I control, is the accumulation of errors (an integral), and changes the steering angle proportional to the integrated value of the CTE. With a high I value, the car will compensate the past errors, oscillating more to reduce the sum of past errors. A low value will produce less oscillations, to reduce the accumulated error slowly.

D is the derivative control, and focuses on the rate of angle change. A high D will make the car less sensitive and will move smoothly to the optimum track. A low D value will make the car swivel more even following the optimum track.

## How the final hyperparameters were chosen?

I have adapted the C++ code to accept argument parameters. If 3 arguments are passed, then the program runs in a "normal" way with the three values as `p,i,d` inputs. For example:

```
./pid 1.1 0.001 4.0
```

If a forth parameter is used, is used to set a maximum number of iterations, and it returns the final error value. For example, this command will launch the same than before, but it will finish after 100 iterations.

```
./pid 1.1 0.001 4.0 100
```

This second version is used from a [python notebook](twiddle.ipynb) to execute the *Twiddle* algorithm.

```
  def run(p, steps=1500):
      out = check_output(["build/pid", str(p[0]), str(p[1]), str(p[2]), str(steps)])
      out = out.decode().replace("4567", "")
      numbers = re.findall(r"[-+]?\d*\.\d+|\d+", out)
      return float(numbers[0])

  def twiddle(tol=0.1, steps=1500):
      p = [0.0, 0.0, 0.0]
      dp = [1.0, 1.0, 1.0]
      best_err = run(p, steps)

      it = 0
      while sum(dp) > tol:
          print("Iteration {}, best_error={}, p=[{:.3f} {:.3f} {:.3f}], dp=[{:.3f} {:.3f} {:.3f}]".format(
                  it, best_err,
                  p[0], p[1], p[2],
                  dp[0], dp[1], dp[2]))

          for i in range(len(p)):
              p[i] += dp[i]
              err = run(p, steps)

              if err < best_err:
                  best_err = err
                  dp[i] *= 1.1
              else:
                  p[i] -= 2 * dp[i]
                  err = run(p, steps)

                  if err < best_err:
                      best_err = err
                      dp[i] *= 1.1
                  else:
                      p[i] += dp[i]
                      dp[i] *= 0.9
          it += 1
      return p
```

To launch the Twiddle algorithm, the simulator must be running and the following python line must be used:

```
  twiddle(tol=0.001, steps=1000)
```

After several tests, my final parameters are (Kp, Ki, Kd) = (0.1, 0.001, 4.0). To execute:

```
./pid 0.1 0.001 4.0
```

## Improvements

Apart of the PID controller for the steering angle, another one is used to control the throttle. This allows the achivement of higher speeds.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

