
# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

Please click on this image to see video of PID controller driving the car for two laps.
[![Two Laps](https://img.youtube.com/vi/-x4BrGwbHZg/0.jpg)](https://youtu.be/-x4BrGwbHZg)

---

## Reflections

#### Build Issues with uWS Versions
* Windows build required changes and uWS version available is v0.14.2 which is source incompatible with v0.13.  ** NB Reviewers: please use v0.14+ of uWS **
* After initial tests, produced a build on Ubuntu as well using uWS v0.13; however on the Ubuntu hosted in VirtualBox on Windows 10, the simulator runs painfully slow and results are extremely unstable.
* Decided to test and submit final version from Windows 10 **with uWS v0.14.2**
* I acknowledge the information provided in help forums and slack channels to get the initial version of project compiling, and also for "restart" code snippet that helps with Twidle.

#### Susceptibility to Simulator's scree size, and graphics quality
* Different screen size and graphics quality result in different frame rates.
* The configuration of computer running the simulator will also impact frame rates.
* Due to the PID controller actually not having the delta time component, the change of frame rates result in different values of the error coefficients.  Hence the values used after tuning on my PC are not easily reproducible on other PCs.  
* Even for recording the video, I had to reduce throttle.

#### Criteria: Describe the effect each of the P, I, D components had in your implementation.

###### Effect of the Proportional ("P") component

Throttle = 0.3

|Low Value|High Value|
|:--------|:---------|
|P=0, I=0, D=0|P=10, I=0, D=0|
|Car goes straight rather than track the road.|Even with a very small tracking error, car starts oscillating due to overshoots.|
|[![Low P](https://img.youtube.com/vi/3Lx1D1UoQgk/0.jpg)](https://youtu.be/3Lx1D1UoQgk)|[![High P](https://img.youtube.com/vi/fE9Dau7d2xI/0.jpg)](https://youtu.be/fE9Dau7d2xI)|

###### Effect of the Integral ("I") component

Throttle = 0.3

|Negative Value|Positive Value|
|:--------|:---------|
|P=0, I=-1, D=0|P=0, I=+1, D=0|
|Car tends to turn right - heavy bias, as -1 is actually pretty high rate of accumulation.|Car tends to turn left - heavy bias, as +1 is actually pretty high rate of accumulation.|
|[![Negative I](https://img.youtube.com/vi/cCANlt4RZ3g/0.jpg)](https://youtu.be/cCANlt4RZ3g)|[![Positive I](https://img.youtube.com/vi/axG822NZoDA/0.jpg)](https://youtu.be/axG822NZoDA)|

###### Effect of the Derivative ("D") component

Throttle = 0.3

|Low Value|Mid Value|High Value|
|:--------|:--------|:---------|
|P=0.3, I=0, D=1|P=0.3, I=0, D=5|P=0.3, I=0, D=15|
|A tuned value of P, but with low D, the car still oscillates.|**With tuned P and tuned value of D, the car follows the track smoothly.**|With tuned P and high value of D, the car follows track but the steering oscillates wildly, which can lead to tyre burnout and reduced speed.|
|[![Low D](https://img.youtube.com/vi/n6KHQxXLn8M/0.jpg)](https://youtu.be/n6KHQxXLn8M)|[![Mid D](https://img.youtube.com/vi/Scp3Gb_MGCE/0.jpg)](https://youtu.be/Scp3Gb_MGCE)|[![High D](https://img.youtube.com/vi/hh584c1j3BE/0.jpg)](https://youtu.be/hh584c1j3BE)|

###### Effect of the vehicle speed on P,I,D components
We take the same values (P=0.3, D=5) that seemed well tuned for throttle value of 0.3 and increase the throttle to 0.7.

The car now begins to oscillate and eventually crash.  Given same steering angle, the car turn more at higher speeds than at lower speeds.

[![High Speed](https://img.youtube.com/vi/CG4opFlzU6w/0.jpg)](https://youtu.be/CG4opFlzU6w)

###### Overall observations
For this exercise: kD >> kP >> kI

#### Criteria: Describe how the final hyperparameters were chosen.

I have an implementation of the Twidle algorithm in the PID controller.  Since the CTE from simulator is dependent on PID output from intermediate stages, simple mean-squared-root error minimization didn't work well. I had tried training it with a couple of variations of error function,
- maximise the number of frames (iterations or steps) run before car went off track
- maxmiise the combination of number of iterations and standard deviation of CTE (steps/stdev)
Both of these did not converge, and it may have been my choice of initial dP values.

Eventually I resorted to manual tuning following suggestions in slack forum that  getting parameters for 50 mph drive was easy to do manually.  For throttle value of 0.3:
- use a low value of kP, keeping kI and kD at 0, and increase (double) kP until car is somewhat able to follow to tracks and starts oscillating.
- next use a low value of kD, keeping kI at 0, and increase kD until it dampens the oscillations to make the car progress on more of the track.
- repeat steps above until car is able to follow turns safely.

I then increased throttle values to .6 and tuned kP and kD down a bit from the values for throttle .3, as at higher speeds, same steering angle will cause car to turn more than at lower speed.  I kept the kI value at 0, as I have tweaked the PID to ignore very small CTE.

###### Adaptive throttle control and hard limits on error to apply breaks
With higher throttle values, I started observing some instabilities.  My implementation has optional parameters to specify maximum CTE beyond which negative throttle is applied.  

However, I had much better success with using a different mechanism for adapting the throttle.  I tried using another PID controller, but tuning it was becoming difficult given the time.  Hence I have used "moving" variance of the last 40 CTE values and a threshold to adapt the throttle value.  When  variance of CTE is high, throttle is reduced. At low variance (below 0.01), the throttle applied boosted!  The size of "moving" variance history (`cteWindowSize`) is also configurable on command line.

    throttle_value = throttle_specified + (0.01 - CTE_variance)

NB: When I ported the code to Ubuntu VM hosted on my PC, these results didn't work.  That when I also discovered the susceptibility to frame rates of simulator.

I have hard coded the paramenters for my Windows PC's configuration, but these parameters can be overridden on command line.  My values are:

|Param|Value|
|-----|-----|
|P|0.0880|
|I|0.000|
|D|2.875|
|-----|-----|
|`throttle_specified`|1.0|
|`cteWindowSize`|40|

#### Command line usage

Usage: pid [[Kp] [Ki] [Kd] [throttle]] [a [cteWindowSize]] [b [highCte]] [t [[dKp] [dKi] [dKd] [tolerance]]
Eg:
    # no arguments, uses built-in defaults 
    pid 

    # Kp=.3, Ki=0, Kd=.4, throttle=.4; enable adaptive throttle, with window size of 50; enable break on CTE higher than .7
    pid .3 0 5 .4 a 50 b .7

    # Kp=0, Ki=0, Kd=0, throttle=.4; twiddle starting with dKp=.1, dKi=0, dKd=5, tolerance=.2
    pid 0 0 0 .4 t .1 0 5 .2


#### Further possible improvements
- Allow for time duration between telemetry readings (delta t), and make the controller frame rate independent.
- Scale P,I,D sensitivity with speed (lower the values for higher throttle specified)
- Better error function for Twidle, such that the parameters can be learnt automatically

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.14, but the master branch will probably work just fine **NB: orginal project specified v0.13 which is difficult to install on Windows.**
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.14.2 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.14.2.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

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

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
