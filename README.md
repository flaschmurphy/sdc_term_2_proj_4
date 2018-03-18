# Udacity Self Driving Car Nanodegree

## Term 2, Project 4 -- PID Controller

### Name: Ciaran Murphy

### Date: 18-Mar-2018

---

## Introduction

The objective of this project is to implement a PID controller to control a car
as it drives around a track in the Udacity simulator.

In this write up, I will cover the points of the project
[rubric](https://review.udacity.com/#!/rubrics/824/view). 


## Compiling

* **Requirement**: *Your code should compile.*

The code can be compiled with no errors or warnings using the provided cmake
file in the project root directory. To compile, move to the root dir of the
project and execute:

```shell
mkdir build && cd build
cmake ..
make

```

This will create a file called `pid` which is the main executable of the
program.


## The PID procedure follows what was taught in the lessons

The PID controller works by calculating a magnitude factor for use in actuating
the steering control. The proportional–integral–derivative controller (PID
controller) uses three terms to calculate a value to be fed back to the
steering control system. The first term is the easiest to understand - it is
simply a value that is proportional to the current error. This error is
referred to as the "Cross Track Error" (CTE) and it is received by the PID
controller system as it's input signal. For each value received in this signal,
the proportional part of the PID value is set to equal the CTE and then scaled
by a factor known as `Kp`. However, if this was the only step taken, the
vehicle would fail to stabilize since at the time the CTE was driven to zero,
the car would be traveling at an angle to the desired path. As a result the CTE
would immediately increase again, causing a new proportional correction and an
endless oscillation through and back over the optimal trajectory would continue
indefinitely. 

Therefore the second step is to take into consideration the rate of change of
the CTE in the calculation. In signal processing, the rate of change (i.e.
derivative) of a signal can be computed simply by subtracting subsequent signal
values from each other. Hence, for every signal input, the previous value is
subtracted from the current value and the result is scaled by a factor known as
`Kd`. This derivative factor is then added to the proportional value. 

There is one more step which is included to cater for any constant factor drift
that may be inherent in the vehicle. Such a drift could result for example from
inaccuracies in the steering control or (in the real world) some misalignment
of the front wheels of the car. It would be reasonable to assume that this
drift would be relatively constant and therefore when viewed as a graph over
time could be thought of as a plot whose total area to the horizontal axis would be
increasing at a constant factor. The area under a graph is also known as the
integral of the graph and that is our third element. The total CTE is therefore
added over all input signals and the result is again scaled by a factor known
as `Ki` which is then added to the other two values to create the final output.

Thus, the calculation for the controller can be represented mathematically as
follows (taken from [here](https://en.wikipedia.org/wiki/PID_controller))

![pid formula](./resources/pid_formula.svg)

The code to implement this is visible in the file `PID.cpp` as follows:

```cpp
void PID::UpdateError(double cte) {

    // Update the differential error
    d_error = cte - p_error;

    // Update the proportional error
    p_error = cte;

    // Update the integral error
    i_error += cte;

}

double PID::TotalError() {
    return Kp*p_error + Kd*d_error + Ki*i_error;
}

```

## Describe the effects of the PID conponents

My expectation was that the P component would dominate the control, that the
D component would be less significant and that the I component would have
little or no impact. What I found was that to my surprise I needed to set the
D component to be several multiples of P in order to make it work at all.
I also did not manage to obtain a very stable result, although the car does
make it around the track at what I think is a reasonable speed. 

Having spend a long time trying to find the best values for `Kp`, `Kd` and `Ki`
I realized that I would make better progress by also controlling the speed
based on the current CTE. This made a big difference as it allows the car to
take more time to correct itself in case it's starting to lose control
completely. I also implemented a speed limit and set it to 30km/h. If the car
starts slowing down, the effect of the steering has less and less effect (just
like a real car) so I also implemented a multiplication of the steering value
in cases where the throttle was not being applied. Therefore the steps are:

* If the car is at the speed limit, set the throttle to zero
* If the CTE is above a pre-defined threshold (set to 0.8), set the throttle to
  zero and double the steering value
* If the car is within the CTE threshold and below the speed limit, increase
  the throttle by 10%

The code section for this is visible in `main.cpp`:

```cpp
double cte_threshold = 0.8;
double max_speed = 30.0;

if (std::abs(cte) > cte_threshold)
{
  pid.throttle = 0.0;
  steer_value *= 2.0;
}

else if (speed < max_speed)
  pid.throttle += 0.1;

else if (speed >= max_speed)
  pid.throttle = 0.0;

```

To see how the car performed, [here](https://youtu.be/HVTuAa3DvPE) is a YouTube video of a lap. Note that
unfortunately YouTube has some issues encoding the video and so after about 30
secs the playback starts to stop and start again for some reason.

<iframe width="560" height="315"
src="https://www.youtube.com/embed/HVTuAa3DvPE" frameborder="0"
allow="autoplay; encrypted-media" allowfullscreen></iframe>
