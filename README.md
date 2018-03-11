# Udacity SDCND Project 11: Path Planning
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)



The goal of this project was to safely navigate a vehicle around on a 3-lane highway with other traffic. The vehicle had to plan its path around the track, overtake slower cars and stay within given limits for speed, acceleration and jerk. The project uses the [term 3 simulator](https://github.com/udacity/self-driving-car-sim/releases). 

The approach used was a simple decision tree that let the vehicle choose the fastest lane while avoiding other vehicles. Doing this the vehicle was able to drive around the track without crashes for hours.

Project video: (Youtube Link)
[![Project track](https://github.com/stefancyliax/CarND-Path-Planning-Project/raw/master/pic/PP.gif)](https://www.youtube.com/watch?v=GHt7KMsX5p8)

## Approach

Besides data about the track and the position of the vehicle on the track, the simulator also provided information about other vehicles on the track. With this I could implement a behavioral planning using a simple decision tree. The behavioral plan was then used to generate a trajectory and the trajectory was realized with a simple motion controller.

The behavioral planner works like this:
If the lane of the vehicle is free, just keep the maximum allowed speed.
If there is a vehicle in the lane that is slower and closer then 40m, check the other lanes.
```cpp
if ((vehicle_in_front[1] < max_vel) && vehicle_in_front[0] < 40)
```
The other lanes are checked for if there is a vehicle not closer than 40m in front and the lane clear for a lane change?
```cpp
if (((other_lane_vehicle[0] > 40) && laneClear(i, car_s, sensor_fusion))
```
If the questions is true, the lane is marked as a possible for a lane change. Based on the current lane and the possible other lanes, a decision is derived like this: 

| Lanes free / current lane | 0           | 1                       | 2           |
|---------------------------|-------------|-------------------------|-------------|
|                        -- | Keep lane   | Keep lane               | Keep lane   |
|                         0 | --          | Change to 0             | Keep lane*2 |
|                         1 | Change to 1 | --                      | Change to 1 |
|                         2 | Keep lane*2 | Change to 2             | --          |
|                      0; 1 | --          | --                      | Change to 1 |
|                      0; 2 | --          | Decide based on speed*1 | --          |
|                      1; 2 | Change to 1 | --                      | --          |

*1: If the vehicle is on the middle lane and both the left and right lanes are free, the decision is done based on the slowest vehicle on front of the car. The lane which has a higher slowest speed is chosen.

*2: I also implemented a solution where the vehicle would change from the outer lane to the other outer lane if there was a gap. This worked perfectly but caused high jerk. It was implemented like this:

```cpp
// else if (lane == 0 && lane_free[2] && !lane_free[1] && laneClear(1, car_s, sensor_fusion)) {lane = 1;}
// else if (lane == 2 && lane_free[0] && !lane_free[1] && laneClear(1, car_s, sensor_fusion)) {lane = 1;}
```

This decision tree was implemented like this:

```cpp
if (lane_free[1])
{
    lane = 1;
}
else if (lane_free[0] && lane_free[2]) // both left and right lane is free. Decide on overall lane speed.
{
    if (slowLaneSpeed(0, car_s, sensor_fusion) > slowLaneSpeed(2, car_s, sensor_fusion))
    {
        lane = 0;
    }
    else
    {
        lane = 2;
    }
}
else if (lane == 1 && lane_free[0] && !lane_free[2])
{
    lane = 0;
}
else if (lane == 1 && !lane_free[0] && lane_free[2])
{
    lane = 2;
}
else // in all other cases, there is no free lane
{
    // adjust speed based on current lane speed
}
}
```

# Reflection
I did not enjoy this project. First I planned to implement a proper jerk-limited controller for the vehicle but this wasn't told in the lessons. After some research on my own, I implemented a jerk-limited controller using s = s0 + v0*t + 1/2 * a0 * t^2 + 1/6 * j * t^3. But in the end it proved to be very difficult to include it in the given code that is rather fragile. Also handling the curves and lane changes was a challenge. In the end I reverted the controller back to the code provided in the walkthrough video. 
Second the path planning works but isn't very advanced. I wanted to do more but time limitations didn't allow for it. 
This is probably a project I return to at a later point in time.


---

## Dependencies (from original README)

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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```