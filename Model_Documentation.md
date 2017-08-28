Title:	    CarND	Project	1, Path Planning  
Ver:	    1.0	(first	submission)	 
Author:	    Tamas Panyi  
Date:	    2017-08-28  
System:     Ubuntu	16.04.2	LTS  
Editors:    Visual Studio Code  

# Self Driving Car ND - Term 3 Project 1: Path Planning
## Files
As part of this project I modified / created the following files:
```bash
src/main.cpp
src/TG.cpp (and .h)
src/PID.cpp (and .h)
src/Aux.cpp (and .h)
CMakeLists.txt
```
Please make sure that you use the submitted `CMakeList.txt`file.  
## Reflection

[//]: # (Image References)

[image1]: Screen1.png "Screenshot "
[image2]: PID.jpg "Screenshot "

### Vehicle model
I used a kinematic vehicle model which is good enough for this simulation application. In a real application one has to have reasonable control over at least tire forces and lateral and longitudinal accelerations in the vehicle (kinetic model).  

### Basic Functional Break Down Structure and Implementation
I used and approach of separation of functionality to some extent, in order to keep the scope of each class manageable and comprehensive, hence I created a few new class and files:
* Communication (main)
* Path Generation (TG)
* Speed Control (PID)
* Mathematical and misc help functions (Aux)

#### Communication and Main Computation Cycle
See file `src/main.cpp`:
This is the file responsible for the main loop and the communication between the simulator and the planner application.

#### Principles of Path Generation
See File `src/TG.cpp (and .h)`
This is the Trajectory Generator. I utilized a spline to generate a drivable path for the vehicle.  
The advantage of this is that it can handle the transient curvatures with the necessary accuracy, and it can accommodate inflection points where a path curve changes heading direction. The application is straight forward based on the provided `spline.h` source.  

First I created way-points:
* Check if there is an existing route to append new points to
* If yes: append to it, if no: use the cars current position and heading to generate two points  
* Make three waypoints, appending the new ones to the ones above (5 to 50 points), along the `route`
* Transform the way-points (`ptsx` and `ptsy`) to the car's frame of reference.

Based on the way-points generate a driving path. The density of points in the driving path will determine the speed.
* Generate a spline like so:
    ```cpp
    tk::spline s;
    s.set_points(ptsx, ptsy);
    ```
* create drive points for the simulator. The points are set apart by the reference speed.
    ```cpp
    double N = (target_dist/(0.02*ref_vel/2.24)); // update cycle and mph to m/s ratio
    double x_point = x_add_on + (target_x)/N; //generated x points for the spline
    double y_point = s(x_point); // s is the spline
    ```
* create and return the path points the simulator will follow:
    ```cpp
    points = make_pair(next_x_vals, next_y_vals);
    ```

#### Cost Calculation
The cost calculation is implemented in the `TG` class. It will try to find the best lanes without causing an accident or inflicting too much uncomfortable lane changes to its passengers. See function `TG::getBestLane`, that returns the best lane to drive in.
The moves with potentially catastrophic outcome have weight factors that ensure that the respective cost will always be too high to be outweighed by low speed etc. As the least important point is that I prefer to stay in the middle lane which leaves the most options open (In Europe one would need to stay in the rightmost possible lane due to the 'keep right' rule, and it is not lawful to overtake a vehicle from the right).

#### Speed Control
See file `PID.cpp`
I've utilized a simplified PD Controller to implement a useable speed control for the  vehicle. The code is encapsulated tin the PID class. The hyper parameters are manually and rather roughly tuned to give a satisfactory result.

```cpp
double init_Kp = -0.5;
double init_Kd = -0.8;
double init_Ki = -0;
```
The control parameter is the distance to the vehicle ahead (if any), which is controlled indirectly. See my _artwork_ for reference.  
![alt text][image2]

The `Kp` corresponds to th distance of the car in front of us, and `Kd` basically corresponds to the relative speed difference between us and the car ahead. `Ki` is set to zero as it is meaningless to use the integral term in this setting.  
The time offset between two cars would be a better control quantity however this would complicate things further and I decided to leave it as a possible improvement prospect. (You would want to keep smaller distance when the traffic flows slowly).
The PID controller is anyways a simplistic approach to this problem. A vehicle predictive control model, as in _Term 2_ would yield much better result.

#### Auxiliary functions
Some math and other help functions are organized in the `Aux` class. These functions are defined as `static`, in order to avoid even more object states which may be difficult to track (more like in functional programing).

### Known issues
* In the beginning the car tries to change to lane 0, as I guess all the variables are initialized to 0, hence the `lane`too. I tried to mitigate this issue. The car stays in it's lane, but tries to change.
* If another car comes in front of us relatively late, our car will not detect it until the other car's CoG ("d-point") has made it's way inside our lane. (One might get unlucky and drive into that car)

### Possibilities for improvement
* Use time as control quantity for better "tailing" control.
* Implement a full blown predictive control for the speed regulation
* Implement functional programming principles for the whole project, avoiding uncontrolled object states.
* Refactor the codebase so that the spline and final drive point generation is encapsulated in a separate class.
* Store the car state in a separate struct (i.e. simulator message packs) in order to make the argument list of functions shorter.

## Result
See an example of an incident free long drive.
![alt text][image1]
