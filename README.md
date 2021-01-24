# Orient_Tbot_Polaris
This project aims to autonomously orient the Turtlebot3 to Polaris (the North Star) from any given location on Earth.

## Problem statement

The Polaris star is aligned with True North from anywhere on Earth. However, Magnetic North and True North are not the same almost anywhere on Earth. This is caused by the fact that the Earth's magnetic poles are not aligned with its axis of rotation, and drift on a yearly basis. Therefore, to compute True North heading, knowing the Magnetic Declination is necessary in addition to magnetic heading. By adding the Magnetic Declination to the Magnetic North heading, we can obtain the True North heading. The Magnetic Declination can be computed using the World Magnetic Model 2020-2025, and is a function of latitude, longitude and altitude above sea level.

The Turtlebot3 can execute velocity commands in x and y and can rotate on itself around the z axis. It is equiped with an IMU incorporating a magnetometer, which allows it to compute its absolute heading with respect to magnetic North, in the ENU frame of reference. To achieve its objective, this ROS implementation performs the following tasks:
- Given a lattitude, longitude and altitude, calculate the True North heading, corresponding to the North Pole.
- Orient the turtlebot to the True North heading, observe data from the `/odom` topic and publish angular velocity commands to the `/cmd_vel` topic.
- Simulate the turtlebot in the Gazebo simulator in order to validate the implementation.

## Setup

### Dependencies

This package requires ROS Melodic and Gazebo 9 and was tested on Ubuntu 18.04. It also requires the turtlebot3 packages, which can be installed with

```sudo apt install ros-melodic-turtlebot3-*```

### Installation

First clone the repository into your catkin workspace src folder. From your catkin workspace, build and source the package using

```catkin build turtle_polaris```
```source devel/setup.bash```

You will also need to setup an environment variable to define which Turtlebot3 model you want to use (this package works for all). For example, to set up the burger model as your default model in your bashrc, do:

```echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc```
```source ~/.bashrc```

## Execution

The package is designed to run directly on the Turtlebot3 or in parallel with a simulated model in Gazebo.

To run on the Turtlebot3, launch:
```roslaunch turtle_polaris turtlebot3_controller.launch```

To launch the simulation environment, launch:
```roslaunch turtle_polaris turtlebot3_sim.launch```

Alternatively, you can launch both at the same time using:
```roslaunch turtle_polaris turtle_to_polaris_main.launch```

`turtlebot3_controller.launch` and `turtle_to_polaris_main.launch` accept 3 launch arguments: `lat`, `lon`, and `alt`. They represent latitude (deg_dec), longitude (deg_dec) and altitude above sea level (m). They are set to 0.0 by default.

`turtlebot3_sim.launch` accepts 1 launch argument: `yaw` (rad). It represents the initial yaw of the robot and is set to 0.0 by default (due East).

## Implementation details
The controller is implemented in the `turtle_polaris_node.py` file and incorporates methods to obtain the current yaw from the `/odom` topic and send an appropriate yaw_rate command through the `/cmd_vel` topic. Additionally, the node depends on 3 other files:
- **`utils.py`**: contains the Utils class. The class initialises the publisher, subscriber and loads the ros parameters.
- **`PI_controller.py`**: contains the PI class. The class is a PI controller that allows to compute appropriate yaw_rate commands for the robot. Although, in this case, it seems that a simple unit P controller gives the best results. Therefore the I gain is set to 0.
- **`lib/mag_declination/declination_wrapper.py`**: A wrapper file that contains functions to determine the True North Heading from computing the Magnetic Declination. The function computing the magnetic declination given a latitude, longitude and altitude is implemented in C, and is stored in the compiled library file `declination.so`. It is based on the World Magnetic Model (WMW) for the years 2020-2025 and on open-source software released by NOAA (the National Oceanic and Atmospheric Administration). As far as the author is aware, this makes it the only publicly available library to compute True North in ROS (using Magnetic North, longitude, latitude and altitude as inputs).

`declination.so` is created from the `declination.c` file using coefficients and functions made publicly available by NOAA in the `WMM.COF` and `GeomagnetismLibrary.c` files. If required, `declination.so` can be recompiled from the `src/turtle_polaris/src/lib/mag_declination` folder using:

```cc -fPIC -shared -o declination_test.so declination.c GeomagnetismLibrary.c```

## Test cases

Several tests cases are designed to check that the robot attains the correct orientation from different locations on Earth. The test is considered successfull if the current yaw rate and the error between the True North heading and the current yaw are both under 0.1 rad/s and rad, respectively. The test automatically fails if it does not reach a successful state within 10 seconds.

5 test cases are implemented and described in the table below. The True North Heading is defined as the heading of True North in the ODOM ENU reference frame, that represents Magnetic North as pi/2. It is obtained by first calculating the magnetic declination using the World Magnetic Model at epoch 2021. The declination is then converted to radian and added to the Magnetic North value (pi/2).

| Test | Location                | Comment                                                 | Latitude | Longitude  | Altitude (m) | True North Heading (deg) |
|------|-------------------------|---------------------------------------------------------|----------|------------|--------------|--------------------------|
| 1    | Equator                 | Default                                                 | 0.0      | 0.0        | 0.0          | 85.4945                  |
| 2    | The North Face HQ       | Just a pun                                              | 40.29377 | -121.66497 | 0.0          | 103.7333                 |
| 3    | The Mariana Trench      | Preparing for the amphibious update                     | 11.35    | 142.2      | -10984       | 90.5167                  |
| 4    | The North Pole          | Here any heading works                                  | 90.0     | 0.0        | 0.0          | 95.7333                  |
| 5    | The Magnetic North Pole | Would be impossible in the real world (see Limitations) | 86.83    | 164.07     | 0.0          | -78.3333                 |

## Known limitations
The following limitations are identified in the current implementation
- The Turtlebot3 does not have a GPS, so the latitude, longitude and altitude information need to be inputed manually by the user on launch.
- The Turtlebot3 will not adjust its heading if it is displaced after launch. However this is intentional as estimates of distance travelled would be inacurate without a GPS.
- The computed magnetic declination is only accurate for the year 2021 as the location of the Magnetic North Pole drifts on a yearly basis. However it is possible to update the year by changing a single line in the code. It was not implemented as an autonomous change as it would greatly complexify the testing architecture.
- In the real world, the reading from the magnetometer would become inacurate around the North and South magnetic poles. This is due to the fact that the vertical component of the magnetic field becomes larger than the horizontal component in those areas, which are known as Blackout Zones. Both test case 4 and 5 are located in Blackout Zones and therefore would not be practical in the real world.

## Future work
In the future, the following improvements could be achieved:

- Validate the implementation on a real Turtlebot3 robot, as it was only ran in the Gazebo simulator so far.
- Add a GPS module in order to automatically obtain the latitude, longitude and altitude information. Most modern GPS chips incorporate the World Magnetic Model and therefore the computation of the magnetic declination may become obsolete. Additionally, the GPS could be used as an heading source if two precise and far apart GPS antennas are used. Alternatively the heading may also be recovered by analysing the past trajectory after some movement of the robot. This would allow for a more accurate heading estimation anywhere on Earth, but in particular in the Blackout Zones.
- The above would also enable the robot to be moving while conserving a notion of where the True North heading is located.
- The above improvements may require a slight rework of the implementation structure, for example by enabling separate threads for each publisher, to allow for a reduction of the delay between messages or commands.
- There was only a single global integration test implemented in this version due to time constraints. More test cases should be implemented that perform unit tests on each of the methods and classes.

## References

1. Chulliat, Arnaud, et al. "The US/UK World Magnetic Model for 
2020-2025: Technical Report." (2020).

2. WMM software: https://www.ngdc.noaa.gov/geomag/WMM/soft.shtml
