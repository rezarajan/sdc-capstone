# **Capstone Project - System Integration** 

## Writeup

The following document outlines the process of integrating numerous systems using ROS, to enable a vehicle in a simulated environment to drive autonomously.

---

## Capstone Project

The goals / steps of this project are the following:
* Implement a path planner which updates waypoints for the vehicle to follow
* Implement a PID controller which handles throttle, steering and braking for the vehicle to match the required pose set by each waypoint
* Implement an image classifier which can detect the state of traffic lights
* Safely navigate a vehicle around the simulated track, obeying the following constraints:
  * Avoid collisions with merging traffic
  * Ensure the vehicle always stays on track
  * Ensure the vehicle stops at red lights
  * Ensure that the vehicle's speed does not exceed the 50 MPH speed limit
  * Ensure that the vehicle's acceleration does not exceed 10ms<sup>-2</sup>
  * Ensure that the vehicle's jerk does not exceed 10ms<sup>-3</sup>
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./imgs/ssd_mobilenet_latency.png "SSD MobileNet Inference Latency"
[image2]: ./imgs/faster_rcnn_latency.png "Faster R-CNN Inference Latency"
[image3]: ./imgs/collision.gif "Collision Example"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/3058/view) and describe how I addressed each point in my implementation.  

---
## Reflection

This documentation is an optional part of the rubric, but as with such projects creating one is always beneficial. Here is a link to the [project code](https://github.com/rezarajan/sdc-path-planning.git). A video showcasing the car driving for one loop around the track is available [here](). #TODO

---

## Running the Code

*The code is built successfully and connects to the simulator.*

See the [readme](./README.md) to get the code set up and running quickly. The code has been tested on a local machine, and successfully runs. On the project workspace, which has much less compute capability, a constrained version of the code runs without image classification. The linked readme outlines this in more detail.

---

## Control and Planning

*Waypoints are published to plan Carla’s route around the track.*

Waypoint updates are handled by `waypoint_updater.py`. The code first receives and stores the base waypoints, which plan the car's route around the track. Depending on whether there is a red light ahead of the car or not, the waypoints are modified to ensure the [desired behaviour](#capstone-project), before being published to the `final_waypoints` topic.

*Controller commands are published to operate Carla’s throttle, brake, and steering.*

The car's control of throttle is handled using a PID controller implemented in `pid.py`. Steering is handled by a yaw controller called `yaw_controller.py`. The braking is calculated mathematically by deriving the torque from the vehicle's weight, radius and desired deceleration value. These controllers are all combined in `twist_controller.py` to receive throttle, brake and steering commands which try to match the pose of each waypoint. Commands are published to the `/vehicle/throttle_cmd`, `/vehicle/brake_cmd` and `/vehicle/steering_cmd` for throttle, brake and steering, respectively.

## Successful Navigation

*Successfully navigate the full track more than once.*

The vehicle is able to successfully navigate the track more than once, adhereing to the [project constraints](#capstone-project).

---

## A Note on Image Detection and Classification

The model......#TODO
![image1]
![image2]

---

## Recommendations

### *Improving the Image Detection and Classification Model*

* The current implementation of the image detector and classifier is still relatively computationally heavy. Granted that on most systems, running locally, the code runs well in the simulator, it still does not run with sufficient latency in the workspace environment provided by Udacity. Since the simulator's environment is relatively simple, a potential solution is to create a TensorFlow lite version of the model which may offer latency improvements with similar accuracy.
* The image detector and classifier currently uses an SSD MobileNet backbone, which has drawbacks of not being able to generalize well enough for use in multiple environments. As such, a model trained for the simulated environment will not be suitable in terms of accuracy for use in the real world, and vice-versa. Different models, such as *Faster R-CNN* should be trained and tested for use in multiple environments, instead of the SSD MobileNet model.
  
### *Performance Optimization*

* The code for this project is mostly implemented in Python. It is recommended that the code be ported to C++, which is supported by ROS and is vastly more performant than Python. This may also resolve the usage issues of performing image operations on computationally constrained hardware described [above](#running-the-code).

## Conclusions

Path planning, image detection and classification, and control systems have been successfully integrated using ROS to navigate a vehicle around a track in accordance with the [project specifications](#capstone-project). Recommendations have also been provided for further improvement of the system's image detection and classification performance, and general code performance, which will allow it to be more suitable for real-time applications in a vehicle driving in the real world.