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

A major part of this project, and a key aspect of enabling its use in real-world environments, is to build and implement a tool which can detect and classify objects in real-time. Naturally, any driver must be able to understand the environment they are driving in - this may include obstacles such as pedestrians, traffic lights, lane segmentation and warning signs. While there are many ways to accomplish some of these tasks, a more modern, effective and robust way is to utilize machine learning models.

For this project the only task being tackled regarding environment analysis is to observe for traffic lights so that the vehicle may know when to stop, or proceed. Furthermore, it is imperative that the analysis be performed in real-time such that the vehicle may react to any sudden changes. Therefore, a Single-Shot Detection (SSD) MobileNet model is used to detect and classify traffic lights as either 'Green', 'Yellow', 'Red' or 'Off', in real-time.

### *Training*

Both models are trained on a set of real-world images of traffic lights by Autti. The actual dataset comes from Udacity, however, and is found here as [Dataset 2](https://github.com/udacity/self-driving-car/tree/master/annotations).

Due to a shortcoming of SSD MobileNet mentioned [below](#performance), the SSD MobileNet model is also trained on traffic light data gathered from the simulator. A well-documented guide on how this is performed can be found [here](https://github.com/alex-lechner/Traffic-Light-Classification). Furthermore, linked is a guide on the actual training process using [TensorFlow's Object-Detection API](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/training.html). 

### *Performance*

On the note of real-time performance, the MobileNet variant of the model is chosen as it significantly reduces the number of parameters in the network, making it faster for training and inference. A drawback of this, however, is that these models tend to offer less accuracy and precision than full models, but in some cases, depending on the computational capability of the system it is run on, the latency gains in inference are well worth the tradeoff.

Two models have been tested for inference latency on a batch of sample images: the SSD MobileNet and Faster R-CNN models. Their performances are shown below:

![image1]

The SSD MobileNet model offers an average latency of 41ms, with a low of 40ms and high of 42ms.

![image2]

The Faster R-CNN model offers an average latency of 485ms, with a low of ~460ms and high of ~500ms.

Clearly the SSD MobileNet model (at least on the hardware it has been tested on) is more suitable for real-time applications than Faster R-CNN. Testing in the simulator proved this to be the case as well, however the tradeoff in accuracy and precision may not be suitable for production environments. A very important observation is that from testing it has become clear that the MobileNet model is not able to generalize as well as other models - a model trained on real-world images of traffic lights will not perform with any reasonable amount of accuracy for use in the simulator, and vice-versa. Therefore, if the MobileNet model is used then two models must be trained - one using simulator images, and one using real-world images. The tradeoff in the time it takes to train these models alone may warrant the use of another model.

---

## Recommendations

### *Improving the Image Detection and Classification Model*

* The current implementation of the image detector and classifier is still relatively computationally heavy. Granted that on most systems, running locally, the code runs well in the simulator, it still does not run with sufficient latency in the workspace environment provided by Udacity. Since the simulator's environment is relatively simple, a potential solution is to create a TensorFlow lite version of the model which may offer latency improvements with similar accuracy.
* The image detector and classifier currently uses an SSD MobileNet backbone, which has drawbacks of not being able to generalize well enough for use in multiple environments. As such, a model trained for the simulated environment will not be suitable in terms of accuracy for use in the real world, and vice-versa. Different models, such as *Faster R-CNN* should be trained and tested for use in multiple environments, instead of the SSD MobileNet model.
  
### *Performance Optimization*

* The code for this project is mostly implemented in Python. It is recommended that the code be ported to C++, which is supported by ROS and is vastly more performant than Python. This may also resolve the usage issues of performing image operations on computationally constrained hardware described [above](#running-the-code).

## Conclusions

Path planning, image detection and classification, and control systems have been successfully integrated using ROS to navigate a vehicle around a track in accordance with the [project specifications](#capstone-project). Recommendations have also been provided for further improvement of the system's image detection and classification performance, and general code performance, which will allow it to be more suitable for real-time applications in a vehicle driving in the real world.