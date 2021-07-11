# Usage Guide

The project may be run natively, or through a Docker container. The recommended way to run the code is through Docker, to ensure consistency. However, in some instances running Docker may not be possible, for instance in some virtual environments. Please refer to the [installation instructions](./Installation.md) for guides on how to set up the system.

For simplicity, the **Docker usage guide** is shown below for **usage with the simulator only**. To get set up, the following steps are required:

### Clone the repository

Run the following command to clone the repository:
```sh 
git clone https://github.com/rezarajan/sdc-capstone.git
cd sdc-capstone
```
### Running the Code

The script to execute code depends on the system type. On constrained systems a more lightweight version of the code will run, which does not perform image classification for traffic lights. At the point of writing the online workspace is not performant enough to run inference, and bogs the system to the point where it is unusable. Otherwise, image classification will be performed.

---

**Running Locally**

1. Build the Docker image:
```sh
docker build . -t capstone
```
2. Run the Docker container
```sh
./run_container.sh
```
*Note: The terminal should now be displaying the Docker container's environment. Continue the steps in the container's terminal.*

3. From inside the Docker container, build the project packages
```sh
catkin_make
```
4. Source the environment
```sh
source devel/setup.bash
```
5. Start ROS
```sh
roslaunch launch/styx.launch
```

---

**Running on the constrained systems:**

1. Build the project packages
```sh
cd ros
catkin_make
```
2. Source the environment
```sh
source devel/setup.bash
```
3. Start ROS
```sh
# This launch file is different, and ensures computationally intensive tasks are suppressed
roslaunch launch/styx-ws.launch
```

### Launch the Simulator
Once roslaunch has finished, open the simulator, and **disable manual mode** and **enable camera**. The car should start navigating around the track in a loop. 

*Note: when running the code locally (not in constrained mode) please wait for TensorFlow to load the inference model before running the simulator. A prompt will appear stating 'Traffic Light Detection Model Loaded', after which it is safe to run the simulator.*