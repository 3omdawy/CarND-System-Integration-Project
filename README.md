This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).
* Note: obstacle detection is excluded

The implemented modules are:

### 1. DBW (Drive-By-Wire) Node

* This node generates the control commands to the simulator or car (throttle, brake, and steering)
* It receives the required twist commands as well as the current state if the car
* PID controller was used for the throttle / brake commands while the steering command was handled by a yaw controller (both provided in Udacity repo).
* Testing (uncomment the logging in line 110 of dbw_node.py):
	- Normally: throttle is 0.4457 and brake and steering are 0 (car is striaght and stopped).
	- Manually increase speed: throttle decreases then becomes 0 and brake increases
	- Manually go left or right --> steering is updated in the opposite direction
	- Update the command in waypoint updater --> target speed is updated accordingly

### 2. Waypoint Updater Node

* This node generates the final path to be followed by the control module (way points with the intended speed at each waypoint) 
* It receives information about traffic light and car position and a map of waypoints.
* Logic: if a red light is detected ahead, set the target speed of next way points to 0. Otherwise follow a speed of 10 mph.
* Testing (uncomment the logging in line 97 of waypoint_updater.py):
	- Detected waypoint starts from 272 then increases gradually till the length of waypoints (> 10000) when moving the car forward manually then starts from 0 again.
	- Position is updated correctly (x increases as we go forward)
	- Waypoint message is generated correctly with target speed
	- When red light is passed from traffic light detector, log message is generated indicating this and also speed is set to 0.


### 3. Traffic Light Detection Node
* This node generates information about red traffic lights that are in front of and near the car 
* It receives images from the camera mounted on the car, as well as the car position and a map of waypoints
* Logic:  State of the art SSD trained by google and a simple opencv color thresholding script was used as described [here] (https://github.com/cochoa0x1/system-integration). Due to tensorflow version conflicts, resnet was used on the final version instead of the faster SSD mobilenet. On modest hardware this led to some interesting problems caused by the lag, optimizations are needed.
* Note: when using it on my PC (an extremely slow and light Inspiron): it gave a warning that a GPU is better [it can work with a CPU but the performance will be degraded]
* Testing (uncomment the logging in line 176 of tl_detector.py):
	-  Approach red lights --> lights are detected and message is published and received by waypoint updatder
	- Move away from them --> no new log messages


### External dependencies
* Please download the coco classifier [here](http://download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz) and unzip it inside ros/src/tl_detector/light_classification.
* Recommendation: use a GPU for the classifier to give better results

### Native Installation
* This is an indicidual submission, just to run on the simulator (not on Carla)

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
	- Note: I used the VM provided by Udacity (as described [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/7e3627d7-14f7-4a33-9dbf-75c98a6e411b/concepts/8c742938-8436-4d3d-9939-31e40284e7a6?contentVersion=1.0.0&contentLocale=en-us))
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this (which I did).

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
		- Again the link provided above for Udacity VM will work for these items
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/3omdawy/CarND-System-Integration-Project
```

2. Install python dependencies
```bash
cd CarND-System-Integration-Project
pip install -r requirements.txt
```
* Note: requirements.text for me was adapted as done [here](https://github.com/cochoa0x1/system-integration) as I re-used the classifier in this repository.

3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator (but take case not to make a large delay from step 3 otherwise a timeout can occur)

### Real world testing (not done by me)
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-System-Integration-Project/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
