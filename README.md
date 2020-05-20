# rokoko_data_forwarding
A ROS package that can forward online data from [Rokoko Smartsuit Pro](https://www.rokoko.com) via [Rokoko Studio](https://rokoko.freshdesk.com/support/solutions/articles/47001111806-rokoko-studio-1-15-1) Custom Streaming.

## Table of contents
   - [Requirements](#requirements)
   - [Installation](#installation)
   - [Usage](#usage)

## Requirements
* Rokoko Smartsuit Pro
* Rokoko Studio
* ROS machine (e.g., indigo)
* Router

(Always connect the devices in the same network.)

### Setup example
The ROS machine is connected to the router via a cable. Rokoko Studio is installed on another machine which is wirelessly connected to the router. The connection between Rokoko Smartsuit Pro and the router is configured in the Studio Custom Streaming (an example can be found [here](https://github.com/Zhengro/rokoko_data_forwarding/blob/master/setup_custom%20streaming.png)) and do not forget to follow this [guide](https://rokoko.freshdesk.com/support/solutions/articles/47001095035-getting-started-guide-smartsuit-pro) for Studio and Smartsuit. Note that Custom Streaming requires Studio Plus subscription or above.

## Installation
```
export ROS_DISTRO=indigo                                           # Set this to your distro
source /opt/ros/$ROS_DISTRO/setup.bash                             # Source your ROS distro 
mkdir -p ~/catkin_ws/src                                           # Make a new catkin workspace if no one exists
cd ~/catkin_ws/src                                                 # Navigate to the source space
git clone https://github.com/Zhengro/rokoko_data_forwarding.git    # Clone repo
cd ~/catkin_ws                                                     # Navigate to the catkin workspace
catkin build rokoko_data_forwarding                                # Build rokoko_data_forwarding
source ~/catkin_ws/devel/setup.bash                                # Add the workspace to your ROS environment
```
## Usage
0. Run the following three lines on every new shell you open or add them to your .bashrc:
```
export ROS_DISTRO=indigo
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash
```
1. Open a terminal and make sure that a roscore is up and running:
```
roscore
```

2. Open a second terminal for the [publisher](https://github.com/Zhengro/rokoko_data_forwarding/blob/master/scripts/online_data_publisher.py) (set **_warm_up_** and **_fps_** beforehand):
```
rosrun rokoko_data_forwarding online_data_publisher.py
```

3. Open a third terminal for the [subscriber](https://github.com/Zhengro/rokoko_data_forwarding/blob/master/scripts/online_data_subscriber.py):
```
rosrun rokoko_data_forwarding online_data_subscriber.py
```

4. Additionaly, online data can be recorded by opening another terminal and running (set **_num_frames_** and **_file_name_** beforehand):
```
rosrun rokoko_data_forwarding online_data_recording.py
```
   When test is done, press Ctrl-C to terminate each terminal.

5. Then, you can view the skeleton and determine separate points of multiple motions in a complete record, using [recorded_data_animation.py](https://github.com/Zhengro/rokoko_data_forwarding/blob/master/scripts/recorded_data_animation.py). To futher preprocess the recorded data, use [recorded_data_preprocessing.py](https://github.com/Zhengro/rokoko_data_forwarding/blob/master/scripts/recorded_data_preprocessing.py).
