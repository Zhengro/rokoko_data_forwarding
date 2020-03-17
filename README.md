# rokoko_data_forwarding
A ROS package that can forward online data from Rokoko Smartsuit Pro via Rokoko Studio Custom Streaming.

## Requirements
* Rokoko Smartsuit Pro
* Rokoko Studio
* ROS machine
* Router

## Installation
```
export ROS_DISTRO=kinetic                                          # Set this to your distro, e.g. kinetic or melodic
source /opt/ros/$ROS_DISTRO/setup.bash                             # Source your ROS distro 
mkdir -p ~/catkin_ws/src                                           # Make a new catkin workspace if no one exists
cd ~/catkin_ws/src                                                 # Navigate to the source space
git clone https://github.com/Zhengro/rokoko_data_forwarding.git    # Clone repo
cd ~/catkin_ws                                                     # Navigate to the catkin workspace
catkin_make                                                        # Build the packages in the catkin workspace
source ~/catkin_ws/devel/setup.bash                                # Add the workspace to your ROS environment
```
## Test steps
0. Run the following three lines on every new shell you open or add them to your .bashrc:
```
export ROS_DISTRO=kinetic
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/catkin_ws/devel/setup.bash
```
1. Open a terminal and make sure that a roscore is up and running:
```
roscore
```
2. Open a second terminal and run the publisher:
```
cd ~/catkin_ws
rosrun rokoko_data_forwarding online_data_publisher.py
```
3. Open a third terminal and run the subscriber:
```
cd ~/catkin_ws
rosrun rokoko_data_forwarding online_data_subscriber.py
```
4. When test is done, press Ctrl-C to terminate each terminal.

## Reminder
* Always connect the devices in the same network
* Set warm_up=0 in online_data_publisher.py if necessary
