# rokoko_data_forwarding
A ROS package that can forward online data from Rokoko Smartsuit Pro via Rokoko Studio Custom Streaming.

## To be tested (ROS machine + router + Smartsuit + pc)
1. Installation
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
2. Forwarding via Custom Streaming
