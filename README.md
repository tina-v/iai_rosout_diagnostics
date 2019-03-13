# iai_rosout_diagnostics

A ROS utility to relay rosout messages to diagnostics.

## Rationale

In our systems, some nodes report errors via rosout, i.e. calling ```rospy.logerror``` or similar. Often, these printouts get overlooked because too much is going on or nobody watches the console. 

As a quick-fix, we suggest ```iai_rosout_diagnostics``` which is a lightweight node that scans the ```/rosout``` topic and relays relevant information to the ```/diagnostics``` topic. Then, people have one place to get a quick overview of issues with the systems: ```rqt_robot_monitor```. 

## Installation
Using ```catkin_tools``` and ```wstool``` in a new workspace for ```ROS Kinetic```:
```
source /opt/ros/kinetic/setup.bash         # start using ROS Kinetic
mkdir -p ~/my_ws/src                       # create directory for workspace
cd ~/my_ws                                 # go to workspace directory
catkin init                                # init workspace
cd src                                     # go to source directory of workspace
git clone https://github.com/airballking/my_ros_monitor.git
                                           # get source code
rosdep install --ignore-src --from-paths . # install dependencies available through apt
cd ..                                      # go to workspace directory
catkin build                               # build packages
source ~/my_ws/devel/setup.bash       # source new overlay
```

## Examples
Run in your console:
```shell
roslaunch iai_rosout_diagnostics test.launch
```

This will bring up two several ROS nodes:
```
$ rosnode list 
/diagnostic_aggregator
/iai_rosout_diagnostics
/node_with_issues
/node_without_fault
/rosout
/rqt_robot_monitor
```
```/node_with_issues``` and ```/node_without_fault``` produces prints using rosout. ```/iai_rosout_diagnostics``` collects their messages and sends to them to the ```/diagnostics``` topic. The ```/diagnostics_aggregator``` filters them and produces an aggregate view that you can visualize with ```/rqt_robot_monitor```. This could look like this:
![rqt robot monitor](https://raw.githubusercontent.com/airballking/my_ros_monitor/master/data/screenshot_rqt_robot_monitor.png)
