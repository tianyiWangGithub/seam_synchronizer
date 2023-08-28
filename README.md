# seam_synchronizer
seam_synchronizer SEAM is a novel message synchronization policy designed to fuse data from different sensors in ROS2. This project integrates SEAM into the ROS2 Dashing framework and evaluates its effectiveness through a series of experiments, comparing it to traditional ROS2 synchronizers. This project compare SEAM algorithm with [ROS2 Approximate Time Policy](https://github.com/ros2/message_filters/blob/master/include/message_filters/sync_policies/approximate_time.h) in success rate and compute time with changing parameters like number of channels, period of messages, threshold of SEAM algorithm, etc.

## Before running
You need to execute commands below before running test
First, clone this package in your workspace, replace work_space as your workspace name
```
cd ~/work_space/src
git clone https://github.com/tianyiWangGithub/seam_synchronizer
```
Copy files from _message_filters/sync_policies_ to _ros2_dashing/src/ros2/message_filters/include/message_filters/sync_policies_

## Build the packages
Buid these message_filters package and seam_synchronizer package in your workspace
```
cd ~/ros2_dashing
colcon build --packages-select message_filters --symlink-install
source ./install/setup.bash
colcon build --packages-select seam_synchronizer --symlink-install
```

## Run the experiment
1 Evaluate the performance by comparing the success rate of sensing data synchronization of the SEAM algorithm with the built-in ROS 2 ApproximateTime algorithm
```
cd ~/ros2_dashing/seam_synchronizer
./run_seam.sh
```
2 Evaluate the performance by comparing the compute time of sensing data synchronization of the SEAM algorithm with the built-in ROS 2 ApproximateTime algorithm
```
cd ~/ros2_dashing/seam_synchronizer
./run_computetime.sh
```
