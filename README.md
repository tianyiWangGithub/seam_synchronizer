# seam_synchronizer
seam_synchronizer SEAM is a novel message synchronization policy designed to fuse data from different sensors in ROS2. This project integrates SEAM into the ROS2 Dashing framework and evaluates its effectiveness through a series of experiments, comparing it to traditional ROS2 synchronizers. This project compare SEAM algorithm with [ROS2 Approximate Time Policy](https://github.com/ros2/message_filters/blob/master/include/message_filters/sync_policies/approximate_time.h) in success rate and compute time with changing parameters like number of channels, period of messages, threshold of SEAM algorithm, etc.

## Before running
You need to execute commands below before running test
First, clone this package in your workspace, replace work_space as your workspace name
```
cd ~/work_space/src
git clone https://github.com/tianyiWangGithub/seam_synchronizer
```
Copy files from `message_filters/sync_policies` to `ros2_dashing/src/ros2/message_filters/include/message_filters/sync_policies`

## Build the packages
Buid these message_filters package and seam_synchronizer package in your workspace
```
cd ~/ros2_dashing
colcon build --packages-select message_filters --symlink-install
source ./install/setup.bash
colcon build --packages-select seam_synchronizer --symlink-install
```

## Before the experiments
The `results` folder already contains the results from previous experiments. Before running the experiments, you can execute `run_draw_py.sh` to generate graphical representations of the results. Running `run_seam.sh` or `run_computetime.sh` will overwrite the existing results. To obtain new graphical representations of the experimental results, you can run `run_draw_py.sh` again. In addition, the experiments are configured to run 1000 sets of experiments for each result. If you wish to change the number of experiments, you can modify the `num_group` value in `run_seam.sh` or `run_computetime.sh`.

## Run the experiments
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
3 Draw graphs based on the experimental results obtained before
```
cd ~/ros2_dashing/seam_synchronizer
./run_draw_py.sh
```
