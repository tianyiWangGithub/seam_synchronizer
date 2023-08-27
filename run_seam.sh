#!/bin/bash

source ~/ros2_dashing/install/setup.bash

# Number of test group
num_group=1

################################# Channel #############################
ros2 run seam_synchronizer validation_seam 3 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/channel_num/channel_3 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 4 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/channel_num/channel_4 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 5 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/channel_num/channel_5 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/channel_num/channel_6 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 7 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/channel_num/channel_7 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 8 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/channel_num/channel_8 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 9 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/channel_num/channel_9 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml

########################## C (bound used in SEAM) ####################
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_75 75 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_80 80 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_85 85 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_90 90 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_95 95 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_100 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_105 105 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_110 110 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_115 115 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/C/C_120 120 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml

######################### Period (Range upper 100ms) ###########################
ros2 run seam_synchronizer validation_seam 6 10 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/period/period_10 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 20 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/period/period_20 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/period/period_30 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 40 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/period/period_40 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 50 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/period/period_50 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml

################################ Tw/Tb ####################################
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/Tw_Tb/Tw_Tb_100 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam_100.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/Tw_Tb/Tw_Tb_105 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam_105.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/Tw_Tb/Tw_Tb_110 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam_110.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/Tw_Tb/Tw_Tb_115 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam_115.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/Tw_Tb/Tw_Tb_120 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/Tw_Tb/Tw_Tb_125 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam_125.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/Tw_Tb/Tw_Tb_130 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam_130.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/Tw_Tb/Tw_Tb_135 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam_135.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/Tw_Tb/Tw_Tb_140 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam_140.yaml

############################ B (bound between each 2 sequent fusion) ####################################
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_100 100 50 100 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_105 100 50 105 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_110 100 50 110 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_115 100 50 115 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_120 100 50 120 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_125 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_130 100 50 130 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_135 100 50 135 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_140 100 50 140 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/B/B_145 100 50 145 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml

#################################### internal length #########################################################
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_10 100 10 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_20 100 20 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_30 100 30 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_40 100 40 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_50 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_60 100 60 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_70 100 70 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_80 100 80 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_90 100 90 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
ros2 run seam_synchronizer validation_seam 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_success_rate/T/T_100 100 100 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_seam.yaml
