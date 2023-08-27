#!/bin/bash

source ~/ros2_dashing/install/setup.bash

# Number of test group
num_group=1

############### Channel ####################

# ros2 run seam_synchronizer validation_seam_computetime 3 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/channel/channel_3 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
ros2 run seam_synchronizer validation_seam_computetime 4 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/channel/channel_4 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 5 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/channel/channel_5 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/channel/channel_6 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 7 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/channel/channel_7 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 8 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/channel/channel_8 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 9 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/channel/channel_9 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml

################ C (bound used in SEAM) ####################
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_75 75 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_80 80 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_85 85 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_90 90 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_95 95 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_100 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_105 105 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_110 110 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_115 115 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/C/C_120 120 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml

################ Period (Range upper 100ms) ###################
# ros2 run seam_synchronizer validation_seam_computetime 6 10 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/period/period_10 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 20 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/period/period_20 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/period/period_30 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 40 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/period/period_40 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 50 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/period/period_50 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml

################# Tw/Tb ####################################
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/Tw_Tb/Tw_Tb_100 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_t100.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/Tw_Tb/Tw_Tb_105 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_t105.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/Tw_Tb/Tw_Tb_110 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_t110.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/Tw_Tb/Tw_Tb_115 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_t115.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/Tw_Tb/Tw_Tb_120 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_t120.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/Tw_Tb/Tw_Tb_125 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_t125.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/Tw_Tb/Tw_Tb_130 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_t130.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/Tw_Tb/Tw_Tb_135 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_t135.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/Tw_Tb/Tw_Tb_140 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_t140.yaml

###################### B (bound between each 2 sequent fusion) ####################################
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_100 100 50 100 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_105 100 50 105 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_110 100 50 110 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_115 100 50 115 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_120 100 50 120 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_125 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_130 100 50 130 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_135 100 50 135 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_140 100 50 140 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/B/B_145 100 50 145 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml

######################### internal length #########################################################
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_10 100 10 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_20 100 20 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_30 100 30 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_40 100 40 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_50 100 50 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_60 100 60 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_70 100 70 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_80 100 80 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_90 100 90 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml
# ros2 run seam_synchronizer validation_seam_computetime 6 30 $num_group /home/wty/ros2_dashing/src/seam_synchronizer/results/validation_seam_computetime/internal_length/internal_length_100 100 100 125 __params:=/home/wty/ros2_dashing/src/seam_synchronizer/config/config_computetime_common.yaml

python3 /home/wty/ros2_dashing/src/seam_synchronizer/scripts/compute_average_N.py
# python3 /home/wty/ros2_dashing/src/seam_synchronizer/scripts/compute_average_C.py
# python3 /home/wty/ros2_dashing/src/seam_synchronizer/scripts/compute_average_P.py
# python3 /home/wty/ros2_dashing/src/seam_synchronizer/scripts/compute_average_ratio.py
# python3 /home/wty/ros2_dashing/src/seam_synchronizer/scripts/compute_average_B.py
# python3 /home/wty/ros2_dashing/src/seam_synchronizer/scripts/compute_average_T.py