import subprocess
import os

bags = os.listdir('/home/cds-jetson-host/catkin_ws')
bags = [bag for bag in bags if bag.endswith('bag')]
bags = [bag for bag in bags if bag[:2].isdigit()]
assert(len(bags) == 7)
bags.sort()

for bag in bags:
    command = "roslaunch cartographer_example cartographer_offline.launch bag_filenames:=/home/cds-jetson-host/catkin_ws/{} \
        urdf_filenames:=/home/cds-jetson-host/catkin_ws/src/cartographer_example/urdf/SDBCS_Husky.urdf \
        save_state_filename:=/home/cds-jetson-host/catkin_ws/cartographer_test/3d/no_opt/{}.pbstream".format(bag, int(bag[:2]))
    process = subprocess.Popen(command.split())
    process.communicate()

for bag in bags:
    command = "rosrun cartographer_ros cartographer_dev_pbstream_trajectories_to_rosbag \
        -input /home/cds-jetson-host/catkin_ws/cartographer_test/3d/no_opt/{}.pbstream \
        -output /home/cds-jetson-host/catkin_ws/cartographer_test/3d/no_opt/{}.bag".format(int(bag[:2]), int(bag[:2]))
    process = subprocess.Popen(command.split())
    process.communicate()

for bag in bags:
    command = "python /home/cds-jetson-host/catkin_ws/src/cartographer_example/scripts/prepare_poses_for_evaluation.py \
        -bag-gt /home/cds-jetson-host/catkin_ws/{} -gt-topic /atlans_odom \
        -bag-res /home/cds-jetson-host/catkin_ws/cartographer_test/3d/no_opt/{}.bag -res-topic trajectory_0 \
        -out-gt /home/cds-jetson-host/slam_validation/cartographer/3d/no_opt/gt/{}.txt \
        -out-res /home/cds-jetson-host/slam_validation/cartographer/3d/no_opt/results/{}.txt".format(bag, int(bag[:2]), int(bag[:2]), int(bag[:2]))
    process = subprocess.Popen(command.split())
    process.communicate()

for projection in ['xy', 'xz']:
    command = "python /home/cds-jetson-host/slam_validation/evaluate_poses.py \
        --dir_gt /home/cds-jetson-host/slam_validation/cartographer/3d/no_opt/gt \
        --dir_result /home/cds-jetson-host/slam_validation/cartographer/3d/no_opt/results \
        --dir_output /home/cds-jetson-host/slam_validation/cartographer/3d/no_opt/output_{} \
        --gt_format kitti --result_format kitti --projection {}".format(projection, projection)
    process = subprocess.Popen(command.split())
    process.communicate()
