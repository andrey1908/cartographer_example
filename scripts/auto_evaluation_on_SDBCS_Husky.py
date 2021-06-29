import subprocess
import os
import argparse


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-bags-fld', '--rosbags-folder', required=True, type=str)
    parser.add_argument('-out-test-fld', '--out-test-folder', required=True, type=str)
    parser.add_argument('-urdf-fld', '--urdf-folder', required=True, type=str)
    parser.add_argument('-val-fld', '--validation-folder', required=True, type=str)
    parser.add_argument('-urdf-v', '--urdf-version', type=int, default=1, choices=[1, 2, 3])
    parser.add_argument('-imu', '--imu-sensor', type=str, default='xsens', choices=['xsens', 'atlans'])
    parser.add_argument('-bags-to-use', '--rosbag-numbers-to-use', type=int, nargs='+')
    return parser


def make_dirs(out_test_folder, validation_folder):
    os.makedirs(out_test_folder, exist_ok=True)
    os.makedirs(os.path.join(validation_folder, 'gt'), exist_ok=True)
    os.makedirs(os.path.join(validation_folder, 'results'), exist_ok=True)


def get_rosbag_filenames(rosbags_folder, rosbag_numbers_to_use=None):
    rosbag_filenames = os.listdir(rosbags_folder)
    rosbag_filenames = [bag for bag in rosbag_filenames if bag.endswith('bag')]
    rosbag_filenames = [bag for bag in rosbag_filenames if bag[:2].isdigit()]
    assert(len(rosbag_filenames) == 21)
    if rosbag_numbers_to_use:
        rosbag_filenames = [bag for bag in rosbag_filenames if int(bag[:2]) in rosbag_numbers_to_use]
    rosbag_filenames.sort()
    return rosbag_filenames


def get_urdf_filename(urdf_folder, bag, urdf_version):
    pattern = '_2020-'
    date_len = 5
    urdf_filename_template = 'SDBCS_Husky_{}{}.urdf'
    
    index = bag.find(pattern)
    assert(index != -1)
    index += len(pattern)
    date = bag[index:index+date_len]
    assert(len(date) == date_len)
    
    urdf_version_suffix = {1: '', 2: '_v2', 3: '_v3'}[urdf_version]
    urdf_filename = urdf_filename_template.format(date, urdf_version_suffix)
    urdf_filename = os.path.abspath(os.path.join(urdf_folder, urdf_filename))
    
    return urdf_filename


def auto_evaluation_on_SDBCS_Husky(rosbags_folder, out_test_folder, urdf_folder, validation_folder, \
                                   urdf_version=1, imu_sensor='xsens', rosbag_numbers_to_use=None):
    make_dirs(out_test_folder, validation_folder)
    bags = get_rosbag_filenames(rosbags_folder, rosbag_numbers_to_use=rosbag_numbers_to_use)
    imu_frame = {'xsens': 'imu', 'atlans': 'isns_link'}[imu_sensor]
    log = str()


    # Run cartographer_offline to generate maps
    for bag in bags:
        rosbag_filename = os.path.abspath(os.path.join(rosbags_folder, bag))
        out_pbstream_filename = os.path.abspath(os.path.join(out_test_folder, '{}.pbstream'.format(bag[:2])))

        command = "roslaunch cartographer_example cartographer_offline.launch    bag_filenames:={}    urdf_filenames:={}    \
save_state_filename:={}".format(rosbag_filename, get_urdf_filename(urdf_folder, bag, urdf_version), out_pbstream_filename)
        log += command + '\n\n\n'
        print('\n\n\n' + command + '\n')
        process = subprocess.Popen(command.split())
        process.communicate()
        assert(process.returncode == 0)


    # Retrive SLAM trajectories from cartographer maps
    for bag in bags:
        pbstream_filename = os.path.abspath(os.path.join(out_test_folder, '{}.pbstream'.format(bag[:2])))
        out_trajectory_bag_filename = os.path.abspath(os.path.join(out_test_folder, '{}.bag'.format(bag[:2])))

        command = "rosrun cartographer_ros cartographer_dev_pbstream_trajectories_to_rosbag    -input {}    -output {}    \
-tracking_frame {}".format(pbstream_filename, out_trajectory_bag_filename, imu_frame)
        log += command + '\n\n\n'
        print('\n\n\n' + command + '\n')
        process = subprocess.Popen(command.split())
        process.communicate()
        assert(process.returncode == 0)


    # Prepare poses in kitti format for evaluation
    for bag in bags:
        rosbag_filename = os.path.abspath(os.path.join(rosbags_folder, bag))
        results_poses_bag_filename = os.path.abspath(os.path.join(out_test_folder, '{}.bag'.format(bag[:2])))
        out_gt_poses_filename = os.path.abspath(os.path.join(validation_folder, 'gt', '{}.txt'.format(bag[:2])))
        out_results_poses_filename = os.path.abspath(os.path.join(validation_folder, 'results', '{}.txt'.format(bag[:2])))
        out_trajectory_bag_filename = os.path.abspath(os.path.join(out_test_folder, '{}_trajectories.bag'.format(bag[:2])))

        command = "python /home/cds-jetson-host/catkin_ws/src/ros_utils/scripts/prepare_poses_for_evaluation.py    \
-gt-bag {}    -gt-topic /atlans_odom    -res-bag {}    -res-topic trajectory_0    -transforms-source {}    \
-out-gt {}    -out-res {}    -out-paths {}".format(rosbag_filename, results_poses_bag_filename, \
            get_urdf_filename(urdf_folder, bag, urdf_version), out_gt_poses_filename, out_results_poses_filename, out_trajectory_bag_filename)
        log += command + '\n\n\n'
        print('\n\n\n' + command + '\n')
        process = subprocess.Popen(command.split())
        process.communicate()
        assert(process.returncode == 0)


    # Run evaluation
    for projection in ['xy', 'xz', 'yz']:
        gt_poses_folder = os.path.abspath(os.path.join(validation_folder, 'gt'))
        results_poses_folder = os.path.abspath(os.path.join(validation_folder, 'results'))
        out_folder = os.path.abspath(os.path.join(validation_folder, 'output_{}'.format(projection)))

        command = "python /home/cds-jetson-host/slam_validation/evaluate_poses.py    --dir_gt {}    --dir_result {}    \
--dir_output {}    --gt_format kitti    --result_format kitti    \
--projection {}".format(gt_poses_folder, results_poses_folder, out_folder, projection)
        log += command + '\n\n\n'
        print('\n\n\n' + command + '\n')
        process = subprocess.Popen(command.split())
        process.communicate()
        assert(process.returncode == 0)
    
    print(log)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    auto_evaluation_on_SDBCS_Husky(**vars(args))
