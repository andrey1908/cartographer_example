import os
import argparse
from auto_evaluation import auto_evaluation


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-bags-fld', '--rosbags-folder', required=True, type=str, help="Folder with rosbag files from SDBCS_Husky dataset. The folder may contain other files.")
    parser.add_argument('-out-test-fld', '--out-test-folder', required=True, type=str, help="Folder to save intermediate results.")
    parser.add_argument('-val-fld', '--validation-folder', required=True, type=str, help="Folder to save kitti poses for evaluation.")

    parser.add_argument('-calib-v', '--calibration-version', type=int, default=1, choices=[1, 2, 3], help="What version of calibraion to use.")
    parser.add_argument('-imu', '--imu-sensor', type=str, default='xsens', choices=['xsens', 'atlans'], help="What imu to use.")
    parser.add_argument('-dim', '--dimension', type=str, default='3d', choices=['3d', '2d'], help="Which SLAM to use: 2d or 3d.")
    parser.add_argument('-node', '--node-to-use', type=str, default='online', choices=['online', 'offline'], help="Cartographer mode.")
    parser.add_argument('--get-odom-from-transforms', action='store_true')
    parser.add_argument('--get-odom-from-topic', action='store_true')
    parser.add_argument('-bags-to-use', '--rosbag-numbers-to-use', type=int, nargs='+', help="Rosbag files to validate on. If empty, all rosbag files will be used.")

    parser.add_argument('--max-union-intersection-time-difference', type=float, default=0.9, help="Max difference between union and intersection or time ragnes where gt and SLAM poses are set.")
    parser.add_argument('--max-time-error', type=float, default=0.01, help="Max time error during matching gt and SLAM poses.")
    parser.add_argument('--max-time-step', type=float, default=0.7, help="Max time step in gt and SLAM poses after matching.")

    parser.add_argument('--skip-running-cartographer', action='store_true')
    parser.add_argument('--skip-trajectory-extraction', action='store_true')
    parser.add_argument('--skip-poses-preparation', action='store_true')
    parser.add_argument('--skip-evaluation', action='store_true')
    return parser


def get_rosbag_files(rosbags_folder, rosbag_numbers_to_use=None):
    rosbag_files = os.listdir(rosbags_folder)
    rosbag_files = [bag for bag in rosbag_files if bag[:2].isdigit() and bag.endswith('bag')]
    if len(rosbag_files) != 21:
        raise RuntimeError
    if rosbag_numbers_to_use:
        rosbag_files = [bag for bag in rosbag_files if int(bag[:2]) in rosbag_numbers_to_use]
    rosbag_files.sort()
    return rosbag_files


def get_robot_name(rosbag_file: str, calibration_version: int, imu_sensor: str):
    robot_name = {'xsens': 'sdbcs_husky', 'atlans': 'sdbcs_husky_atlans_imu'}[imu_sensor]

    pattern = '_2020-'
    date_len = 5
    index = rosbag_file.find(pattern)
    if index == -1:
        raise RuntimeError
    index += len(pattern)
    date = rosbag_file[index:index+date_len]
    if len(date) != date_len:
        raise RuntimeError
    robot_name = robot_name + {'03-16': '_03_16', '03-17': '_03_17', '04-24': '_04_24'}[date]

    if calibration_version > 1:
        robot_name = robot_name + '_v' + str(calibration_version)

    return robot_name


def auto_evaluation_on_SDBCS_Husky(rosbags_folder, out_test_folder, validation_folder, calibration_version=1, \
                                   imu_sensor='xsens', dimension='3d', node_to_use='online', \
                                   get_odom_from_transforms=False, get_odom_from_topic=False, rosbag_numbers_to_use=None, \
                                   max_union_intersection_time_difference=0.9, max_time_error=0.01, max_time_step=0.7, \
                                   skip_running_cartographer=False, skip_trajectory_extraction=False, \
                                   skip_poses_preparation=False, skip_evaluation=False):
    rosbag_files = get_rosbag_files(rosbags_folder, rosbag_numbers_to_use=rosbag_numbers_to_use)

    if not skip_running_cartographer:
        for rosbag_file in rosbag_files:
            rosbag_path = os.path.join(rosbags_folder, rosbag_file)
            robot_name = get_robot_name(rosbag_file, calibration_version, imu_sensor)
            test_name = rosbag_file[:2]
            auto_evaluation(rosbag_path, rosbag_path, '/atlans_odom', out_test_folder, validation_folder, \
                            robot_name=robot_name, dimension=dimension, node_to_use=node_to_use, \
                            get_odom_from_transforms=get_odom_from_transforms, get_odom_from_topic=get_odom_from_topic, test_name=test_name, \
                            max_union_intersection_time_difference=max_union_intersection_time_difference, \
                            max_time_error=max_time_error, max_time_step=max_time_step, \
                            skip_running_cartographer=skip_running_cartographer, skip_trajectory_extraction=True, \
                            skip_poses_preparation=True, skip_evaluation=True)

    if not skip_trajectory_extraction:
        for rosbag_file in rosbag_files:
            rosbag_path = os.path.join(rosbags_folder, rosbag_file)
            robot_name = get_robot_name(rosbag_file, calibration_version, imu_sensor)
            test_name = rosbag_file[:2]
            auto_evaluation(rosbag_path, rosbag_path, '/atlans_odom', out_test_folder, validation_folder, \
                            robot_name=robot_name, dimension=dimension, node_to_use=node_to_use, \
                            get_odom_from_transforms=get_odom_from_transforms, get_odom_from_topic=get_odom_from_topic, test_name=test_name, \
                            max_union_intersection_time_difference=max_union_intersection_time_difference, \
                            max_time_error=max_time_error, max_time_step=max_time_step, \
                            skip_running_cartographer=True, skip_trajectory_extraction=skip_trajectory_extraction, \
                            skip_poses_preparation=True, skip_evaluation=True)

    if not skip_poses_preparation:
        for rosbag_file in rosbag_files:
            rosbag_path = os.path.join(rosbags_folder, rosbag_file)
            robot_name = get_robot_name(rosbag_file, calibration_version, imu_sensor)
            test_name = rosbag_file[:2]
            auto_evaluation(rosbag_path, rosbag_path, '/atlans_odom', out_test_folder, validation_folder, \
                            robot_name=robot_name, dimension=dimension, node_to_use=node_to_use, \
                            get_odom_from_transforms=get_odom_from_transforms, get_odom_from_topic=get_odom_from_topic, test_name=test_name, \
                            max_union_intersection_time_difference=max_union_intersection_time_difference, \
                            max_time_error=max_time_error, max_time_step=max_time_step, \
                            skip_running_cartographer=True, skip_trajectory_extraction=True, \
                            skip_poses_preparation=skip_poses_preparation, skip_evaluation=True)
    
    if not skip_evaluation:
        auto_evaluation('', '', '', out_test_folder, validation_folder, skip_running_cartographer=True, skip_trajectory_extraction=True, \
                        skip_poses_preparation=True, skip_evaluation=skip_evaluation)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    auto_evaluation_on_SDBCS_Husky(**vars(args))
