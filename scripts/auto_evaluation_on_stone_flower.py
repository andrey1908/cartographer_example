import os
import argparse
from auto_evaluation import auto_evaluation


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-bags-fld', '--rosbags-folder', required=True, type=str, help="Folder with rewritten rosbag files from stone flower dataset. The folder may contain other files.")
    parser.add_argument('-out-test-fld', '--out-test-folder', required=True, type=str, help="Folder to save intermediate results.")
    parser.add_argument('-val-fld', '--validation-folder', required=True, type=str, help="Folder to save kitti poses for evaluation.")

    parser.add_argument('-imu', '--imu-sensor', type=str, default='xsens', choices=['xsens', 'left', 'central', 'right'], help="What imu to use.")
    parser.add_argument('-dim', '--dimension', type=str, default='3d', choices=['3d', '2d'], help="Which SLAM to use: 2d or 3d.")
    parser.add_argument('-node', '--node-to-use', type=str, default='online', choices=['online', 'offline'], help="Cartographer mode.")

    parser.add_argument('--max-union-intersection-time-difference', type=float, default=0.9, help="Max difference between union and intersection or time ragnes where gt and SLAM poses are set.")
    parser.add_argument('--max-time-error', type=float, default=0.01, help="Max time error during matching gt and SLAM poses.")
    parser.add_argument('--max-time-step', type=float, default=0.7, help="Max time step in gt and SLAM poses after matching.")

    parser.add_argument('--skip-running-cartographer', action='store_true')
    parser.add_argument('--skip-trajectory-extraction', action='store_true')
    parser.add_argument('--skip-poses-preparation', action='store_true')
    parser.add_argument('--skip-evaluation', action='store_true')
    return parser


def get_rosbag_files_groups(rosbags_folder):
    rosbag_files_all = os.listdir(rosbags_folder)
    rosbag_files = [bag for bag in rosbag_files_all if bag.startswith('stone_flower') and bag.find('add') == -1 and bag.endswith('rewritten.bag')]
    rosbag_files_add = [bag for bag in rosbag_files_all if bag.startswith('stone_flower_add') and bag.endswith('rewritten.bag')]
    if len(rosbag_files) != 5:
        raise RuntimeError
    if len(rosbag_files_add) != 12:
        raise RuntimeError
    rosbag_files.sort()
    rosbag_files_add.sort()
    rosbag_files_groups = [rosbag_files, rosbag_files_add]
    return rosbag_files_groups


def auto_evaluation_on_stone_flower(rosbags_folder, out_test_folder, validation_folder, \
                                   imu_sensor='xsens', dimension='3d', node_to_use='online', \
                                   max_union_intersection_time_difference=0.9, max_time_error=0.01, max_time_step=0.7, \
                                   skip_running_cartographer=False, skip_trajectory_extraction=False, \
                                   skip_poses_preparation=False, skip_evaluation=False):
    rosbag_files_groups = get_rosbag_files_groups(rosbags_folder)
    robot_name = {'xsens': 'sweeper', 'left': 'sweeper_left_imu', 'central': 'sweeper_central_imu', 'right': 'sweeper_right_imu'}[imu_sensor]

    if not skip_running_cartographer:
        for rosbag_files in rosbag_files_groups:
            rosbag_paths = list(map(lambda rosbag_file: os.path.join(rosbags_folder, rosbag_file), rosbag_files))
            test_name = 'stone_flower' if rosbag_files[0].find('add') == -1 else 'stone_flower_add'
            auto_evaluation(rosbag_paths, rosbag_paths, '/xsens/odometry', out_test_folder, validation_folder, \
                            robot_name=robot_name, dimension=dimension, node_to_use=node_to_use, test_name=test_name, \
                            max_union_intersection_time_difference=max_union_intersection_time_difference, \
                            max_time_error=max_time_error, max_time_step=max_time_step, \
                            skip_running_cartographer=skip_running_cartographer, skip_trajectory_extraction=True, \
                            skip_poses_preparation=True, skip_evaluation=True)

    if not skip_trajectory_extraction:
        for rosbag_files in rosbag_files_groups:
            rosbag_paths = list(map(lambda rosbag_file: os.path.join(rosbags_folder, rosbag_file), rosbag_files))
            test_name = 'stone_flower' if rosbag_files[0].find('add') == -1 else 'stone_flower_add'
            auto_evaluation(rosbag_paths, rosbag_paths, '/xsens/odometry', out_test_folder, validation_folder, \
                            robot_name=robot_name, dimension=dimension, node_to_use=node_to_use, test_name=test_name, \
                            max_union_intersection_time_difference=max_union_intersection_time_difference, \
                            max_time_error=max_time_error, max_time_step=max_time_step, \
                            skip_running_cartographer=True, skip_trajectory_extraction=skip_trajectory_extraction, \
                            skip_poses_preparation=True, skip_evaluation=True)

    if not skip_poses_preparation:
        for rosbag_files in rosbag_files_groups:
            rosbag_paths = list(map(lambda rosbag_file: os.path.join(rosbags_folder, rosbag_file), rosbag_files))
            test_name = 'stone_flower' if rosbag_files[0].find('add') == -1 else 'stone_flower_add'
            auto_evaluation(rosbag_paths, rosbag_paths, '/xsens/odometry', out_test_folder, validation_folder, \
                            robot_name=robot_name, dimension=dimension, node_to_use=node_to_use, test_name=test_name, \
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
    auto_evaluation_on_stone_flower(**vars(args))
