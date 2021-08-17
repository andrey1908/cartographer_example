import subprocess
import os
import argparse
import time
from auto_evaluation import run_cartographer, retrieve_SLAM_trajectory, prepare_poses_for_evaluation, run_evaluation


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-bags-fld', '--rosbags-folder', required=True, type=str, help="Folder with rosbag files from SDBCS_Husky dataset. The folder may contain other files.")
    parser.add_argument('-out-test-fld', '--out-test-folder', required=True, type=str, help="Folder to save intermediate results.")
    parser.add_argument('-urdf-fld', '--urdf-folder', required=True, type=str, help="Folder with urdf files.")
    parser.add_argument('-val-fld', '--validation-folder', required=True, type=str, help="Folder to save kitti poses for evaluation.")

    parser.add_argument('-urdf-v', '--urdf-version', type=int, default=1, choices=[1, 2, 3], help="Version of calibraion.")
    parser.add_argument('-imu', '--imu-sensor', type=str, default='xsens', choices=['xsens', 'atlans'], help="Imu that is used in launch file for running cartographer.")
    parser.add_argument('-dim', '--dimension', type=str, default='3d', choices=['3d', '2d'], help="Which SLAM to use: 2d or 3d.")
    parser.add_argument('-node', '--node-to-use', type=str, default='online', choices=['online', 'offline'], help="Which launch file to use: cartographer.launch or cartographer_offline.launch.")
    parser.add_argument('-wlc', '--with-loop-closure', action='store_true')
    parser.add_argument('-bags-to-use', '--rosbag-numbers-to-use', type=int, nargs='+', help="Rosbag files to validate on. If empty, all rosbag files will be used.")

    parser.add_argument('--max-union-intersection-time-difference', type=float, default=0.9, help="Max difference between union and intersection or time ragnes where gt and SLAM poses are set.")
    parser.add_argument('--max-time-error', type=float, default=0.01, help="Max time error during matching gt and SLAM poses.")
    parser.add_argument('--max-time-step', type=float, default=0.7, help="Max time step in gt and SLAM poses after matching.")
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


def get_urdf_filename(urdf_folder, rosbag_file, urdf_version):
    pattern = '_2020-'
    date_len = 5
    urdf_filename_template = 'SDBCS_Husky_{}{}.urdf'
    
    index = rosbag_file.find(pattern)
    assert(index != -1)
    index += len(pattern)
    date = rosbag_file[index:index+date_len]
    assert(len(date) == date_len)
    
    urdf_version_suffix = {1: '', 2: '_v2', 3: '_v3'}[urdf_version]
    urdf_file = urdf_filename_template.format(date, urdf_version_suffix)
    urdf_filename = os.path.abspath(os.path.join(urdf_folder, urdf_file))
    
    return urdf_filename


def auto_evaluation_on_SDBCS_Husky(rosbags_folder, out_test_folder, urdf_folder, validation_folder, urdf_version=1, \
                                   imu_sensor='xsens', dimension='3d', node_to_use='online', with_loop_closure=False, rosbag_numbers_to_use=None, \
                                   max_union_intersection_time_difference=0.9, max_time_error=0.01, max_time_step=0.7):
    make_dirs(out_test_folder, validation_folder)
    rosbag_files = get_rosbag_filenames(rosbags_folder, rosbag_numbers_to_use=rosbag_numbers_to_use)
    imu_frame = {'xsens': 'imu', 'atlans': 'isns_link'}[imu_sensor]
    log = str()

    # Run cartographer to generate maps
    for rosbag_file in rosbag_files:
        rosbag_filename = os.path.abspath(os.path.join(rosbags_folder, rosbag_file))
        out_pbstream_filename = os.path.abspath(os.path.join(out_test_folder, '{}.pbstream'.format(rosbag_file[:2])))
        urdf_filename = get_urdf_filename(urdf_folder, rosbag_file, urdf_version)
        command = run_cartographer(rosbag_filename, out_pbstream_filename, robot_name='sdbcs_husky', urdf_filename=urdf_filename, \
                                   dimension=dimension, node_to_use=node_to_use, with_loop_closure=with_loop_closure, print_command=True)
        log += command + '\n\n\n'

    # Retrieve SLAM trajectories from cartographer maps
    for rosbag_file in rosbag_files:
        pbstream_filename = os.path.abspath(os.path.join(out_test_folder, '{}.pbstream'.format(rosbag_file[:2])))
        out_results_rosbag_filename = os.path.abspath(os.path.join(out_test_folder, '{}.bag'.format(rosbag_file[:2])))
        command = retrieve_SLAM_trajectory(pbstream_filename, out_results_rosbag_filename, imu_frame, print_command=True)
        log += command + '\n\n\n'

    # Prepare poses in kitti format for evaluation
    for rosbag_file in rosbag_files:
        gt_rosbag_filename = os.path.abspath(os.path.join(rosbags_folder, rosbag_file))
        results_rosbag_filename = os.path.abspath(os.path.join(out_test_folder, '{}.bag'.format(rosbag_file[:2])))
        urdf_filename = get_urdf_filename(urdf_folder, rosbag_file, urdf_version)
        out_gt_poses_filename = os.path.abspath(os.path.join(validation_folder, 'gt', '{}.txt'.format(rosbag_file[:2])))
        out_results_poses_filename = os.path.abspath(os.path.join(validation_folder, 'results', '{}.txt'.format(rosbag_file[:2])))
        out_trajectories_rosbag_filename = os.path.abspath(os.path.join(out_test_folder, '{}_trajectories.bag'.format(rosbag_file[:2])))
        command = prepare_poses_for_evaluation(gt_rosbag_filename, '/atlans_odom', results_rosbag_filename, 'trajectory_0', \
                                               out_gt_poses_filename, out_results_poses_filename, urdf_filename, out_trajectories_rosbag_filename, \
                                               max_union_intersection_time_difference=max_union_intersection_time_difference, \
                                               max_time_error=max_time_error, max_time_step=max_time_step, print_command=True)
        log += command + '\n\n\n'

    # Run evaluation
    for projection in ['xy', 'xz', 'yz']:
        gt_poses_folder = os.path.abspath(os.path.join(validation_folder, 'gt'))
        results_poses_folder = os.path.abspath(os.path.join(validation_folder, 'results'))
        out_folder = os.path.abspath(os.path.join(validation_folder, 'output_{}'.format(projection)))
        command = run_evaluation(validation_folder, projection=projection, print_command=True)
        log += command + '\n\n\n'
    
    print(log)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    auto_evaluation_on_SDBCS_Husky(**vars(args))
