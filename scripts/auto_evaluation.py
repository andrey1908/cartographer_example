import subprocess
import signal
import os
import argparse
import time


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-test-bag', '--test-rosbag-file', required=True, type=str)
    parser.add_argument('-gt-bag', '--gt-rosbag-file', required=True, type=str, help=".bag file with gt poses")
    parser.add_argument('-gt-topic', '--gt-topic', required=True, type=str, help="topic to read gt poses")
    parser.add_argument('-out-test-fld', '--out-test-folder', required=True, type=str)
    parser.add_argument('-val-fld', '--validation-folder', required=True, type=str)
    parser.add_argument('--imu-frame', required=True, type=str)

    parser.add_argument('-urdf', '--urdf-file', type=str)
    parser.add_argument('-dim', '--dimension', type=str, default='3d', choices=['3d', '2d'], help="Which SLAM to use: 2d or 3d.")
    parser.add_argument('-node', '--node-to-use', type=str, default='online', choices=['online', 'offline'], help="Which launch file to use: cartographer.launch or cartographer_offline.launch.")
    parser.add_argument('-wlc', '--with-loop-closure', action='store_true')
    parser.add_argument('--test-name', type=str, default='test')

    parser.add_argument('--max-union-intersection-time-difference', type=float, default=0.9, help="Max difference between union and intersection or time ragnes where gt and SLAM poses are set.")
    parser.add_argument('--max-time-error', type=float, default=0.01, help="Max time error during matching gt and SLAM poses.")
    parser.add_argument('--max-time-step', type=float, default=0.7, help="Max time step in gt and SLAM poses after matching.")
    return parser


def make_dirs(out_test_folder, validation_folder):
    os.makedirs(out_test_folder, exist_ok=True)
    os.makedirs(os.path.join(validation_folder, 'gt'), exist_ok=True)
    os.makedirs(os.path.join(validation_folder, 'results'), exist_ok=True)


def run_cartographer(rosbag_filename, out_pbstream_filename, urdf_filename='\"\"', dimension='3d', node_to_use='online', with_loop_closure=False, print_command=False):
    if node_to_use == 'online':
        command = "roslaunch cartographer_example cartographer.launch    urdf_filename:={}    \
dim:={}    publish_occupancy_grid:=false".format(urdf_filename, dimension)
    elif node_to_use == 'offline':
        command = "roslaunch cartographer_example cartographer_offline.launch    bag_filenames:={}    urdf_filenames:={}    \
save_state_filename:={}    dim:={}".format(rosbag_filename, urdf_filename, out_pbstream_filename, dimension)
    else:
        raise(RuntimeError)
    if print_command:
        print('\n\n\n' + command + '\n')
    process = subprocess.Popen(command.split())
    
    if node_to_use == 'online':
        time.sleep(3)
        rosbag_play_command = "rosbag play --clock {}".format(rosbag_filename)
        rosbag_play = subprocess.Popen(rosbag_play_command.split())
        rosbag_play.communicate()
        assert(rosbag_play.returncode == 0)
        time.sleep(1)
        
        if with_loop_closure:
            finish_trajectory_command = "rosservice call /cartographer/finish_trajectory 0"
            finish_trajectory = subprocess.Popen(finish_trajectory_command.split())
            finish_trajectory.communicate()
            assert(finish_trajectory.returncode == 0)
            
            trajectory_state = 0
            while trajectory_state == 0:
                get_trajectory_states_command = "rosservice call /cartographer/get_trajectory_states"
                trajectory_states = subprocess.check_output(get_trajectory_states_command.split()).decode()
                idx = trajectory_states.find('trajectory_state:')
                idx += trajectory_states[idx:].find('[')
                trajectory_state = int(trajectory_states[idx+1:idx+2])
            time.sleep(1)
                
        write_state_command = "rosservice call /cartographer/write_state {} true".format(out_pbstream_filename)
        write_state = subprocess.Popen(write_state_command.split())
        write_state.communicate()
        assert(write_state.returncode == 0)
        process.terminate()
        
    process.communicate()
    assert(process.returncode == 0)
    return command


def retrieve_SLAM_trajectory(pbstream_filename, out_results_rosbag_filename, imu_frame, print_command=False):
    command = "rosrun cartographer_ros cartographer_dev_pbstream_trajectories_to_rosbag    -input {}    -output {}    \
-tracking_frame {}".format(pbstream_filename, out_results_rosbag_filename, imu_frame)
    if print_command:
        print('\n\n\n' + command + '\n')
    process = subprocess.Popen(command.split())
    process.communicate()
    assert(process.returncode == 0)
    return command


def prepare_poses_for_evaluation(gt_rosbag_filename, gt_topic, results_rosbag_filename, results_topic, \
                                 out_gt_poses_filename, out_results_poses_filename, transforms_source_filename='\'\'', \
                                 out_trajectories_rosbag_filename='\'\'', max_union_intersection_time_difference=0.9, \
                                 max_time_error=0.01, max_time_step=0.7, print_command=False):
    command = "python /home/cds-jetson-host/catkin_ws/src/ros_utils/scripts/prepare_poses_for_evaluation.py    \
-gt-bag {}    -gt-topic {}    -res-bag {}    -res-topic {}     -out-gt {}    -out-res {}    \
-transforms-source {}    -out-trajectories {}    --max-union-intersection-time-difference {}    --max-time-error {}    \
--max-time-step {}".format(gt_rosbag_filename, gt_topic, results_rosbag_filename, results_topic, out_gt_poses_filename, out_results_poses_filename, \
                           transforms_source_filename, out_trajectories_rosbag_filename, \
                           max_union_intersection_time_difference, max_time_error, max_time_step)
    if print_command:
        print('\n\n\n' + command + '\n')
    process = subprocess.Popen(command.split())
    process.communicate()
    assert(process.returncode == 0)
    return command


def run_evaluation(validation_folder, projection='xy', print_command=False):
    gt_poses_folder = os.path.abspath(os.path.join(validation_folder, 'gt'))
    results_poses_folder = os.path.abspath(os.path.join(validation_folder, 'results'))
    out_folder = os.path.abspath(os.path.join(validation_folder, 'output_{}'.format(projection)))

    command = "python /home/cds-jetson-host/slam_validation/evaluate_poses.py    --dir_gt {}    --dir_result {}    \
--dir_output {}    --gt_format kitti    --result_format kitti    \
--projection {}".format(gt_poses_folder, results_poses_folder, out_folder, projection)
    if print_command:
        print('\n\n\n' + command + '\n')
    process = subprocess.Popen(command.split())
    process.communicate()
    assert(process.returncode == 0)
    return command


def auto_evaluation(test_rosbag_file, gt_rosbag_file, gt_topic, out_test_folder, validation_folder, imu_frame, \
                    urdf_file=None, dimension='3d', node_to_use='online', with_loop_closure=False, test_name='test', \
                    max_union_intersection_time_difference=0.9, max_time_error=0.01, max_time_step=0.7):
    make_dirs(out_test_folder, validation_folder)
    log = str()

    # Run cartographer to generate a map
    test_rosbag_filename = os.path.abspath(test_rosbag_file)
    out_pbstream_filename = os.path.abspath(os.path.join(out_test_folder, '{}.pbstream'.format(test_name)))
    urdf_filename = os.path.abspath(urdf_file) if urdf_file else '\'\''
    command = run_cartographer(test_rosbag_filename, out_pbstream_filename, urdf_filename=urdf_filename, \
                               dimension=dimension, node_to_use=node_to_use, with_loop_closure=with_loop_closure, print_command=True)
    log += command + '\n\n\n'
    
    # Retrieve SLAM trajectories from cartographer map
    out_results_rosbag_filename = os.path.abspath(os.path.join(out_test_folder, '{}.bag'.format(test_name)))
    command = retrieve_SLAM_trajectory(out_pbstream_filename, out_results_rosbag_filename, imu_frame, print_command=True)
    log += command + '\n\n\n'

    # Prepare poses in kitti format for evaluation
    gt_rosbag_filename = os.path.abspath(gt_rosbag_file)
    transforms_source_filename = urdf_filename if urdf_file else test_rosbag_filename
    out_gt_poses_filename = os.path.abspath(os.path.join(validation_folder, 'gt', '{}.txt'.format(test_name)))
    out_results_poses_filename = os.path.abspath(os.path.join(validation_folder, 'results', '{}.txt'.format(test_name)))
    out_trajectories_rosbag_filename = os.path.abspath(os.path.join(out_test_folder, '{}_trajectories.bag'.format(test_name)))
    command = prepare_poses_for_evaluation(gt_rosbag_filename, gt_topic, out_results_rosbag_filename, 'trajectory_0', \
                                           out_gt_poses_filename, out_results_poses_filename, \
                                           transforms_source_filename, out_trajectories_rosbag_filename, \
                                           max_union_intersection_time_difference=max_union_intersection_time_difference, \
                                           max_time_error=max_time_error, max_time_step=max_time_step, print_command=True)
    log += command + '\n\n\n'
    
    # Run evaluation
    for projection in ['xy', 'xz', 'yz']:
        command = run_evaluation(validation_folder, projection=projection, print_command=True)
        log += command + '\n\n\n'
    
    print(log)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    auto_evaluation(**vars(args))
