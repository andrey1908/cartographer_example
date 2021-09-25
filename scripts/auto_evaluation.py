import rospy
import rospkg
import subprocess
import os
import argparse
import time
import xml.etree.ElementTree as ET


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-test-bags', '--test-rosbag-files', required=True, type=str, nargs='+', help=".bag files to test on")
    parser.add_argument('-gt-bags', '--gt-rosbag-files', required=True, type=str, nargs='+', help=".bag files with gt poses")
    parser.add_argument('-gt-topic', '--gt-topic', required=True, type=str, help="topic to read gt poses")
    parser.add_argument('-out-test-fld', '--out-test-folder', required=True, type=str)
    parser.add_argument('-val-fld', '--validation-folder', required=True, type=str)

    parser.add_argument('-robot', '--robot-name', type=str, default='default')
    parser.add_argument('-dim', '--dimension', type=str, default='3d', choices=['3d', '2d'], help="Which SLAM to use: 2d or 3d.")
    parser.add_argument('-node', '--node-to-use', type=str, default='online', choices=['online', 'offline'], help="Cartographer mode.")
    parser.add_argument('--test-name', type=str, default='test')

    parser.add_argument('--max-union-intersection-time-difference', type=float, default=0.9, help="Max difference between union and intersection or time ragnes where gt and SLAM poses are set.")
    parser.add_argument('--max-time-error', type=float, default=0.01, help="Max time error during matching gt and SLAM poses.")
    parser.add_argument('--max-time-step', type=float, default=0.7, help="Max time step in gt and SLAM poses after matching.")

    parser.add_argument('--skip-running-cartographer', action='store_true')
    parser.add_argument('--skip-trajectory-extraction', action='store_true')
    parser.add_argument('--skip-poses-preparation', action='store_true')
    parser.add_argument('--skip-evaluation', action='store_true')
    return parser


def make_dirs(out_test_folder, validation_folder):
    os.makedirs(out_test_folder, exist_ok=True)
    os.makedirs(os.path.join(validation_folder, 'gt'), exist_ok=True)
    os.makedirs(os.path.join(validation_folder, 'results'), exist_ok=True)


def run_cartographer(rosbag_filenames, out_pbstream_filename, robot_name='default', dimension='3d', \
                     node_to_use='online', print_command=False):
    if isinstance(rosbag_filenames, str):
        rosbag_filenames = [rosbag_filenames]
    if node_to_use not in ['online', 'offline']:
        raise RuntimeError()
    if node_to_use == 'offline':
        raise NotImplementedError('Launch file for offline node is out of date.')

    if node_to_use == 'online':
        command = "roslaunch cartographer_example cartographer.launch   robot:={}    dim:={}    \
publish_occupancy_grid:=false".format(robot_name, dimension)
    if print_command:
        print('\n\n\n' + command + '\n')
    process = subprocess.Popen(command.split())
    
    if node_to_use == 'online':
        rospy.wait_for_service('/cartographer/write_state')
        time.sleep(3)
        rosbag_play_command = "rosbag play --clock {}".format(' '.join(rosbag_filenames))
        rosbag_play = subprocess.Popen(rosbag_play_command.split())
        rosbag_play.communicate()
        assert(rosbag_play.returncode == 0)
        time.sleep(1)
        
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


def robot_name_to_imu_frame(robot: str):
    dim = '3d'  # imu frame does not depend on dim, but this variable is needed to evaluate expression in eval()
    rospack = rospkg.RosPack()
    cartographer_example_folder = rospack.get_path('cartographer_example')
    tree = ET.parse(os.path.join(cartographer_example_folder, 'launch/cartographer.launch'))
    root = tree.getroot()
    for elem in root:
        if elem.tag != 'arg':
            continue
        if elem.attrib['name'] != 'config_file':
            continue
        expression = elem.attrib.get('if')
        if expression is None:
            continue
        expression = expression.replace('$(eval', '')
        expression = expression[:expression.rfind(')')]
        if not eval(expression):
            continue
        config_filename = os.path.join(cartographer_example_folder, 'config', elem.attrib['value'])
        with open(config_filename, 'r') as f:
            lines = f.readlines()
        for line in lines:
            if line.find('options.tracking_frame') == -1:
                continue
            start_idx = line.find('=')
            if start_idx == -1:
                raise RuntimeError
            start_idx += 1
            end_idx = len(line) - 1
            imu_frame = eval(line[start_idx:end_idx])
            if not isinstance(imu_frame, str):
                raise RuntimeError
            return imu_frame
    raise RuntimeError


def extract_SLAM_trajectories(pbstream_filename, out_results_rosbag_filename, robot_name='default', print_command=False):
    imu_frame = robot_name_to_imu_frame(robot_name)
    command = "rosrun cartographer_ros pbstream_trajectories_to_rosbag    -input {}    -output {}    \
-tracking_frame {}".format(pbstream_filename, out_results_rosbag_filename, imu_frame)
    if print_command:
        print('\n\n\n' + command + '\n')
    process = subprocess.Popen(command.split())
    process.communicate()
    assert(process.returncode == 0)
    return command


def robot_name_to_transforms_source_filename(robot: str):
    rospack = rospkg.RosPack()
    cartographer_example_folder = rospack.get_path('cartographer_example')
    tree = ET.parse(os.path.join(cartographer_example_folder, 'launch/cartographer.launch'))
    root = tree.getroot()
    for elem in root:
        if elem.tag != 'include':
            continue
        expression = elem.attrib.get('if')
        if expression is None:
            continue
        expression = expression.replace('$(eval', '')
        expression = expression[:expression.rfind(')')]
        if not eval(expression):
            continue
        transforms_source_file = elem.attrib['file'].replace('$(find cartographer_example)/', '')
        transforms_source_filename = os.path.join(cartographer_example_folder, transforms_source_file)
        return transforms_source_filename
    raise RuntimeError


def prepare_poses_for_evaluation(gt_rosbag_filenames, gt_topic, results_rosbag_filenames, results_topic, \
                                 out_gt_poses_filename, out_results_poses_filename, robot_name='default', \
                                 out_trajectories_rosbag_filename='\'\'', max_union_intersection_time_difference=0.9, \
                                 max_time_error=0.01, max_time_step=0.7, print_command=False):
    if isinstance(gt_rosbag_filenames, str):
        gt_rosbag_filenames = [gt_rosbag_filenames]
    if isinstance(results_rosbag_filenames, str):
        results_rosbag_filenames = [results_rosbag_filenames]
    transforms_source_filename = robot_name_to_transforms_source_filename(robot_name)
    command = "rosrun ros_utils prepare_poses_for_evaluation.py    \
-gt-bags {}    -gt-topic {}    -res-bag {}    -res-topic {}     -out-gt {}    -out-res {}    \
-transforms-source {}    -out-trajectories {}    --max-union-intersection-time-difference {}    --max-time-error {}    \
--max-time-step {}".format(' '.join(gt_rosbag_filenames), gt_topic, ' '.join(results_rosbag_filenames), results_topic, out_gt_poses_filename, \
                           out_results_poses_filename, transforms_source_filename, out_trajectories_rosbag_filename, \
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


def auto_evaluation(test_rosbag_files, gt_rosbag_files, gt_topic, out_test_folder, validation_folder, \
                    robot_name='default', dimension='3d', node_to_use='online', test_name='test', \
                    max_union_intersection_time_difference=0.9, max_time_error=0.01, max_time_step=0.7, \
                    skip_running_cartographer=False, skip_trajectory_extraction=False, skip_poses_preparation=False, skip_evaluation=False):
    if isinstance(test_rosbag_files, str):
        test_rosbag_files = [test_rosbag_files]
    if isinstance(gt_rosbag_files, str):
        gt_rosbag_files = [gt_rosbag_files]
    make_dirs(out_test_folder, validation_folder)
    log = str()

    # Run cartographer to generate a map
    test_rosbag_filenames = list(map(os.path.abspath, test_rosbag_files))
    out_pbstream_filename = os.path.abspath(os.path.join(out_test_folder, '{}.pbstream'.format(test_name)))
    if not skip_running_cartographer:
        command = run_cartographer(test_rosbag_filenames, out_pbstream_filename, robot_name=robot_name, dimension=dimension, \
                                   node_to_use=node_to_use, print_command=True)
        log += command + '\n\n\n'
    
    # Extract SLAM trajectories from cartographer map
    out_results_rosbag_filename = os.path.abspath(os.path.join(out_test_folder, '{}.bag'.format(test_name)))
    if not skip_trajectory_extraction:
        command = extract_SLAM_trajectories(out_pbstream_filename, out_results_rosbag_filename, robot_name, print_command=True)
        log += command + '\n\n\n'

    # Prepare poses in kitti format for evaluation
    gt_rosbag_filenames = list(map(os.path.abspath, gt_rosbag_files))
    out_global_gt_poses_filename = os.path.abspath(os.path.join(validation_folder, 'gt', 'global_{}.txt'.format(test_name)))
    out_global_results_poses_filename = os.path.abspath(os.path.join(validation_folder, 'results', 'global_{}.txt'.format(test_name)))
    out_local_gt_poses_filename = os.path.abspath(os.path.join(validation_folder, 'gt', 'local_{}.txt'.format(test_name)))
    out_local_results_poses_filename = os.path.abspath(os.path.join(validation_folder, 'results', 'local_{}.txt'.format(test_name)))
    out_trajectories_rosbag_filename = os.path.abspath(os.path.join(out_test_folder, '{}_trajectories.bag'.format(test_name)))
    if not skip_poses_preparation:
        command = prepare_poses_for_evaluation(gt_rosbag_filenames, gt_topic, out_results_rosbag_filename, 'global_trajectory_0', \
                                               out_global_gt_poses_filename, out_global_results_poses_filename, \
                                               robot_name, out_trajectories_rosbag_filename, \
                                               max_union_intersection_time_difference=max_union_intersection_time_difference, \
                                               max_time_error=max_time_error, max_time_step=max_time_step, print_command=True)
        log += command + '\n\n\n'
        command = prepare_poses_for_evaluation(gt_rosbag_filenames, gt_topic, out_results_rosbag_filename, 'local_trajectory_0', \
                                               out_local_gt_poses_filename, out_local_results_poses_filename, \
                                               robot_name, out_trajectories_rosbag_filename, \
                                               max_union_intersection_time_difference=max_union_intersection_time_difference, \
                                               max_time_error=max_time_error, max_time_step=max_time_step, print_command=True)
        log += command + '\n\n\n'
    
    # Run evaluation
    if not skip_evaluation:
        for projection in ['xy', 'xz', 'yz']:
            command = run_evaluation(validation_folder, projection=projection, print_command=True)
            log += command + '\n\n\n'
    
    print(log)


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    auto_evaluation(**vars(args))
