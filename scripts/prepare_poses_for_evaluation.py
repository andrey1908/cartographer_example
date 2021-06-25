#!/usr/bin/env python3
from re import A
import rosbag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tqdm import tqdm
import argparse
import numpy as np
from transforms3d.quaternions import quat2mat


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-bag-gt', '--rosbag-gt-file', required=True, type=str, help=".bag file with gt poses")
    parser.add_argument('-gt-topic', '--gt-topic', required=True, type=str, help="topic to read gt poses")

    parser.add_argument('-bag-res', '--rosbag-results-file', required=True, type=str, help=".bag file with SLAM trajectory")
    parser.add_argument('-res-topic', '--results-topic', required=True, type=str, help="topic to read SLAM trajectory")

    parser.add_argument('-out-gt', '--out-gt-file', required=True, type=str, help="output file with gt poses in kitti format")
    parser.add_argument('-out-res', '--out-results-file', required=True, type=str, help="output file with SLAM poses in kitti format")
    return parser


def find_mutual_indexes(A, B):
    swapped = False
    if len(A) > len(B):
        A, B = B, A
        swapped = True
    
    A_indexes = list()
    B_indexes = list()
    for A_index, a in enumerate(A):
        B_index = np.argmin(np.abs(B - a))
        A_indexes.append(A_index)
        B_indexes.append(B_index)

    if swapped:
        A_indexes, B_indexes = B_indexes, A_indexes
    return A_indexes, B_indexes


def pose_to_matrix(position, orientation):
    pose = np.zeros((4, 4))
    pose[:3, :3] = quat2mat([orientation.w, orientation.x, orientation.y, orientation.z])
    pose[:3, 3] = [position.x, position.y, position.z]
    pose[3, 3] = 1
    return pose


def ros_pose_to_matrix(ros_pose):
    if hasattr(ros_pose, 'pose'):
        return pose_to_matrix(ros_pose.pose.pose.position, ros_pose.pose.pose.orientation)
    if hasattr(ros_pose, 'transform'):
        return pose_to_matrix(ros_pose.transform.translation, ros_pose.transform.rotation)
    raise(TypeError("Unknown type {}".format(type(ros_pose))))


def prepare_poses_for_evaluation(rosbag_gt_file, gt_topic, rosbag_results_file, results_topic, out_gt_file, out_results_file):
    print("Loading bags...")
    bag_gt = rosbag.Bag(rosbag_gt_file)
    bag_results = rosbag.Bag(rosbag_results_file)
    print("Bags loaded")

    gt_timestamps = list()
    gt_messages = list()
    results_timestamps = list()
    results_messages = list()
    for topic, msg, t in tqdm(bag_gt.read_messages(topics=[gt_topic]), total=bag_gt.get_message_count(topic_filters=[gt_topic])):
        gt_timestamps.append(msg.header.stamp.to_sec())
        gt_messages.append(msg)
    for topic, msg, t in tqdm(bag_results.read_messages(topics=[results_topic]), total=bag_results.get_message_count(topic_filters=[results_topic])):
        results_timestamps.append(msg.header.stamp.to_sec())
        results_messages.append(msg)

    gt_indexes, results_indexes = find_mutual_indexes(np.array(gt_timestamps), np.array(results_timestamps))

    with open(out_gt_file, 'w') as f:
        origin_pose = None
        for i, gt_index in enumerate(gt_indexes):
            pose = ros_pose_to_matrix(gt_messages[gt_index])
            # remember the first pose
            if origin_pose is None:
                origin_pose = pose
                origin_pose_inv = np.linalg.inv(origin_pose)
            # move odometry to the origin
            pose = origin_pose_inv @ pose
            out_pose_str = '{:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e}'.format(
                    pose[0][0], pose[0][1], pose[0][2], pose[0][3],
                    pose[1][0], pose[1][1], pose[1][2], pose[1][3],
                    pose[2][0], pose[2][1], pose[2][2], pose[2][3])
            if i != len(gt_indexes) - 1:
                out_pose_str = out_pose_str + '\n'
            f.write(out_pose_str)

    with open(out_results_file, 'w') as f:
        origin_pose = None
        for i, results_index in enumerate(results_indexes):
            pose = ros_pose_to_matrix(results_messages[results_index])
            # remember the first pose
            if origin_pose is None:
                origin_pose = pose
                origin_pose_inv = np.linalg.inv(origin_pose)
            # move odometry to the origin
            pose = origin_pose_inv @ pose
            out_pose_str = '{:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e} {:.6e}'.format(
                    pose[0][0], pose[0][1], pose[0][2], pose[0][3],
                    pose[1][0], pose[1][1], pose[1][2], pose[1][3],
                    pose[2][0], pose[2][1], pose[2][2], pose[2][3])
            if i != len(results_indexes) - 1:
                out_pose_str = out_pose_str + '\n'
            f.write(out_pose_str)

    bag_gt.close()
    bag_results.close()

    print("Finished!")


if __name__ == '__main__':
    parser = build_parser()
    args = parser.parse_args()
    prepare_poses_for_evaluation(**vars(args))
    
