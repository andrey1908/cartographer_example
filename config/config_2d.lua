-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu",
  published_frame = "isns_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  publish_tracked_pose = true,
  use_pose_extrapolator = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- Range filter --
TRAJECTORY_BUILDER_2D.min_range = 1.
MAX_2D_RANGE = 120.
TRAJECTORY_BUILDER_2D.max_range = MAX_2D_RANGE
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = MAX_2D_RANGE
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = MAX_2D_RANGE
TRAJECTORY_BUILDER_2D.min_z = -0.4
TRAJECTORY_BUILDER_2D.max_z = 1.

-- Voxel filter --
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 2.
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 2.
POSE_GRAPH.constraint_builder.min_score = 0.4
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.45

-- Motion filter --
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004

-- Local SLAM --
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 3.  -- '5.' shows bad results on 18 track
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.  -- '4e2' is worse than this

-- Global SLAM --
MAP_BUILDER.num_background_threads = 4
POSE_GRAPH.optimize_every_n_nodes = 50
POSE_GRAPH.constraint_builder.sampling_ratio = 0.02
--POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 4

-- Logs --
POSE_GRAPH.log_residual_histograms = false
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.optimization_problem.log_solver_summary = false

-- Localization mode --
--[[
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
--]]

return options
