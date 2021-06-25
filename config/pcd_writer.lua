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

-- WARNING: we create a lot of X-Rays of a potentially large space in this
-- pipeline. For example, running over the
-- cartographer_paper_deutsches_museum.bag requires ~25GiB of memory. You can
-- reduce this by writing fewer X-Rays or upping VOXEL_SIZE - which is the size
-- of a pixel in a X-Ray.

options = {
  tracking_frame = "realsense_back_imu_optical_frame",
  pipeline = {
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 200.,
    },
    {
      action = "dump_num_points",
    },
    {
      action = "color_points",
      frame_id = "",
      color = { 255., 255., 255. },
    },
    {
      action = "write_pcd",
      filename = "",
    },
  }
}

return options
