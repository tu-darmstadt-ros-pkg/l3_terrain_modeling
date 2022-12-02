//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#pragma once

#include <sensor_msgs/PointCloud2.h>

#include <l3_terrain_model_generator/plugins/base/point_cloud_sensor_plugin.h>

namespace l3_terrain_modeling
{
/**
 * @brief The GenericLidarSensor represents a generic lidar sensor that processes the
 * sensor data as Pointcloud.
 *
 * @param sensor_frame (default: "lidar") Frame in which the sensor is localized
 * @param map_frame (default: "map") Frame in which the data should be represented
 * @param auto_update_sensor_pose (default: true) Tries to update sensor pose based on tf
 * @param rate (default: 0.0) Maximum rate at which the input data should be processed, 0.0 for no limit
 * @param input_data_name (default: "cloud") Data name as found in the DataManager
 * @param point_type Point type the sensor should process. Possible values: "PointXYZ", "PointXYZRGB"
 */
class GenericLidarSensor : public PointCloudSensorPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<GenericLidarSensor> Ptr;
  typedef l3::SharedPtr<const GenericLidarSensor> ConstPtr;

  GenericLidarSensor();

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

protected:
  void pointcloudCb(const sensor_msgs::PointCloud2& msg);

  ros::Subscriber pointcloud_sub_;
};
}  // namespace l3_terrain_modeling
