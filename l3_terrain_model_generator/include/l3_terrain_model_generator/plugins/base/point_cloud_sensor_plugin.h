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

#include <ros/ros.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <l3_terrain_model_generator/utils/pcl/pcl_data_handle.h>
#include <l3_terrain_model_generator/utils/pcl/pcl_utils.h>
#include <l3_terrain_model_generator/plugins/base/sensor_plugin.h>

namespace l3_terrain_modeling
{
/**
 * @brief The PointCloudSensorPlugin represents a generic sensor that processes the
 * sensor data as Pointcloud.
 * @param map_frame (default: "map") Frame in which the data is represented
 * @param input_data_name (default: "cloud") Data name as found in the DataManager
 * @param point_type Point type the sensor should process. Possible values: "PointXYZ", "PointXYZRGB"
 */
class PointCloudSensorPlugin : public SensorPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<PointCloudSensorPlugin> Ptr;
  typedef l3::SharedPtr<const PointCloudSensorPlugin> ConstPtr;

  PointCloudSensorPlugin(const std::string& name);

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

protected:
  template <typename PointT>
  void process(typename pcl::PointCloud<PointT>::Ptr cloud)
  {
    ROS_ASSERT(cloud_pcl_handle_->isPointType<PointT>());

    // transform point cloud to map frame if necessary
    std_msgs::Header header;
    pcl_conversions::fromPCL(cloud->header, header);
    std::string cloud_frame_id = l3::strip_const(cloud->header.frame_id, '/');

    std::string error_msg;
    if (tf_buffer_.canTransform(getMapFrame(), header.frame_id, header.stamp, ros::Duration(1.0), &error_msg))
    {
      geometry_msgs::TransformStamped t = tf_buffer_.lookupTransform(getMapFrame(), header.frame_id, header.stamp);

      pcl_ros::transformPointCloud(*cloud, *cloud, t.transform);
      cloud->header.frame_id = getMapFrame();
    }
    else
    {
      ROS_ERROR_THROTTLE(5.0, "[%s] Could not transform input point cloud from '%s' to '%s':\n%s", getName().c_str(), cloud_frame_id.c_str(), getMapFrame().c_str(),
                         error_msg.c_str());
      return;
    }

    Time time = Timer::timeFromRos(header.stamp);

    // set sensor pose in point cloud
    updateSensorPose(time); /// @todo early update required (also called in SensorPlugin::process)
    setSensorPoseInCloud(*cloud, getSensorPose().data);

    // switch pointer to new pointcloud
    l3::UniqueLockPtr lock;
    cloud_pcl_handle_->value<PointT>(lock) = cloud;
    lock.reset();

    // call default pipeline
    UpdatedHandles updates = { cloud_pcl_handle_->handle() };
    SensorPlugin::process(time, updates);
  }

  PclDataHandle<pcl::PointCloud>::Ptr cloud_pcl_handle_;
};
}  // namespace l3_terrain_modeling
