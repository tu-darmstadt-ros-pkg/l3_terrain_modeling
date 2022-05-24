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

#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <l3_terrain_model_generator/plugins/base/process_plugin.h>

namespace l3_terrain_modeling
{
class FilterPlugin : public ProcessPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<FilterPlugin> Ptr;
  typedef l3::SharedPtr<const FilterPlugin> ConstPtr;

  FilterPlugin(const std::string& name);

  virtual ~FilterPlugin() = default;

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

protected:
  void processImpl(const Timer& timer, UpdatedHandles& input, const SensorPlugin* sensor) override { filter(input, sensor); }

  virtual void filter(UpdatedHandles& input, const SensorPlugin* sensor) const = 0;

  bool determineCurrentRobotPose(l3::Pose& pose) const;

private:
  void robotPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose);

  geometry_msgs::PoseStamped::Ptr robot_pose_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber robot_pose_sub_;
};
}  // namespace l3_terrain_modeling
