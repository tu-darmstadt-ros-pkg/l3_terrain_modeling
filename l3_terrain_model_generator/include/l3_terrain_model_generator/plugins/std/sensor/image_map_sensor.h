//=================================================================================================
// Copyright (c) 2023, Alexander stumpf, Felix Sternkopf, TU Darmstadt
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

#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <l3_terrain_model_generator/plugins/base/sensor_plugin.h>

namespace l3_terrain_modeling
{
/**
 * @brief The ImageMapSensor represents a fake sensor that publishes the
 * map data from a greyscale image.
 * @param topic (default: "image") Topic name to subscribe at
 * @param input_data (default: "cloud") Data name as found in the DataManager
 * @param map_frame (default: "map") Frame in which the data is represented
 */
class ImageMapSensor : public SensorPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<ImageMapSensor> Ptr;
  typedef l3::SharedPtr<const ImageMapSensor> ConstPtr;

  ImageMapSensor();

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

private:
  void imageCb(const sensor_msgs::Image& msg);

  DataHandle::Ptr cloud_handle_;

  grid_map::GridMap image_map_;
  std::string layer_;

  double resolution_;
  float min_height_;
  float max_height_;

  ros::Subscriber image_sub_;
};
}  // namespace l3_terrain_modeling
