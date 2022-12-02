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

#include <pcl/PolygonMesh.h>

#include <l3_terrain_model_generator/plugins/base/publisher_plugin.h>

namespace l3_terrain_modeling
{
/**
 * @brief The SurfaceMeshPublisher is a generic publisher for data from type grid_map.
 * @param topic (default: "surface_mesh") Topic name to publish at (the corresponding marker topic is suffixed with "_marker")
 * @param input_data (default: "surface_mesh") Data name as found in the DataManager
 */
class SurfaceMeshPublisher : public PublisherPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<SurfaceMeshPublisher> Ptr;
  typedef l3::SharedPtr<const SurfaceMeshPublisher> ConstPtr;

  SurfaceMeshPublisher();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  bool postInitialize(const vigir_generic_params::ParameterSet& params) override;

protected:
  void publish(const UpdatedHandles& updates) const override;

  DataHandle::Ptr surface_mesh_handle_;

  ros::Publisher surface_mesh_pub_;
  ros::Publisher surface_mesh_marker_pub_;
};
}  // namespace l3_terrain_modeling
