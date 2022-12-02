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

#include <l3_terrain_model/terrain_model.h>

#include <l3_terrain_model_generator/utils/pcl/pcl_data_handle.h>
#include <l3_terrain_model_generator/utils/pcl/octree_voxel_grid.h>
#include <l3_terrain_model_generator/plugins/base/generator_plugin.h>

namespace l3_terrain_modeling
{
class NormalsCloudGenerator : public GeneratorPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<NormalsCloudGenerator> Ptr;
  typedef l3::SharedPtr<const NormalsCloudGenerator> ConstPtr;

  NormalsCloudGenerator();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  bool postInitialize(const vigir_generic_params::ParameterSet& params) override;

  void reset() override;

protected:
  void update(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor) override;

  void computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr search_points);

  PclDataHandle<OctreeVoxelGrid>::Ptr input_octree_pcl_handle_;
  PclDataHandle<pcl::PointCloud>::Ptr input_cloud_pcl_handle_;
  DataHandle::Ptr normals_octree_handle_;
  DataHandle::Ptr normals_cloud_handle_;
  DataHandle::Ptr grid_map_handle_;

  // parameters
  int threads_;
  unsigned int min_aggregation_size_;
  unsigned int filter_mask_;

  // params for normal estimator
  double ne_radius_;

  // params for voxel grid filter
  l3::Vector3 voxel_grid_size_;

  // params for moving least squares smoother
  double ms_radius_;

  // params for statistical outlier filter
  double so_radius_;
  int so_k_;
};
}  // namespace l3_terrain_modeling
