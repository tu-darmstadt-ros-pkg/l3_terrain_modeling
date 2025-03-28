//=================================================================================================
// Copyright (c) 2025, Alexander Stumpf, TU Darmstadt
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

#include <l3_terrain_model_generator/plugins/base/grid_map_filter_plugin.h>

namespace l3_terrain_modeling
{
/**
 * @brief The PclVoxelGrridFilter downsamples the pointcloud based on a fixed voxel grid.
 * @param lx (default: 1.0) Leaf size for X
 * @param ly (default: 1.0) Leaf size for Y
 * @param lz (default: 1.0) Leaf size for Z
 */
class GridMapCropFilter : public GridMapFilterPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<GridMapCropFilter> Ptr;
  typedef l3::SharedPtr<const GridMapCropFilter> ConstPtr;

  GridMapCropFilter();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

protected:
  void filter(UpdatedHandles& updates, const SensorPlugin* sensor) const override;

  // parameters
  grid_map::Position center_;
  grid_map::Length size_;
};
}  // namespace l3_terrain_modeling
