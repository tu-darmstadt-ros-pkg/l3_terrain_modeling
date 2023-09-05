//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
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

#include <l3_terrain_model_generator/utils/pcl/pcl_data_handle.h>
#include <l3_terrain_model_generator/plugins/base/generator_plugin.h>

namespace l3_terrain_modeling
{
class GridMapGeneratorPlugin : public GeneratorPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<GridMapGeneratorPlugin> Ptr;
  typedef l3::SharedPtr<const GridMapGeneratorPlugin> ConstPtr;

  GridMapGeneratorPlugin(const std::string& name);

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  virtual void reset() override;

protected:
  /**
   * @brief Returns the header of the data that is used to update the grid map.
   * @return Header of the data.
   */
  virtual std_msgs::Header getDataHeader() = 0;

  /**
   * @brief Returns the boundary of the data that is used to update the grid map.
   * @param min [out] Minimum boundary of the data.
   * @param max [out] Maximum boundary of the data.
   */
  virtual void getDataBoundary(l3::Vector3& min, l3::Vector3& max) = 0;

  void processImpl(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor) override;

  DataHandle::Ptr input_handle_; // should be set by derived class to detect if input is available
  DataHandle::Ptr grid_map_handle_;

  bool use_color_;
};


/**
 * @brief The GridCellGridMapGeneratorPlugin represents a grid map generator
 * that uses a grid cell data structure as input.
 */
class GridCellGridMapGeneratorPlugin : public GridMapGeneratorPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<GridCellGridMapGeneratorPlugin> Ptr;
  typedef l3::SharedPtr<const GridCellGridMapGeneratorPlugin> ConstPtr;

  GridCellGridMapGeneratorPlugin(const std::string& name);

  bool postInitialize(const vigir_generic_params::ParameterSet& params) override;

protected:
  std_msgs::Header getDataHeader() override;
  void getDataBoundary(l3::Vector3& min, l3::Vector3& max) override;
};


/**
 * @brief The PclGridMapGeneratorPlugin represents a grid map generator
 * that uses a pcl point cloud as input.
 */
class PclGridMapGeneratorPlugin : public GridMapGeneratorPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<PclGridMapGeneratorPlugin> Ptr;
  typedef l3::SharedPtr<const PclGridMapGeneratorPlugin> ConstPtr;

  PclGridMapGeneratorPlugin(const std::string& name);

  bool postInitialize(const vigir_generic_params::ParameterSet& params) override;

protected:
  std_msgs::Header getDataHeader() override;
  void getDataBoundary(l3::Vector3& min, l3::Vector3& max) override;

  PclDataHandle<pcl::PointCloud>::Ptr cloud_pcl_handle_;
};
}  // namespace l3_terrain_modeling
