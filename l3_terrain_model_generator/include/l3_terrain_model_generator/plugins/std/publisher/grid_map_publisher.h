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

#include <ros/ros.h>

#include <l3_terrain_model_generator/typedefs.h>
#include <l3_terrain_model_generator/plugins/base/publisher_plugin.h>

namespace l3_terrain_modeling
{
/**
 * @brief The GridMapPublisher is a generic publisher for data from type grid_map.
 * @param topic (default: "terrain_model_map") Topic name to publish at
 * @param latch (default: true) Defines if topic is latched
 * @param input_data (default: "grid_map") Data name as found in the DataManager
 * @param layers (default: empty) List of names representing the layers to be copied in the published message (keep empty for all)
 */
class GridMapPublisher : public PublisherPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<GridMapPublisher> Ptr;
  typedef l3::SharedPtr<const GridMapPublisher> ConstPtr;

  GridMapPublisher();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  bool postInitialize(const vigir_generic_params::ParameterSet& params) override;

protected:
  void publish(const UpdatedHandles& updates) const override;

  DataHandle::Ptr grid_map_handle_;
  std::vector<std::string> layers_;

  ros::Publisher grid_map_pub_;
};
}  // namespace l3_terrain_modeling
