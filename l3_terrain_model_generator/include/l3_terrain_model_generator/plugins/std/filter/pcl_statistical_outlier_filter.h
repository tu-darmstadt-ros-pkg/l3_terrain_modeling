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

#include <l3_terrain_model_generator/plugins/base/point_cloud_filter_plugin.h>

namespace l3_terrain_modeling
{
/**
 * @brief The PclStatisticalOutlierFilter removes outliers based on statistical data of the neighborhood.
 * The distance threshold will be equal to: mean + stddev_mult * stddev.
 * @param k (default: 10) Number of nearest neighbors to use for mean distance estimation
 * @param stddev_mult (default: 1.0) The standard deviation multiplier for the distance threshold calculation.
 */
class PclStatisticalOutlierFilter : public PointCloudFilterPlugin
{
public:
  // typedefs
  typedef l3::SharedPtr<PclStatisticalOutlierFilter> Ptr;
  typedef l3::SharedPtr<const PclStatisticalOutlierFilter> ConstPtr;

  PclStatisticalOutlierFilter();

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

protected:
  void filter(UpdatedHandles& updates, const SensorPlugin* sensor) const override;

  int k_;
  double stddev_mult_;
};
}  // namespace l3_terrain_modeling
