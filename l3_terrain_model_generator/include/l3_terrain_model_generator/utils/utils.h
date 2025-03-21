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

#include <tf2_ros/transform_listener.h>

#include <grid_map_core/GridMap.hpp>

#include <vigir_generic_params/parameter_set.h>

#include <l3_libs/types/types.h>

namespace l3_terrain_modeling
{
bool getTransformAsPose(const tf2_ros::Buffer& tf_buffer, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, l3::Pose& pose);
bool getTransformAsPose(const tf2_ros::Buffer& tf_buffer, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, l3::StampedPose& pose);

bool getDeltaTransformAsPose(const tf2_ros::Buffer& tf_buffer, const std::string& target_frame, const std::string& fixed_frame, const ros::Time& time_past, const ros::Time& time_future, l3::StampedPose& pose);

template <template<class, class> class Container, class Type>
void getBoundary(const Container<Type, std::allocator<Type>>& container, l3::Vector3& min, l3::Vector3& max)
{
  min.x() = min.y() = min.z() = std::numeric_limits<double>::max();
  max.x() = max.y() = max.z() = -std::numeric_limits<double>::max();

  for (const Type& vec : container)
  {
    min.x() = std::min(min.x(), vec.x());
    min.y() = std::min(min.y(), vec.y());
    min.z() = std::min(min.z(), vec.z());
    max.x() = std::max(max.x(), vec.x());
    max.y() = std::max(max.y(), vec.y());
    max.z() = std::max(max.z(), vec.z());
  }
}

template <template<class, class> class Container, class Type>
void getBoundaryPtr(const Container<Type, std::allocator<Type>>& container, l3::Vector3& min, l3::Vector3& max)
{
  min.x() = min.y() = min.z() = std::numeric_limits<double>::max();
  max.x() = max.y() = max.z() = -std::numeric_limits<double>::max();

  for (Type vec : container)
  {
    min.x() = std::min(min.x(), vec->x());
    min.y() = std::min(min.y(), vec->y());
    min.z() = std::min(min.z(), vec->z());
    max.x() = std::max(max.x(), vec->x());
    max.y() = std::max(max.y(), vec->y());
    max.z() = std::max(max.z(), vec->z());
  }
}

void resize(grid_map::GridMap& grid_map, const l3::Vector3& min, const l3::Vector3& max);

template <typename T>
T getInputDataParam(const vigir_generic_params::ParameterSet& p, const std::string& key, const T& default_val)
{
  vigir_generic_params::ParameterSet param;
  param = p.getSubset("data");
  param = param.getSubset("in");
  return param.param(key, default_val, true);
}

template <typename T>
T getOutputDataParam(const vigir_generic_params::ParameterSet& p, const std::string& key, const T& default_val)
{
  vigir_generic_params::ParameterSet param;
  param = p.getSubset("data");
  param = param.getSubset("out");
  return param.param(key, default_val, true);
}
}  // namespace l3_terrain_modeling
