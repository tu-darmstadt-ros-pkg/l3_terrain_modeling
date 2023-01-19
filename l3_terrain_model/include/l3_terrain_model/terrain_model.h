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

#include <grid_map_core/GridMap.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <l3_math/angles.h>

#include <l3_terrain_model_msgs/TerrainModelMsg.h>

namespace l3_terrain_modeling
{
using namespace l3_terrain_model_msgs;

class TerrainModel
{
public:
  // typedefs
  typedef l3::SharedPtr<TerrainModel> Ptr;
  typedef l3::SharedPtr<const TerrainModel> ConstPtr;

  inline static const std::string ELEVATION_LAYER = "elevation";
  inline static const std::string NORMAL_LAYER_PREFIX = "normal_";
  inline static const std::string RGB_LAYER = "rgb";

  TerrainModel(const grid_map::GridMap& grid_map);
  TerrainModel(const grid_map_msgs::GridMap& grid_map);
  TerrainModel(const TerrainModelMsg& terrain_model);
  virtual ~TerrainModel();

  void reset();

  void fromMsg(const grid_map_msgs::GridMap& grid_map);
  void fromMsg(const TerrainModelMsg& terrain_model);

  void toMsg(grid_map_msgs::GridMap& grid_map) const;
  void toMsg(TerrainModelMsg& terrain_model) const;

  inline const std::string& getFrameId() const { return grid_map_.getFrameId(); }

  inline double getResolution() const { return grid_map_.getResolution(); }

  inline bool hasTerrainModel() const { return grid_map_.exists(ELEVATION_LAYER); }

  bool getHeight(double x, double y, double& height) const;

  template <typename T>
  bool getNormal(const T& pose, l3::Vector3& normal) const
  {
    grid_map::Index index;
    if (!has_normals_ || !grid_map_.getIndex(l3::Position2D(pose.x(), pose.y()), index))
      return false;

    return grid_map_.getVector(NORMAL_LAYER_PREFIX, index, normal);
  }

  template <typename T>
  bool update3DData(T& pose) const
  {
    grid_map::Index index;
    if (!grid_map_.getIndex(l3::Position2D(pose.x(), pose.y()), index))
      return false;

    if (!grid_map_.isValid(index, ELEVATION_LAYER))
      return false;

    // get z
    pose.setZ(grid_map_.at(ELEVATION_LAYER, index));

    // get roll and pitch
    l3::Vector3 normal;
    if (has_normals_ && !grid_map_.getVector(NORMAL_LAYER_PREFIX, index, normal))
      return false;

    double roll;
    double pitch;
    double yaw = pose.yaw();
    l3::normalToRP(normal, yaw, roll, pitch);
    pose.setRPY(roll, pitch, yaw);

    return true;
  }

protected:
  grid_map::GridMap grid_map_;

  bool has_normals_;
};

template <>
bool TerrainModel::update3DData<geometry_msgs::Pose>(geometry_msgs::Pose& pose) const;
}  // namespace l3_terrain_modeling
