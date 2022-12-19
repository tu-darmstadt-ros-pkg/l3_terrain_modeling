#include <l3_terrain_model/terrain_model.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <l3_libs/types/types.h>
#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3_terrain_modeling
{
TerrainModel::TerrainModel(const grid_map::GridMap& grid_map)
  : grid_map_(grid_map)
  , has_normals_(false)
{
}

TerrainModel::TerrainModel(const grid_map_msgs::GridMap& grid_map)
{
  fromMsg(grid_map);
}

TerrainModel::TerrainModel(const TerrainModelMsg& terrain_model)
{
  fromMsg(terrain_model);
}

TerrainModel::~TerrainModel() {}

void TerrainModel::reset()
{
  grid_map_.clearAll();
}

void TerrainModel::fromMsg(const grid_map_msgs::GridMap& grid_map)
{
  grid_map::GridMapRosConverter::fromMessage(grid_map, grid_map_);
  has_normals_ = grid_map_.exists(NORMAL_LAYER_PREFIX + "x") && grid_map_.exists(NORMAL_LAYER_PREFIX + "y") && grid_map_.exists(NORMAL_LAYER_PREFIX + "z");
}

void TerrainModel::fromMsg(const TerrainModelMsg& terrain_model)
{
  fromMsg(terrain_model.map);
}

void TerrainModel::toMsg(grid_map_msgs::GridMap& grid_map) const
{
  grid_map::GridMapRosConverter::toMessage(grid_map_, grid_map);
}

void TerrainModel::toMsg(TerrainModelMsg& terrain_model) const
{
  toMsg(terrain_model.map);
}

bool TerrainModel::getHeight(double x, double y, double& height) const
{
  grid_map::Index index;
  if (!grid_map_.getIndex(l3::Position2D(x, y), index))
    return false;

  if (!grid_map_.isValid(index, ELEVATION_LAYER))
    return false;

  height = static_cast<double>(grid_map_.at(ELEVATION_LAYER, index));
  return true;
}

template <>
bool TerrainModel::update3DData<geometry_msgs::Pose>(geometry_msgs::Pose& pose) const
{
  l3::Pose p_l3;
  l3::poseMsgToL3(pose, p_l3);

  if (update3DData(p_l3))
  {
    l3::poseL3ToMsg(p_l3, pose);
    return true;
  }

  return false;
}
}  // namespace l3_terrain_modeling
