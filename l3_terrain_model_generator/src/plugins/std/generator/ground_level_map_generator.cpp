#include <l3_terrain_model_generator/plugins/std/generator/ground_level_map_generator.h>

#include <limits.h>

namespace l3_terrain_modeling
{
GroundLevelMapGenerator::GroundLevelMapGenerator()
  : GridMapGeneratorPlugin("ground_level_map_generator")
{
}

bool GroundLevelMapGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapGeneratorPlugin::initialize(params))
    return false;

  l3::UniqueLockPtr lock;
  grid_map_handle_->value<grid_map::GridMap>(lock).add("ground_level");

  return true;
}

bool GroundLevelMapGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapGeneratorPlugin::postInitialize(params))
    return false;

  cloud_gradients_handle_ = getHandleT<pcl::PointCloud<pcl::PointXYZI>::Ptr>("cloud_gradients");
  cloud_edges_handle_ = getHandleT<pcl::PointCloud<pcl::PointXYZI>::Ptr>("cloud_edges");

  if (!cloud_gradients_handle_ && !cloud_edges_handle_)
  {
    ROS_ERROR("[%s] Neither \"cloud_gradients\" nor \"cloud_edges\" is available!", getName().c_str());
    return false;
  }

  return true;
}

void GroundLevelMapGenerator::reset()
{
  GeneratorPlugin::reset();

  l3::UniqueLockPtr grid_map_lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(grid_map_lock);
//  grid_map.clear(TerrainModel::NORMAL_LAYER_PREFIX + "x");
//  grid_map.clear(TerrainModel::NORMAL_LAYER_PREFIX + "y");
//  grid_map.clear(TerrainModel::NORMAL_LAYER_PREFIX + "z");
}

void GroundLevelMapGenerator::update(const Timer& /*timer*/, UpdatedHandles& /*input*/, const SensorPlugin* /*sensor*/)
{
  if (!cloud_gradients_handle_ || !cloud_edges_handle_)
    return;

  generateGroundLevelMap();
}

void GroundLevelMapGenerator::generateGroundLevelMap()
{
  l3::SharedLockPtr gradients_lock;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_gradients = cloud_gradients_handle_->value<pcl::PointCloud<pcl::PointXYZI>::Ptr>(gradients_lock);

  l3::SharedLockPtr edges_lock;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_edges = cloud_edges_handle_->value<pcl::PointCloud<pcl::PointXYZI>::Ptr>(edges_lock);

  if (cloud_gradients->empty() && cloud_edges->empty())
  {
    ROS_ERROR("Can't generate grid map! Neither gradients nor edges have been computed yet.");
    return;
  }

  // determine min and max coordinatesss
  double min_x, max_x;
  double min_y, max_y;

  min_x = min_y = std::numeric_limits<double>::max();
  max_x = max_y = std::numeric_limits<double>::min();

  // add data from gradients point cloud
  if (!cloud_gradients->empty())
  {
    ROS_INFO("...adding gradients");
    for (size_t i = 0; i < cloud_gradients->size(); i++)
    {
      const pcl::PointXYZI& p = cloud_gradients->at(i);

      //      if (ignore_near)
      //      {
      //        if ((p.x-robot_x)*(p.x-robot_x) + (p.y-robot_y)*(p.y-robot_y) < 0.4*0.4)
      //          continue;
      //      }

      int idx = 0;
      //      if (GridMap::getGridMapIndex(*grid_map_, p.x, p.y, idx))
      //        grid_map_->data.at(idx) = std::max(grid_map_->data.at(idx), (int8_t)floor((1.0 - p.intensity) * 100));
    }
  }

  // add data from edge point cloud
  if (!cloud_edges->empty())
  {
    ROS_INFO("...adding edges");
    ROS_ASSERT(cloud_gradients->size() == cloud_edges->size());

    for (size_t i = 0; i < cloud_edges->size(); i++)
    {
      const pcl::PointXYZI& p = cloud_edges->at(i);

      //      if (ignore_near)
      //      {
      //        if ((p.x-robot_x)*(p.x-robot_x) + (p.y-robot_y)*(p.y-robot_y) < 0.4*0.4)
      //          continue;
      //      }

      int idx = 0;
      //      if (GridMap::getGridMapIndex(*grid_map_, p.x, p.y, idx))
      //        grid_map_->data.at(idx) = std::max(grid_map_->data.at(idx), (int8_t)floor((1.0 - p.intensity) * 100));
    }
  }

  // update grid map timestamp
  l3::UniqueLockPtr lock;
  grid_map_handle_->value<grid_map::GridMap>(lock).setTimestamp(cloud_gradients->header.stamp);
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GroundLevelMapGenerator, l3_terrain_modeling::ProcessPlugin)
