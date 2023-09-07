#include <l3_terrain_model_generator/plugins/std/generator/occupancy_map_generator.h>

#include <nav_msgs/OccupancyGrid.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <l3_terrain_model/typedefs.h>

namespace l3_terrain_modeling
{
OccupancyMapGenerator::OccupancyMapGenerator()
  : GeneratorPlugin("occupancy_map_generator")
{}

bool OccupancyMapGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  layer_ = param("layer", ELEVATION_LAYER, true);

  min_height_ = param("min_height", -0.1, true);
  max_height_ = param("max_height", 0.9, true);

  binarize_ = param("binarize", true, true);
  binary_threshold_ = param("binary_threshold", 50, true);

  return true;
}

bool OccupancyMapGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::initialize(params))
    return false;

  const std::string& output_data_name = param("output_data", std::string("occupancy_map"), true);
  occupancy_map_handle_ = DataManager::addData(output_data_name, nav_msgs::OccupancyGrid());
  if (!occupancy_map_handle_)
    return false;

  return true;
}

bool OccupancyMapGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  const std::string& input_data_name = param("input_data", std::string(GRID_MAP_NAME), true);
  grid_map_handle_ = getHandleT<grid_map::GridMap>(input_data_name);
  if (!grid_map_handle_)
    return false;

  return true;
}

void OccupancyMapGenerator::update(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor)
{
  l3::UniqueLockPtr occupancy_map_lock;
  nav_msgs::OccupancyGrid& occupancy_map = occupancy_map_handle_->value<nav_msgs::OccupancyGrid>(occupancy_map_lock);

  l3::SharedLockPtr grid_map_lock;
  const grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(grid_map_lock);

  // get sensor height to slice grid map
  float sensor_height = 0.0;
  if (sensor)
    sensor_height = sensor->getSensorPose().data.z();

  // convert grid map to occupancy grid
  grid_map::GridMapRosConverter::toOccupancyGrid(grid_map, layer_, sensor_height + min_height_, sensor_height + max_height_, occupancy_map);

  // generate binary occupancy grid
  if (binarize_)
  {
    for (int i = 0; i < occupancy_map.data.size(); ++i)
    {
      if (occupancy_map.data[i] > binary_threshold_)
        occupancy_map.data[i] = 100;
      else
        occupancy_map.data[i] = 0;
    }
  }
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::OccupancyMapGenerator, l3_terrain_modeling::ProcessorPlugin)
