#include <l3_terrain_model_generator/plugins/std/generator/occupancy_map_generator.h>

#include <nav_msgs/OccupancyGrid.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <l3_terrain_model/typedefs.h>

namespace l3_terrain_modeling
{
OccupancyMapGenerator::OccupancyMapGenerator()
  : GeneratorPlugin("occupancy_map_generator")
  , tf_listener_(tf_buffer_)
{}

bool OccupancyMapGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  layer_ = param("layer", ELEVATION_LAYER, true);

  z_ref_frame_id_ = param("z_ref_frame", std::string(), true);

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

  GET_OUTPUT_HANDLE_DEFAULT(nav_msgs::OccupancyGrid(), "occupancy_map", occupancy_map_handle_);

  return true;
}

bool OccupancyMapGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  GET_INPUT_HANDLE_DEFAULT(grid_map::GridMap, GRID_MAP_NAME, grid_map_handle_);

  return true;
}

void OccupancyMapGenerator::update(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor)
{
  l3::UniqueLockPtr occupancy_map_lock;
  nav_msgs::OccupancyGrid& occupancy_map = occupancy_map_handle_->value<nav_msgs::OccupancyGrid>(occupancy_map_lock);

  l3::SharedLockPtr grid_map_lock;
  const grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(grid_map_lock);

  // determine z reference height
  float ref_height = 0.0;
  if (!z_ref_frame_id_.empty())
  {
    l3::Pose z_ref_pose;
    if (getTransformAsPose(tf_buffer_, grid_map.getFrameId(), z_ref_frame_id_, ros::Time().fromNSec(grid_map.getTimestamp()), z_ref_pose))
      ref_height = z_ref_pose.z();
    else
      ROS_WARN("[%s] Failed to adjust occupancy map z position to reference frame \"%s\"!", getName().c_str(),
               z_ref_frame_id_.c_str());
  }
  else if (sensor)
    ref_height = sensor->getSensorPose().data.z();

  // convert grid map to occupancy grid
  grid_map::GridMapRosConverter::toOccupancyGrid(grid_map, layer_, ref_height + min_height_,
                                                 ref_height + max_height_, occupancy_map);

  // generate binary occupancy grid
  if (binarize_)
  {
    for (size_t i = 0; i < occupancy_map.data.size(); ++i)
    {
      if (occupancy_map.data[i] > binary_threshold_)
        occupancy_map.data[i] = 100;
      else
        occupancy_map.data[i] = 0;
    }
  }

  // adjust z position
  occupancy_map.info.origin.position.z = ref_height;
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::OccupancyMapGenerator, l3_terrain_modeling::ProcessorPlugin)
