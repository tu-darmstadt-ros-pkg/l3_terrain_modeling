#include <l3_terrain_model_generator/plugins/std/publisher/terrain_model_publisher.h>

#include <grid_map_ros/grid_map_ros.hpp>

#include <l3_terrain_model/terrain_model.h>

namespace l3_terrain_modeling
{
TerrainModelPublisher::TerrainModelPublisher()
  : PublisherPlugin("terrain_model_publisher")
{}

bool TerrainModelPublisher::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("/terrain_model"), true);
  terrain_model_pub_ = nh_.advertise<l3_terrain_modeling::TerrainModelMsg>(topic, 1, true);

  return true;
}

bool TerrainModelPublisher::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::postInitialize(params))
    return false;

  grid_map_handle_ = getHandleT<grid_map::GridMap>(GRID_MAP_NAME);
  if (!grid_map_handle_)
    return false;

  return true;
}

void TerrainModelPublisher::publish(const UpdatedHandles& /*input*/) const
{
  if (initial_publish_ || terrain_model_pub_.getNumSubscribers() > 0)
  {
    if (grid_map_handle_)
    {
      TerrainModelMsg msg;

      l3::SharedLockPtr lock;
      grid_map::GridMapRosConverter::toMessage(grid_map_handle_->value<grid_map::GridMap>(lock), msg.map);
      lock.reset();

      terrain_model_pub_.publish(msg);
      initial_publish_ = false;
    }
  }
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::TerrainModelPublisher, l3_terrain_modeling::ProcessPlugin)
