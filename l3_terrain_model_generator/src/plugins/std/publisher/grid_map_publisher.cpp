#include <l3_terrain_model_generator/plugins/std/publisher/grid_map_publisher.h>

#include <grid_map_ros/grid_map_ros.hpp>

namespace l3_terrain_modeling
{
GridMapPublisher::GridMapPublisher()
  : PublisherPlugin("grid_map_publisher")
{}

bool GridMapPublisher::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::loadParams(params))
    return false;

  layers_ = param("layers", std::vector<std::string>(), true);

  return true;
}

bool GridMapPublisher::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("/grid_map"), true);
  grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>(topic, 1, latch_topics_);

  return true;
}

bool GridMapPublisher::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::postInitialize(params))
    return false;

  GET_INPUT_HANDLE_DEFAULT(grid_map::GridMap, GRID_MAP_NAME, grid_map_handle_);

  return true;
}

void GridMapPublisher::publish(const UpdatedHandles& /*input*/) const
{
  if (initial_publish_ || grid_map_pub_.getNumSubscribers() > 0)
  {
    if (grid_map_handle_)
    {
      grid_map_msgs::GridMap msg;

      if (layers_.empty())
      {
        l3::SharedLockPtr lock;
        grid_map::GridMapRosConverter::toMessage(grid_map_handle_->value<grid_map::GridMap>(lock), msg);
      }
      else
      {
        l3::SharedLockPtr lock;
        grid_map::GridMapRosConverter::toMessage(grid_map_handle_->value<grid_map::GridMap>(lock), layers_, msg);
      }

      grid_map_pub_.publish(msg);
      initial_publish_ = false;
    }
  }
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GridMapPublisher, l3_terrain_modeling::ProcessorPlugin)
