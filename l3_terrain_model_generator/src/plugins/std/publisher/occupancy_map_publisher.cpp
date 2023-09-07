#include <l3_terrain_model_generator/plugins/std/publisher/occupancy_map_publisher.h>

#include <nav_msgs/OccupancyGrid.h>

namespace l3_terrain_modeling
{
OccupancyMapPublisher::OccupancyMapPublisher()
  : PublisherPlugin("occupancy_map_publisher")
{}

bool OccupancyMapPublisher::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("/occupancy_map"), true);
  occupancy_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(topic, 1, latch_topics_);

  return true;
}

bool OccupancyMapPublisher::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::postInitialize(params))
    return false;

  const std::string& input_data_name = param("input_data", std::string("occupancy_map"), true);
  occupancy_map_handle_ = getHandleT<nav_msgs::OccupancyGrid>(input_data_name);
  if (!occupancy_map_handle_)
    return false;

  return true;
}

void OccupancyMapPublisher::publish(const UpdatedHandles& /*input*/) const
{
  if (initial_publish_ || occupancy_map_pub_.getNumSubscribers() > 0)
  {
    if (occupancy_map_handle_)
    {
      l3::SharedLockPtr lock;
      occupancy_map_pub_.publish(occupancy_map_handle_->value<nav_msgs::OccupancyGrid>(lock));
      initial_publish_ = false;
    }
  }
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::OccupancyMapPublisher, l3_terrain_modeling::ProcessorPlugin)
