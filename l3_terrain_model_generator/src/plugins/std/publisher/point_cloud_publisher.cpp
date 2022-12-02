#include <l3_terrain_model_generator/plugins/std/publisher/point_cloud_publisher.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

namespace l3_terrain_modeling
{
PointCloudPublisher::PointCloudPublisher()
  : PublisherPlugin("point_cloud_publisher")
{}

bool PointCloudPublisher::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::loadParams(params))
    return false;

  return true;
}

bool PointCloudPublisher::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("cloud"), true);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic, 1, latch_topics_);

  return true;
}

bool PointCloudPublisher::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::postInitialize(params))
    return false;

  const std::string& input_data_name = param("input_data", std::string("cloud"), true);

  // get pcl handle
  cloud_pcl_handle_ = PclDataHandle<pcl::PointCloud>::makeHandle(input_data_name);
  if (!cloud_pcl_handle_)
  {
    ROS_ERROR("[%s] Data handle \"%s\" seems not to contain valid pcl data!", getName().c_str(), input_data_name.c_str());
    return false;
  }

  return true;
}

void PointCloudPublisher::publish(const UpdatedHandles& updates) const
{
  if (initial_publish_ || cloud_pub_.getNumSubscribers() > 0)
  {
    if (cloud_pcl_handle_)
    {
      sensor_msgs::PointCloud2 msg;
      cloud_pcl_handle_->dispatch<l3::SharedLock>([&](auto& cloud, auto type_trait) { pcl::toROSMsg(*cloud, msg); });
      cloud_pub_.publish(msg);
      initial_publish_ = false;
    }
  }
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::PointCloudPublisher, l3_terrain_modeling::ProcessorPlugin)
