#include <l3_terrain_model_generator/plugins/std/sensor/generic_lidar_sensor.h>

namespace l3_terrain_modeling
{
GenericLidarSensor::GenericLidarSensor()
  : PointCloudSensorPlugin("generic_lidar_sensor")
{}

bool GenericLidarSensor::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PointCloudSensorPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("cloud"), true);
  pointcloud_sub_ = nh_.subscribe(topic, 1, &GenericLidarSensor::pointCloudCb, this);

  return true;
}

void GenericLidarSensor::pointCloudCb(const sensor_msgs::PointCloud2& msg)
{
  // consider processing rate
  if (!canProcess(Timer::timeFromRos(msg.header.stamp)))
    return;

  if (msg.data.empty())
    return;

  if (cloud_pcl_handle_)
  {
    cloud_pcl_handle_->dispatch([&](auto type_trait)
    {
      typename pcl::PointCloud<decltype(type_trait)>::Ptr cloud = boost::make_shared<pcl::PointCloud<decltype(type_trait)>>();
      pcl::fromROSMsg(msg, *cloud);
      process<decltype(type_trait)>(cloud);
    });
  }
  else
    ROS_ERROR("[%s] Cloud handle is not set!", getName().c_str());
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GenericLidarSensor, l3_terrain_modeling::SensorPlugin)
