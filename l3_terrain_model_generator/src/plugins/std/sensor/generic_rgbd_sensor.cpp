#include <l3_terrain_model_generator/plugins/std/sensor/generic_rgbd_sensor.h>

namespace l3_terrain_modeling
{
GenericRGBDSensor::GenericRGBDSensor()
  : PointCloudSensorPlugin("generic_rgbd_sensor")
{}

bool GenericRGBDSensor::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PointCloudSensorPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("cloud"), true);
  pointcloud_sub_ = nh_.subscribe(topic, 1, &GenericRGBDSensor::pointCloudCb, this);

  return true;
}

void GenericRGBDSensor::pointCloudCb(const sensor_msgs::PointCloud2& msg)
{
  // consider processing rate
  if (!canProcess(Timer::timeFromRos(msg.header.stamp)))
    return;

  if (msg.data.empty())
    return;

  if (msg.height == 1)
    ROS_WARN_ONCE("[%s] Received unorganized data. This warning is printed only once.", getName().c_str());

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
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GenericRGBDSensor, l3_terrain_modeling::SensorPlugin)
