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
  pointcloud_sub_ = nh_.subscribe(topic, 1, &GenericRGBDSensor::pointcloudCb, this);

  return true;
}

void GenericRGBDSensor::pointcloudCb(const sensor_msgs::PointCloud2& msg)
{
  // consider processing rate
  if (!canProcess(Timer::timeFromRos(msg.header.stamp)))
    return;

  if (msg.data.empty())
    return;

  if (msg.height == 1)
    ROS_WARN_ONCE("[%s] Received unorganized data. This warning is printed only once.", getName().c_str());

  // extract xyzrbg pointcloud if possible
  for (const sensor_msgs::PointField& field : msg.fields)
  {
    if (field.name == "rgb" || field.name == "rgba")
    {
      if (cloud_pcl_handle_->isPointType<pcl::PointXYZRGB>())
      {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::fromROSMsg(msg, *cloud);
        process<pcl::PointXYZRGB>(cloud);
        return;
      }
      else
        ROS_WARN_ONCE("[%s] Received colored pointcloud but sensor plugin is not configured to handle rgb data. This warning is printed only once.", getName().c_str());
    }
  }

  // otherwise extract xyz pointcloud
  if (cloud_pcl_handle_->isPointType<pcl::PointXYZ>())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(msg, *cloud);
    process<pcl::PointXYZ>(cloud);
  }
  else
    ROS_ERROR("[%s] Received non-colored pointcloud but sensor plugin is configured to handle colored pointclouds!", getName().c_str());
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GenericRGBDSensor, l3_terrain_modeling::SensorPlugin)
