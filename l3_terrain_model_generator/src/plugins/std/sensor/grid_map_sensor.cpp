#include <l3_terrain_model_generator/plugins/std/sensor/grid_map_sensor.h>

#include <l3_terrain_model/typedefs.h>

namespace l3_terrain_modeling
{
GridMapSensor::GridMapSensor()
  : SensorPlugin("grid_map_sensor")
{}

bool GridMapSensor::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!SensorPlugin::initialize(params))
    return false;

  // init pointcloud
  const std::string& input_data_name = param("input_data", std::string("cloud"), true);
  cloud_handle_ = DataManager::addData(input_data_name, boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>());
  if (!cloud_handle_)
    return false;

  std::string topic = param("topic", ELEVATION_LAYER, true);
  layer_ = param("layer", std::string("elevation"), true);

  grid_map_sub_ = nh_.subscribe(topic, 1, &GridMapSensor::gridMapCb, this);

  return true;
}

void GridMapSensor::gridMapCb(const grid_map_msgs::GridMap& msg)
{
  if (msg.data.empty())
    return;

  // convert msg to grid map
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(msg, input_map, { layer_ });

  // convert grid map to point cloud
  sensor_msgs::PointCloud2 point_cloud_msg;
  grid_map::GridMapRosConverter::toPointCloud(input_map, layer_, point_cloud_msg);
  point_cloud_msg.header.frame_id = getMapFrame();
  point_cloud_msg.header.stamp = ros::Time::now();

  // update pointcloud
  l3::UniqueLockPtr lock;
  pcl::fromROSMsg(point_cloud_msg, *cloud_handle_->value<pcl::PointCloud<pcl::PointXYZ>::Ptr>(lock));
  lock.reset();

  // call default processing pipeline
  UpdatedHandles updates = { cloud_handle_ };
  SensorPlugin::process(Timer::timeFromRos(point_cloud_msg.header.stamp), updates);
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GridMapSensor, l3_terrain_modeling::SensorPlugin)
