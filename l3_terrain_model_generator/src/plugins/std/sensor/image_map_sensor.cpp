#include <l3_terrain_model_generator/plugins/std/sensor/image_map_sensor.h>

#include <l3_terrain_model/typedefs.h>

namespace l3_terrain_modeling
{
ImageMapSensor::ImageMapSensor() : SensorPlugin("image_map_sensor")
{
}

bool ImageMapSensor::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!SensorPlugin::loadParams(params))
    return false;

  resolution_ = param("resolution", 0.025);
  min_height_ = param("min_height", 0.0);
  max_height_ = param("max_height", 1.0);

  layer_ = param("layer", ELEVATION_LAYER, true);

  return true;
}

bool ImageMapSensor::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!SensorPlugin::initialize(params))
    return false;

  GET_OUTPUT_PCL_HANDLE_DEFAULT("cloud", cloud_pcl_handle_);

  std::string topic = param("topic", std::string("image"), true);
  image_sub_ = nh_.subscribe(topic, 1, &ImageMapSensor::imageCb, this);

  return true;
}

void ImageMapSensor::imageCb(const sensor_msgs::Image& msg)
{
  if (msg.data.empty())
    return;

  // generate grid map from image
  grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, image_map_);
  ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", image_map_.getLength().x(),
           image_map_.getLength().y(), image_map_.getSize().x(), image_map_.getSize().y());
  grid_map::GridMapRosConverter::addLayerFromImage(msg, layer_, image_map_, min_height_, max_height_);

  // convert grid map to point cloud
  sensor_msgs::PointCloud2 point_cloud_msg;
  grid_map::GridMapRosConverter::toPointCloud(image_map_, layer_, point_cloud_msg);
  point_cloud_msg.header.frame_id = getMapFrame();
  point_cloud_msg.header.stamp = ros::Time::now();

  // update pointcloud
  cloud_pcl_handle_->dispatch<l3::UniqueLock>(
      [&](auto& cloud, auto type_trait) { pcl::fromROSMsg(point_cloud_msg, *cloud); });

  // call default processing pipeline
  SensorPlugin::process(Timer::timeFromRos(point_cloud_msg.header.stamp),
                        l3::makeShared<UpdatedHandles>(std::initializer_list<DataHandle::ConstPtr>{ cloud_pcl_handle_->handle() }));
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::ImageMapSensor, l3_terrain_modeling::SensorPlugin)
