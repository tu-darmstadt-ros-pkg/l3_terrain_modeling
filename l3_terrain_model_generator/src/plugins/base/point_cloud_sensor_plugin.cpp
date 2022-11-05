#include <l3_terrain_model_generator/plugins/base/point_cloud_sensor_plugin.h>

namespace l3_terrain_modeling
{
PointCloudSensorPlugin::PointCloudSensorPlugin(const std::string& name)
  : SensorPlugin(name)
{}

bool PointCloudSensorPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!SensorPlugin::initialize(params))
    return false;

  // init pointcloud
  const std::string& input_data_name = param("input_data", std::string("cloud"), true);
  std::string point_type = param("point_type", std::string("PointXYZ"), true);
  cloud_pcl_handle_ = PclDataHandle<pcl::PointCloud>::makeHandle(input_data_name, point_type);

  if (!cloud_pcl_handle_)
  {
    ROS_ERROR("[%s] Unsupported point type: \"%s\"", getName().c_str(), point_type.c_str());
    return false;
  }

  return true;
}
}  // namespace l3_terrain_modeling
