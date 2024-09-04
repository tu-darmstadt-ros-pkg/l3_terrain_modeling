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

  GET_OUTPUT_PCL_HANDLE("data", "cloud", "PointXYZ", cloud_pcl_handle_);

  return true;
}
}  // namespace l3_terrain_modeling
