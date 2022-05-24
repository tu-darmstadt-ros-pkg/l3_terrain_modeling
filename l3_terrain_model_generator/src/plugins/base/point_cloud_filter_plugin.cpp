#include <l3_terrain_model_generator/plugins/base/point_cloud_filter_plugin.h>

namespace l3_terrain_modeling
{
PointCloudFilterPlugin::PointCloudFilterPlugin(const std::string& name)
  : FilterPlugin(name)
{}

bool PointCloudFilterPlugin::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!FilterPlugin::postInitialize(params))
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
}  // namespace l3_terrain_modeling
