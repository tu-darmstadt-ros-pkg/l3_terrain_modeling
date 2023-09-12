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

  GET_INPUT_PCL_HANDLE_DEFAULT("cloud", cloud_pcl_handle_);

  return true;
}
}  // namespace l3_terrain_modeling
