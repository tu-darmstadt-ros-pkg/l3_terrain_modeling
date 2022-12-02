#include <l3_terrain_model_generator/plugins/std/filter/pcl_mls_smooth_filter.h>

#include <l3_terrain_model_generator/utils/pcl/point_cloud_filter.h>

namespace l3_terrain_modeling
{
PclMlsSmoothFilter::PclMlsSmoothFilter()
  : PointCloudFilterPlugin("pcl_mls_smooth_filter")
{}

bool PclMlsSmoothFilter::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PointCloudFilterPlugin::loadParams(params))
    return false;

  radius_ = param("radius", 1.0, true);

  return true;
}

void PclMlsSmoothFilter::filter(UpdatedHandles& updates, const SensorPlugin* /*sensor*/) const
{
  // run only on changes
  if (!updates.has(cloud_pcl_handle_->handle()))
    return;

  if (cloud_pcl_handle_)
    cloud_pcl_handle_->dispatch<l3::UniqueLock>([&](auto& cloud, auto type_trait) { cloud = filterMlsSmooth<decltype(type_trait)>(cloud, radius_); });
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::PclMlsSmoothFilter, l3_terrain_modeling::ProcessorPlugin)
