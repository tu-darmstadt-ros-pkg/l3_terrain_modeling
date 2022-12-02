#include <l3_terrain_model_generator/plugins/std/filter/pcl_statistical_outlier_filter.h>

#include <l3_terrain_model_generator/utils/pcl/point_cloud_filter.h>

namespace l3_terrain_modeling
{
PclStatisticalOutlierFilter::PclStatisticalOutlierFilter()
  : PointCloudFilterPlugin("pcl_statistical_outlier_filter")
{}

bool PclStatisticalOutlierFilter::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PointCloudFilterPlugin::loadParams(params))
    return false;

  k_ = param("k", 10, true);
  stddev_mult_ = param("stddev_mult", 1.0, true);

  return true;
}

void PclStatisticalOutlierFilter::filter(UpdatedHandles& updates, const SensorPlugin* /*sensor*/) const
{
  // run only on changes
  if (!updates.has(cloud_pcl_handle_->handle()))
    return;

  if (cloud_pcl_handle_)
    cloud_pcl_handle_->dispatch<l3::UniqueLock>([&](auto& cloud, auto type_trait) { cloud = filterStatisticalOutlier<decltype(type_trait)>(cloud, k_, stddev_mult_); });
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::PclStatisticalOutlierFilter, l3_terrain_modeling::ProcessorPlugin)
