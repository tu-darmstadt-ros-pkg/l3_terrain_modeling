#include <l3_terrain_model_generator/plugins/std/filter/pcl_pass_through_box_filter.h>

#include <l3_terrain_model_generator/utils/pcl/point_cloud_filter.h>

namespace l3_terrain_modeling
{
PclPassThroughBoxFilter::PclPassThroughBoxFilter()
  : PointCloudFilterPlugin("pcl_pass_through_box_filter")
{}

bool PclPassThroughBoxFilter::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PointCloudFilterPlugin::loadParams(params))
    return false;

  field_name_ = param("field_name", std::string());
  limit_min_ = param("limit_min", -std::numeric_limits<double>::max(), true);
  limit_max_ = param("limit_max", std::numeric_limits<double>::max(), true);

  return true;
}

void PclPassThroughBoxFilter::filter(UpdatedHandles& updates, const SensorPlugin* /*sensor*/) const
{
  // run only on changes
  if (!updates.has(cloud_pcl_handle_->handle()))
    return;

  if (cloud_pcl_handle_)
    cloud_pcl_handle_->dispatch<l3::UniqueLock>([&](auto& cloud, auto type_trait) { cloud = filterPassThroughBox<decltype(type_trait)>(cloud, field_name_, limit_min_, limit_max_); });
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::PclPassThroughBoxFilter, l3_terrain_modeling::ProcessorPlugin)
