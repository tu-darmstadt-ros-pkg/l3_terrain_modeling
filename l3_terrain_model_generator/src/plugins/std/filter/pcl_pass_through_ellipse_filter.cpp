#include <l3_terrain_model_generator/plugins/std/filter/pcl_pass_through_ellipse_filter.h>

#include <l3_terrain_model_generator/utils/pcl/point_cloud_filter.h>

namespace l3_terrain_modeling
{
PclPassThroughEllipseFilter::PclPassThroughEllipseFilter()
  : PointCloudFilterPlugin("pcl_pass_through_ellipse_filter")
{}

bool PclPassThroughEllipseFilter::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PointCloudFilterPlugin::loadParams(params))
    return false;

  radius_x_ = param("radius_x", std::numeric_limits<double>::max(), true);
  radius_y_ = param("radius_y", std::numeric_limits<double>::max(), true);

  return true;
}

void PclPassThroughEllipseFilter::filter(UpdatedHandles& /*input*/, const SensorPlugin* /*sensor*/) const
{
  if (cloud_pcl_handle_)
  {
    cloud_pcl_handle_->dispatch<l3::UniqueLock>([&](auto& cloud, auto type_trait) {
      cloud = filterPassThroughEllipse<decltype(type_trait)>(cloud, getSensorPoseFromCloud(*cloud), radius_x_, radius_y_);
    });
  }
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::PclPassThroughEllipseFilter, l3_terrain_modeling::ProcessorPlugin)
