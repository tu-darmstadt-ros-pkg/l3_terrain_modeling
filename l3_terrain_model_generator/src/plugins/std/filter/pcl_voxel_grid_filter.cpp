#include <l3_terrain_model_generator/plugins/std/filter/pcl_voxel_grid_filter.h>

#include <l3_terrain_model_generator/utils/pcl/point_cloud_filter.h>

namespace l3_terrain_modeling
{
PclVoxelGrridFilter::PclVoxelGrridFilter()
  : PointCloudFilterPlugin("pcl_voxel_grid_filter")
{}

bool PclVoxelGrridFilter::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PointCloudFilterPlugin::loadParams(params))
    return false;

  lx_ = param("lx", 1.0, true);
  ly_ = param("ly", 1.0, true);
  lz_ = param("lz", 1.0, true);

  return true;
}

void PclVoxelGrridFilter::filter(UpdatedHandles& /*input*/, const SensorPlugin* /*sensor*/) const
{
  if (cloud_pcl_handle_)
    cloud_pcl_handle_->dispatch<l3::UniqueLock>([&](auto& cloud, auto type_trait) { cloud = filterVoxelGrid<decltype(type_trait)>(cloud, lx_, ly_, lz_); });
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::PclVoxelGrridFilter, l3_terrain_modeling::ProcessPlugin)
