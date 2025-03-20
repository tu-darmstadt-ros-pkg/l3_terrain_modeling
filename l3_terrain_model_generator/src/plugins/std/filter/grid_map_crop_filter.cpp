#include <l3_terrain_model_generator/plugins/std/filter/grid_map_crop_filter.h>

namespace l3_terrain_modeling
{
GridMapCropFilter::GridMapCropFilter()
  : GridMapFilterPlugin("grid_map_crop_filter")
{}

bool GridMapCropFilter::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapFilterPlugin::loadParams(params))
    return false;

  // parse parameters
  std::vector<double> center;
  if (getParam("center", center, std::vector<double>(), true))
  {
    if (center.size() != 2)
    {
      ROS_ERROR("[%s] Center must be a 2D vector", getName().c_str());
      return false;
    }
    center_ = grid_map::Position(center[0], center[1]);
  }

  std::vector<double> size = param("size", std::vector<double>());
  if (size.size() != 2)
  {
    ROS_ERROR("[%s] Size must be a 2D vector", getName().c_str());
    return false;
  }
  size_ = grid_map::Length(size[0], size[1]);

  return true;
}

void GridMapCropFilter::filter(UpdatedHandles& updates, const SensorPlugin* /*sensor*/) const
{
  // run only on changes
  if (!updates.has(grid_map_handle_))
    return;

  if (!grid_map_handle_)
    return;

  l3::UniqueLockPtr lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(lock);

  // crop grid map
  bool success = false;
  grid_map::GridMap cropped_map = grid_map.getSubmap(grid_map.getPosition() + center_, size_, success);
  if (success)
    grid_map = cropped_map;
  else
    ROS_ERROR_THROTTLE(1.0, "[%s] Failed to crop grid map", getName().c_str());
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GridMapCropFilter, l3_terrain_modeling::ProcessorPlugin)
