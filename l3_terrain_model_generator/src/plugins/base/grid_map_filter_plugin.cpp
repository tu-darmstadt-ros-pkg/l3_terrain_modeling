#include <l3_terrain_model_generator/plugins/base/grid_map_filter_plugin.h>

namespace l3_terrain_modeling
{
GridMapFilterPlugin::GridMapFilterPlugin(const std::string& name)
  : FilterPlugin(name)
{}

bool GridMapFilterPlugin::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!FilterPlugin::postInitialize(params))
    return false;

  GET_INPUT_HANDLE_DEFAULT(grid_map::GridMap, "grid_map", grid_map_handle_);

  return true;
}
}  // namespace l3_terrain_modeling
