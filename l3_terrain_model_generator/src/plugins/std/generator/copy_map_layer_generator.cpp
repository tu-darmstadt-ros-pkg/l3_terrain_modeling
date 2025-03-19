#include <l3_terrain_model_generator/plugins/std/generator/copy_map_layer_generator.h>

#include <grid_map_core/GridMapMath.hpp>

#include <l3_terrain_model/typedefs.h>

namespace l3_terrain_modeling
{
CopyMapLayerGenerator::CopyMapLayerGenerator()
  : GeneratorPlugin("copy_map_layer_generator")
{}

bool CopyMapLayerGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  in_layer_ = param("in_layer", ELEVATION_LAYER, true);
  out_layer_ = param("out_layer", ELEVATION_LAYER + "_copy", false);

  return true;
}

bool CopyMapLayerGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  GET_INPUT_HANDLE_DEFAULT(grid_map::GridMap, "grid_map", grid_map_handle_);

  l3::UniqueLockPtr lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(lock);

  grid_map.add(out_layer_);

  return true;
}

void CopyMapLayerGenerator::update(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor)
{
  l3::UniqueLockPtr lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(lock);

  grid_map.add(out_layer_, grid_map.get(in_layer_));
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::CopyMapLayerGenerator, l3_terrain_modeling::ProcessorPlugin)
