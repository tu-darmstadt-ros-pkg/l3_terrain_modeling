#include <l3_terrain_model_generator/utils/utils.h>

namespace l3_terrain_modeling
{
void resize(grid_map::GridMap& grid_map, const l3::Vector3& min, const l3::Vector3& max)
{
  grid_map::Length length = grid_map.getLength();
  l3::Vector2 map_min(-0.5 * length.x(), -0.5 * length.y());
  l3::Vector2 map_max(0.5 * length.x(), 0.5 * length.y());

  // check if resize is needed (enlargement only)
  if (min.x() >= map_min.x() && min.y() >= map_min.y() && max.x() <= map_max.x() && max.y() <= map_max.y())
    return;

  grid_map::GridMap new_grid_map;
  grid_map::Length new_length(2 * std::max(std::abs(max.x()), std::abs(min.x())), 2 * std::max(std::abs(max.x()), std::abs(min.x())));
  grid_map::Position new_pos(0.0, 0.0);
  new_grid_map.setGeometry(new_length, grid_map.getResolution(), new_pos);

  grid_map.extendToInclude(new_grid_map);
}
}  // namespace l3_terrain_modeling
