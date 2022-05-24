#include <l3_terrain_model_generator/utils/pcl/octree_voxel_grid.h>

namespace l3_terrain_modeling
{
template <>
void OctreeCentroidContainer<pcl::PointNormal>::addPoint(const pcl::PointNormal& new_point, double update_weight)
{
  if (!updated_once_)
  {
    update_weight = 1.0;
    updated_once_ = true;
  }

  point_.x += update_weight * (new_point.x - point_.x);
  point_.y += update_weight * (new_point.y - point_.y);
  point_.z += update_weight * (new_point.z - point_.z);

  point_.normal_x += update_weight * (new_point.normal_x - point_.normal_x);
  point_.normal_y += update_weight * (new_point.normal_y - point_.normal_y);
  point_.normal_z += update_weight * (new_point.normal_z - point_.normal_z);
}

template <>
void OctreeCentroidContainer<pcl::PointNormal>::reset()
{
  point_.x = point_.y = point_.z = 0;
  point_.normal_x = point_.normal_y = point_.normal_z = 0;
  updated_once_ = false;
}
}  // namespace l3_terrain_modeling
