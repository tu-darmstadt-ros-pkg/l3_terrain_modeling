#include <l3_terrain_model_generator/plugins/std/generator/elevation_map_generator.h>

#include <grid_map_core/GridMapMath.hpp>

#include <l3_terrain_model_generator/utils/pcl/pcl_utils.h>

namespace l3_terrain_modeling
{
ElevationMapGenerator::ElevationMapGenerator()
  : GridMapGeneratorPlugin("elevation_map_generator")
{}

bool ElevationMapGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapGeneratorPlugin::loadParams(params))
    return false;

  update_weight_ = param("update_weight", 0.75);
  use_color_ = param("colored", false, true);

  return true;
}

bool ElevationMapGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapGeneratorPlugin::initialize(params))
    return false;

  l3::UniqueLockPtr lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(lock);

  grid_map.add(TerrainModel::ELEVATION_LAYER);

  if (use_color_)
    grid_map.add(TerrainModel::RGB_LAYER);

  return true;
}

// void ElevationMapGenerator::update(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
//{
//  if (cloud->empty())
//    return;

//  update own octree if no external is available if (!external_input_octree_ && input_octree_) input_octree_->insertPointCloud(cloud);

//  const OctreeVoxelGrid<pcl::PointXYZ>::ConstPtr& octree = external_input_octree ? external_input_octree : input_octree;

//  pcl::PointCloud<pcl::PointXYZ>::VectorType points;
//  octree->getVoxelCentroids(points);

//  // fill missing data
//  pcl::KdTreeFLANN<pcl::PointNormal> tree;
//  tree.setInputCloud(cloud_points_with_normals);
//  pcl::PointNormal current;
//  current.z = ground_z;

//  unsigned int k = 100;
//  std::vector<int> pointIdxNKNSearch;
//  std::vector<float> pointNKNSquaredDistance;
//  pcl::PointNormal result;

//  pointIdxNKNSearch.resize(k);
//  pointNKNSquaredDistance.resize(k);

//  int8_t ground_level_height = (int8_t)floor((ground_z - min.z) * height_scale_inv) - 127;

//  for (size_t i = 0; i < height_grid_map.data.size(); i++)
//  {
//    int8_t& height = height_grid_map.data.at(i);
//    if (height == -128)
//    {
//      if (params.gg_reconstruct)
//      {
//        getGridMapCoords(height_grid_map, i, current.x, current.y);

//        // find k nearest neighbour and use their z
//        if (tree.nearestKSearch(current, k, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//        {
//          double z = 0.0;
//          float total_dist = 0.0;

//          for (size_t n = 0; n < k; n++)
//            total_dist += pointNKNSquaredDistance[n];

//          for (size_t n = 0; n < k; n++)
//          {
//            result = cloud_points_with_normals->points[pointIdxNKNSearch[n]];
//            z += result.z * (1 - pointNKNSquaredDistance[n] / total_dist);
//          }
//          z /= k;

//          height = (int8_t)floor((z - min.z) * height_scale_inv) - 127;

//          result.x = current.x;
//          result.y = current.y;
//          result.z = z;
//          cloud_points_with_normals->push_back(result);
//        }
//      }
//      else
//        height = ground_level_height;
//    }
//  }
//}

void ElevationMapGenerator::update(const Timer& timer, UpdatedHandles& input, const SensorPlugin* sensor)
{
  l3::UniqueLockPtr lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(lock);

  // clang-format off
  cloud_pcl_handle_->dispatch<l3::SharedLock>([&](auto& cloud, auto type_trait)
  {
    grid_map::Index index;

    for (const decltype(type_trait)& point : *cloud)
    {
      if (!grid_map.getIndex(l3::Position2D(static_cast<double>(point.x), static_cast<double>(point.y)), index))
        continue;

      pcl::RGB color;
      if (use_color_)
        color = getPointColor(point);
      else
        color.a = 0;

      updateCell(grid_map, index, static_cast<float>(point.z), color);
    }
  });
  // clang-format on
}

void ElevationMapGenerator::updateCell(grid_map::GridMap& grid_map, const grid_map::Index& index, float height, const pcl::RGB& color)
{
  // update height
  if (grid_map.isValid(index, TerrainModel::ELEVATION_LAYER))
  {
    float& h = grid_map.at(TerrainModel::ELEVATION_LAYER, index);
    h += update_weight_ * (height - h);
  }
  else
    grid_map.at(TerrainModel::ELEVATION_LAYER, index) = height;

  // add color information
  if (color.a > 0)
    grid_map.at(TerrainModel::RGB_LAYER, index) = color.rgb;
  /// @todo define fallback color
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::ElevationMapGenerator, l3_terrain_modeling::ProcessPlugin)
