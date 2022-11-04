#include <l3_terrain_model_generator/plugins/std/generator/normals_cloud_generator.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>

#include <l3_libs/yaml_parser.h>

#include <l3_terrain_model_generator/typedefs.h>
#include <l3_terrain_model_generator/utils/pcl/point_cloud_filter.h>

namespace l3_terrain_modeling
{
NormalsCloudGenerator::NormalsCloudGenerator()
  : GeneratorPlugin("normals_cloud_generator")
{}

bool NormalsCloudGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  threads_ = param("threads", 0, true);
  min_aggregation_size_ = param("min_aggregation_size", 10000u, true);
  filter_mask_ = param("filter_mask", 0u);

  ne_radius_ = param("ne_radius", 0.05);

  // load filter parameters
  if (filter_mask_ & FILTER_VOXEL_GRID)
  {
    XmlRpc::XmlRpcValue p = param("voxel_grid_size", XmlRpc::XmlRpcValue());
    if (!getYamlValue(p, voxel_grid_size_))
      ROS_ERROR("[%s] initialize: Failed to read \"voxel_grid_size\" parameter!", getName().c_str());
  }

  if (filter_mask_ & FILTER_MLS_SMOOTH)
    ms_radius_ = param("ms_radius", 0.1);

  if (filter_mask_ & FILTER_STATISTICAL_OUTLIER)
  {
    so_radius_ = param("so_radius", 0.01);
    so_k_ = param("so_k", 25);
  }

  return true;
}

bool NormalsCloudGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::initialize(params))
    return false;

  // init octree
  double resolution = param("resolution", 0.05);
  double update_weight = param("update_weight", 0.75);

  normals_octree_handle_ = DataManager::addData("normals_octree", OctreeVoxelGrid<pcl::PointNormal>(resolution, update_weight));
  normals_cloud_handle_ = DataManager::addData("normals_cloud", boost::make_shared<pcl::PointCloud<pcl::PointNormal>>());

  if (!normals_octree_handle_ || !normals_cloud_handle_)
    return false;

  return true;
}

bool NormalsCloudGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  const std::string& input_data_name = param("input_data", std::string("cloud"), true);

  // get input octree pcl handle
  input_octree_pcl_handle_ = PclDataHandle<OctreeVoxelGrid>::makeHandle(INPUT_OCTREE_NAME);
  if (!input_octree_pcl_handle_)
  {
    ROS_ERROR("[%s] Data handle \"%s\" seems not to contain valid pcl data!", getName().c_str(), INPUT_OCTREE_NAME);
    return false;
  }

  // get input cloud pcl handle
  input_cloud_pcl_handle_ = PclDataHandle<pcl::PointCloud>::makeHandle(input_data_name);
  if (!input_cloud_pcl_handle_)
  {
    ROS_ERROR("[%s] Data handle \"%s\" seems not to contain valid pcl data!", getName().c_str(), input_data_name.c_str());
    return false;
  }

  grid_map_handle_ = getHandleT<grid_map::GridMap>(GRID_MAP_NAME);
  if (!grid_map_handle_)
    return false;

  // add normal layers
  l3::UniqueLockPtr lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(lock);
  grid_map.add(TerrainModel::NORMAL_LAYER_PREFIX + "x");
  grid_map.add(TerrainModel::NORMAL_LAYER_PREFIX + "y");
  grid_map.add(TerrainModel::NORMAL_LAYER_PREFIX + "z");

  return true;
}

void NormalsCloudGenerator::reset()
{
  GeneratorPlugin::reset();

  l3::UniqueLockPtr normals_octree_lock;
  normals_octree_handle_->value<OctreeVoxelGrid<pcl::PointNormal>>(normals_octree_lock).deleteTree();

  l3::UniqueLockPtr normals_cloud_lock;
  normals_cloud_handle_->value<pcl::PointCloud<pcl::PointNormal>::Ptr>(normals_cloud_lock)->clear();

  l3::UniqueLockPtr grid_map_lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(grid_map_lock);
  grid_map.clear(TerrainModel::NORMAL_LAYER_PREFIX + "x");
  grid_map.clear(TerrainModel::NORMAL_LAYER_PREFIX + "y");
  grid_map.clear(TerrainModel::NORMAL_LAYER_PREFIX + "z");
}

void NormalsCloudGenerator::update(const Timer& /*timer*/, UpdatedHandles& input, const SensorPlugin* /*sensor*/)
{
  if (!input_octree_pcl_handle_ || !input_cloud_pcl_handle_)
    return;

  // determine search points
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_points = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  input_cloud_pcl_handle_->dispatch<l3::SharedLock>([&](auto& cloud, auto type_trait)
  {
    pcl::copyPointCloud(*cloud, *search_points);
  });

  computeNormals(search_points);

  input.insert(normals_octree_handle_);
  input.insert(normals_cloud_handle_);
  input.insert(grid_map_handle_);
}

void NormalsCloudGenerator::computeNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr search_points)
{
  // do not process any data before sufficient points have been collected
  unsigned int leaf_count = 0;
  input_octree_pcl_handle_->dispatch<l3::SharedLock>([&](auto& octree, auto type_trait)
  {
    leaf_count = octree->getLeafCount();
  });

  if (leaf_count < min_aggregation_size_)
    return;

  // reduce search points to a minimum
  if (filter_mask_ & FILTER_VOXEL_GRID)
    search_points = filterVoxelGrid<pcl::PointXYZ>(search_points, voxel_grid_size_.x(), voxel_grid_size_.y(), voxel_grid_size_.z());

  // init normals data structure
  pcl::PointCloud<pcl::Normal> normals_cloud;

  // lock grid map
  l3::UniqueLockPtr grid_map_lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(grid_map_lock);

  // enforce search points to be within grid map
  search_points = filterInGridMap<pcl::PointXYZ>(search_points, grid_map);

  if (search_points->isOrganized())
  {
    // from https://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.html#normal-estimation-using-integral-images
    // estimate normals
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(search_points);
    ne.compute(normals_cloud);
  }
  else
  {
    input_octree_pcl_handle_->dispatch<l3::SharedLock>([&](auto& octree, auto type_trait)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      std::vector<float> k_sqr_distances;

      octree->radiusSearch(*search_points, 1.2 * ne_radius_, *processed_cloud, k_sqr_distances);

      if (processed_cloud->empty())
        return;

      // preprocessing of search surface
      if (filter_mask_ & FILTER_MLS_SMOOTH)  // smooth data
        processed_cloud = filterMlsSmooth<pcl::PointXYZ>(processed_cloud, ms_radius_);

      // compute normals
      pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne(threads_);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
      ne.setInputCloud(search_points);
      ne.setSearchSurface(processed_cloud);
      ne.setSearchMethod(tree);
      ne.setRadiusSearch(ne_radius_);
      // ne.setKSearch(20);
      ne.compute(normals_cloud);
    });
  }

  // concat resulting normals with their positions
  pcl::PointCloud<pcl::PointNormal>::Ptr new_points_with_normals = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  pcl::concatenateFields(*search_points, normals_cloud, *new_points_with_normals);

  // postprocessing
  if (filter_mask_ & FILTER_STATISTICAL_OUTLIER)  // remove outliers
    new_points_with_normals = filterStatisticalOutlier<pcl::PointNormal>(new_points_with_normals, so_k_, so_radius_);

  // filter and some further postprocessing
  std::vector<int> indices;
  for (size_t i = 0; i < new_points_with_normals->size(); i++)
  {
    pcl::PointNormal& n = new_points_with_normals->at(i);
    if (!std::isfinite(n.normal_z))
      continue;

    indices.push_back((int)i);

    // flip all other normals in one direction
    if (n.normal_z < 0.0)
    {
      n.normal_x = -n.normal_x;
      n.normal_y = -n.normal_y;
      n.normal_z = -n.normal_z;
    }
  }

  // apply filtering
  pcl::PointCloud<pcl::PointNormal>::Ptr normals_filtered = boost::make_shared<pcl::PointCloud<pcl::PointNormal>>(*new_points_with_normals, indices);

  // update oct-tree
  l3::UniqueLockPtr normals_octree_lock;
  OctreeVoxelGrid<pcl::PointNormal>& normals_octree = normals_octree_handle_->value<OctreeVoxelGrid<pcl::PointNormal>>(normals_octree_lock);
  normals_octree.insertPointCloud(normals_filtered);

  // retrieve updated pointcloud with normals
  l3::UniqueLockPtr normals_cloud_lock;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud = normals_cloud_handle_->value<pcl::PointCloud<pcl::PointNormal>::Ptr>(normals_cloud_lock);
  normals_octree.getPointCloud(*cloud);
  cloud->header = normals_filtered->header;
  normals_cloud_lock.reset();
  normals_octree_lock.reset();

  // update grid_map
  for (size_t i = 0; i < normals_filtered->size(); i++)
  {
    const pcl::PointNormal& n = normals_filtered->at(i);

    grid_map::Index index;
    grid_map.getIndex(l3::Position2D(static_cast<double>(n.x), static_cast<double>(n.y)), index);
    grid_map.at(TerrainModel::NORMAL_LAYER_PREFIX + "x", index) = n.normal_x;
    grid_map.at(TerrainModel::NORMAL_LAYER_PREFIX + "y", index) = n.normal_y;
    grid_map.at(TerrainModel::NORMAL_LAYER_PREFIX + "z", index) = n.normal_z;
  }

  grid_map.setTimestamp(cloud->header.stamp);

  grid_map_lock.reset();
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::NormalsCloudGenerator, l3_terrain_modeling::ProcessPlugin)
