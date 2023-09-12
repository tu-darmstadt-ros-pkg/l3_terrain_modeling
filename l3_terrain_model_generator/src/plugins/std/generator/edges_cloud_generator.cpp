#include <l3_terrain_model_generator/plugins/std/generator/edges_cloud_generator.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace l3_terrain_modeling
{
EdgesCloudGenerator::EdgesCloudGenerator()
  : GeneratorPlugin("edges_cloud_generator")
{}

bool EdgesCloudGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  radius_ = param("radius", 0.05);
  max_std_ = param("max_std", 0.022);
  non_max_supp_radius_ = param("non_max_supp_radius", 0.05);

  return true;
}

bool EdgesCloudGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::initialize(params))
    return false;

  GET_OUTPUT_HANDLE_DEFAULT(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(), "edges_cloud", edges_cloud_handle_);

  return true;
}

bool EdgesCloudGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  GET_INPUT_HANDLE_DEFAULT(pcl::PointCloud<pcl::PointNormal>::Ptr, "normals_cloud", normals_cloud_handle_);

  return true;
}

void EdgesCloudGenerator::reset()
{
  GeneratorPlugin::reset();

  l3::UniqueLockPtr lock;
  edges_cloud_handle_->value<pcl::PointCloud<pcl::PointXYZI>::Ptr>(lock)->clear();
}

void EdgesCloudGenerator::update(const Timer& /*timer*/, UpdatedHandles& updates, const SensorPlugin* /*sensor*/)
{
  if (!normals_cloud_handle_)
    return;

  // run only on changes
  if (!updates.has(normals_cloud_handle_))
    return;

  detectEdges();

  updates.insert(edges_cloud_handle_);
}

void EdgesCloudGenerator::detectEdges()
{
  l3::SharedLockPtr normals_cloud_lock;
  pcl::PointCloud<pcl::PointNormal>::ConstPtr normals_cloud = normals_cloud_handle_->value<pcl::PointCloud<pcl::PointNormal>::Ptr>(normals_cloud_lock);

  if (!normals_cloud || normals_cloud->empty())
  {
    ROS_ERROR("Can't run edge detection! Must compute normals first!");
    return;
  }

  // init edge data structure
  l3::UniqueLockPtr cloud_lock;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = edges_cloud_handle_->value<pcl::PointCloud<pcl::PointXYZI>::Ptr>(cloud_lock);
  cloud->resize(normals_cloud->size());

  // project all data to plane
  pcl::PointCloud<pcl::PointXY>::Ptr points(new pcl::PointCloud<pcl::PointXY>());
  points->resize(normals_cloud->size());
  for (unsigned int i = 0; i < normals_cloud->size(); i++)
  {
    const pcl::PointNormal& n = normals_cloud->at(i);
    pcl::PointXY& p = points->at(i);
    p.x = n.x;
    p.y = n.y;
  }

  pcl::KdTreeFLANN<pcl::PointXY> tree;
  tree.setInputCloud(points);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_edges(new pcl::PointCloud<pcl::PointXYZI>());
  tmp_edges->resize(normals_cloud->size());

  // run edge detection
  for (size_t i = 0; i < points->size(); i++)
  {
    const pcl::PointNormal& current = normals_cloud->at(i);
    pcl::PointXYZI& result = tmp_edges->at(i);

    result.x = current.x;
    result.y = current.y;
    result.z = current.z;
    result.intensity = 0.0;

    if (tree.radiusSearch(i, radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      // determine squared mean error
      double sq_sum_e = 0.0;

      for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
      {
        if (pointIdxRadiusSearch[j] == (int)i)
          continue;

        const pcl::PointNormal& neigh = normals_cloud->at(pointIdxRadiusSearch[j]);

        // determine diff of normals
        double diff_nx = (neigh.normal_x - current.normal_x);
        double diff_ny = (neigh.normal_y - current.normal_y);
        double sq_err_nx = diff_nx * diff_nx;
        double sq_err_ny = diff_ny * diff_ny;

        // determine diff in height
        double diff_z = (neigh.z - current.z) * 5.0;  // scale up diff (weight)
        double sq_err_z = diff_z * diff_z;

        // compute support in direction of normal
        double neigh_norm = sqrt((current.normal_x * current.normal_x + current.normal_y * current.normal_y) *
                                 ((neigh.x - current.x) * (neigh.x - current.x) + (neigh.y - current.y) * (neigh.y - current.y)));
        double dot_scale = std::abs(current.normal_x * std::abs(neigh.x - current.x) + current.normal_y * std::abs(neigh.y - current.y)) / neigh_norm;

        // compute support in distance
        double dist_scale = (radius_ - sqrt(pointRadiusSquaredDistance[j])) / radius_;

        sq_sum_e += (sq_err_nx + sq_err_ny + sq_err_z) * dot_scale * dist_scale;
      }

      double sq_mean_e = sq_sum_e / pointIdxRadiusSearch.size();

      // check for edge
      if (sq_mean_e > max_std_ * max_std_)
        result.intensity = sq_mean_e;
    }
  }

  // run non-maximum suppression
  for (size_t i = 0; i < points->size(); i++)
  {
    const pcl::PointNormal& current = normals_cloud->at(i);
    const pcl::PointXYZI& edge = tmp_edges->at(i);
    pcl::PointXYZI& result = cloud->at(i);

    result.x = edge.x;
    result.y = edge.y;
    result.z = edge.z;
    result.intensity = 1.0;  // = no edge

    if (edge.intensity == 0.0)
      continue;

    if (tree.radiusSearch(i, non_max_supp_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      bool local_max = true;

      // determine local max
      for (size_t j = 0; j < pointIdxRadiusSearch.size(); j++)
      {
        if (pointIdxRadiusSearch[j] == (int)i)
          continue;

        const pcl::PointXYZI& neigh = tmp_edges->at(pointIdxRadiusSearch[j]);

        /// @TODO: Handle 0/0/1 normals
        double neigh_norm = sqrt((current.normal_x * current.normal_x + current.normal_y * current.normal_y) *
                                 ((neigh.x - current.x) * (neigh.x - current.x) + (neigh.y - current.y) * (neigh.y - current.y)));
        double dot_scale = std::abs(current.normal_x * std::abs(neigh.x - current.x) + current.normal_y * std::abs(neigh.y - current.y)) / neigh_norm;

        // check for local maximum
        if (edge.intensity < neigh.intensity * dot_scale)
        {
          local_max = false;
          break;
        }
      }

      // check for edge
      if (local_max)
        result.intensity = 0.0;
    }
  }
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::EdgesCloudGenerator, l3_terrain_modeling::ProcessorPlugin)
