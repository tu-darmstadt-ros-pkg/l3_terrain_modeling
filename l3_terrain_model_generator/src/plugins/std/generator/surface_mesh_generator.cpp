#include <l3_terrain_model_generator/plugins/std/generator/surface_mesh_generator.h>

#include <pcl/surface/gp3.h>

#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

namespace l3_terrain_modeling
{
SurfaceMeshGenerator::SurfaceMeshGenerator()
  : GeneratorPlugin("surface_mesh_generator")
{
}

bool SurfaceMeshGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::initialize(params))
    return false;

  // add mesh data structure
  surface_mesh_handle_ = DataManager::addData("surface_mesh", pcl::PolygonMesh());
  if (!surface_mesh_handle_)
    return false;

  return true;
}

bool SurfaceMeshGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  normals_cloud_handle_ = getHandleT<pcl::PointCloud<pcl::PointNormal>::Ptr>("normals_cloud");
  if (!normals_cloud_handle_)
    return false;

  return true;
}

void SurfaceMeshGenerator::reset()
{
  GeneratorPlugin::reset();

  l3::UniqueLockPtr lock;
  surface_mesh_handle_->value<pcl::PolygonMesh>(lock).polygons.clear();
}

void SurfaceMeshGenerator::update(const Timer& /*timer*/, UpdatedHandles& input, const SensorPlugin* /*sensor*/)
{
  // run only on changes
  if (!normals_cloud_handle_)
    return;

  computeMesh();

  input.insert(surface_mesh_handle_);
}

void SurfaceMeshGenerator::computeMesh()
{
  l3::SharedLockPtr normals_cloud_lock;
  pcl::PointCloud<pcl::PointNormal>::ConstPtr normals_cloud = normals_cloud_handle_->value<pcl::PointCloud<pcl::PointNormal>::Ptr>(normals_cloud_lock);

  if (!normals_cloud || normals_cloud->empty())
  {
    ROS_ERROR("Can't generate surface mesh! Must compute normals first!");
    return;
  }

  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud(normals_cloud);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(0.10);

  // Set typical values for the parameters
  gp3.setMu(6.0);
  gp3.setMaximumNearestNeighbors(50);
  gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
  gp3.setMinimumAngle(M_PI / 18);        // 10 degrees
  gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
  gp3.setNormalConsistency(true);
  gp3.setConsistentVertexOrdering(true);

  // Get result
  l3::UniqueLockPtr surface_mesh_lock;
  pcl::PolygonMesh& surface_mesh = surface_mesh_handle_->value<pcl::PolygonMesh>(surface_mesh_lock);

  gp3.setInputCloud(normals_cloud);
  gp3.setSearchMethod(tree);
  gp3.reconstruct(surface_mesh);

  // Additional vertex information
  //  std::vector<int> parts = gp3.getPartIDs();
  //  std::vector<int> states = gp3.getPointStates();
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::SurfaceMeshGenerator, l3_terrain_modeling::ProcessPlugin)
