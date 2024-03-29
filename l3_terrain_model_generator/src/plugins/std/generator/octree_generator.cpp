#include <l3_terrain_model_generator/plugins/std/generator/octree_generator.h>

#include <l3_terrain_model_generator/typedefs.h>
#include <l3_terrain_model_generator/utils/pcl/octree_voxel_grid.h>

namespace l3_terrain_modeling
{
OctreeGenerator::OctreeGenerator()
  : GeneratorPlugin("octree_generator")
{}

bool OctreeGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  return true;
}

bool OctreeGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::initialize(params))
    return false;

  // init octree
  double resolution = param("resolution", 0.02, true);
  double update_weight = param("update_weight", 0.75, true);

  const std::string& output_octree_name = getOutputDataParam(getParams(), "output_octree", INPUT_OCTREE_NAME);
  const std::string& point_type = getOutputDataParam(getParams(), "output_octree_point_type", std::string("PointXYZ"));
  octree_pcl_handle_ = PclDataHandle<OctreeVoxelGrid>::makeHandle(this, output_octree_name, point_type, resolution, update_weight);
  if (!octree_pcl_handle_)
  {
    ROS_ERROR("[%s] Unsupported point type: \"%s\"", getName().c_str(), point_type.c_str());
    return false;
  }

  return true;
}

bool OctreeGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  // get pcl handle
  GET_INPUT_PCL_HANDLE_DEFAULT("cloud", cloud_pcl_handle_);
  if (cloud_pcl_handle_->pointType() != octree_pcl_handle_->pointType())
  {
    ROS_ERROR("[%s] Input pointcloud has not same point type (%s) as octree (%s)!", getName().c_str(), toString(cloud_pcl_handle_->pointType()).c_str(),
              toString(octree_pcl_handle_->pointType()).c_str());
    return false;
  }

  return true;
}

void OctreeGenerator::reset()
{
  GeneratorPlugin::reset();

  // clear old data
  if (octree_pcl_handle_)
    octree_pcl_handle_->dispatch<l3::UniqueLock>([](auto& octree, auto type_trait) { octree->deleteTree(); });
}

void OctreeGenerator::update(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor)
{
  if (!octree_pcl_handle_ || !cloud_pcl_handle_)
    return;

  // run only on changes
  if (!updates.has(cloud_pcl_handle_->handle()))
    return;

  // clang-format off
  octree_pcl_handle_->dispatch<l3::UniqueLock>([&](auto& octree, auto type_trait)
  {
    cloud_pcl_handle_->dispatch<l3::SharedLock>([&](auto& cloud, auto type_trait)
    {
      octree->insertPointCloud(cloud);
    });
  });
  // clang-format on

  updates.insert(octree_pcl_handle_->handle());
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::OctreeGenerator, l3_terrain_modeling::ProcessorPlugin)
