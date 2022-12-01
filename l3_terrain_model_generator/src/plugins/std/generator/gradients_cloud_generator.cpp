#include <l3_terrain_model_generator/plugins/std/generator/gradients_cloud_generator.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace l3_terrain_modeling
{
GradientsCloudGenerator::GradientsCloudGenerator()
  : GeneratorPlugin("gradients_cloud_generator")
{
}

bool GradientsCloudGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  gradient_thresh_ = param("gradient_thresh", 0.5);

  return true;
}

bool GradientsCloudGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::initialize(params))
    return false;

  gradients_cloud_handle_ = DataManager::addData("gradients_cloud", boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>());
  if (!gradients_cloud_handle_)
    return false;

  return true;
}

bool GradientsCloudGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  normals_cloud_handle_ = getHandleT<pcl::PointCloud<pcl::PointNormal>::Ptr>("normals_cloud");
  if (!normals_cloud_handle_)
    return false;

  return true;
}

void GradientsCloudGenerator::reset()
{
  GeneratorPlugin::reset();

  l3::UniqueLockPtr lock;
  gradients_cloud_handle_->value<pcl::PointCloud<pcl::PointXYZI>::Ptr>(lock)->clear();
}

void GradientsCloudGenerator::update(const Timer& /*timer*/, UpdatedHandles& input, const SensorPlugin* /*sensor*/)
{
  if (!normals_cloud_handle_)
    return;

  computeGradients();

  input.insert(gradients_cloud_handle_);
}

void GradientsCloudGenerator::computeGradients()
{
  l3::SharedLockPtr normals_cloud_lock;
  pcl::PointCloud<pcl::PointNormal>::ConstPtr normals_cloud = normals_cloud_handle_->value<pcl::PointCloud<pcl::PointNormal>::Ptr>(normals_cloud_lock);

  if (!normals_cloud || normals_cloud->empty())
  {
    ROS_ERROR("Can't generate gradients! Must compute normals first!");
    return;
  }

  l3::UniqueLockPtr cloud_lock;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = gradients_cloud_handle_->value<pcl::PointCloud<pcl::PointXYZI>::Ptr>(cloud_lock);

  // init gradient data structure
  cloud->resize(normals_cloud->size());

  for (unsigned int i = 0; i < normals_cloud->size(); i++)
  {
    const pcl::PointNormal& pn = normals_cloud->at(i);
    pcl::PointXYZI& pi = cloud->at(i);

    pi.x = pn.x;
    pi.y = pn.y;
    pi.z = pn.z;

    pi.intensity = sqrt(pn.normal_x * pn.normal_x + pn.normal_y * pn.normal_y);  // = sqrt(1 - n.normal_z*n.normal_z)
    pi.intensity = pi.intensity < gradient_thresh_ ? 1.0 : 0.0;
  }

  // do voxelizing only after gradient estimation?
  //  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
  //  pcl::VoxelGrid<pcl::PointXYZI> vox;
  //  vox.setInputCloud(cloud_gradient);
  //  vox.setLeafSize(0.01f, 0.01f, 10.0f);
  //  vox.filter(*cloud_filtered);
  //  cloud_gradient = cloud_filtered;
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GradientsCloudGenerator, l3_terrain_modeling::ProcessorPlugin)
