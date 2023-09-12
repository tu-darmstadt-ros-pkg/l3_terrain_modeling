#include <l3_terrain_model_generator/plugins/std/publisher/octree_publisher.h>

#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>

#include <l3_terrain_model_generator/typedefs.h>

namespace l3_terrain_modeling
{
OctreePublisher::OctreePublisher()
  : PublisherPlugin("octree_publisher")
{}

bool OctreePublisher::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::loadParams(params))
    return false;

  return true;
}

bool OctreePublisher::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("octree"), true);
  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(topic + "_cloud", 1, latch_topics_);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(topic + "_markers", 1, latch_topics_);

  return true;
}

bool OctreePublisher::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::postInitialize(params))
    return false;

  // get pcl handle
  const std::string& input_data_name = getInputDataParam(getParams(), "input_data", INPUT_OCTREE_NAME);
  octree_pcl_handle_ = PclDataHandle<OctreeVoxelGrid>::makeHandle(this, input_data_name);
  if (!octree_pcl_handle_)
  {
    ROS_ERROR("[%s] Data handle \"%s\" seems not to contain valid pcl data!", getName().c_str(), input_data_name.c_str());
    return false;
  }

  return true;
}

void OctreePublisher::publish(const UpdatedHandles& /*input*/) const
{
  bool has_published = false;

  // publish as pointcloud
  if (initial_publish_ || cloud_pub_.getNumSubscribers() > 0)
  {
    if (octree_pcl_handle_)
    {
      sensor_msgs::PointCloud2 msg;

      octree_pcl_handle_->dispatch<l3::SharedLock>([&](auto& octree, auto type_trait) {
        pcl::PointCloud<decltype(type_trait)> pc;
        octree->getPointCloud(pc);
        pcl::toROSMsg(pc, msg);
      });

      cloud_pub_.publish(msg);
      has_published = true;
    }
  }

  // publish as marker array
  if (initial_publish_ || markers_pub_.getNumSubscribers() > 0)
  {
    if (octree_pcl_handle_)
    {
      /// @todo
      ROS_ERROR_ONCE("Octree Markerarray visualization is not supported yet.");
      visualization_msgs::MarkerArray msg;
      markers_pub_.publish(msg);
      has_published = true;
    }
  }

  if (has_published)
    initial_publish_ = false;
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::OctreePublisher, l3_terrain_modeling::ProcessorPlugin)
