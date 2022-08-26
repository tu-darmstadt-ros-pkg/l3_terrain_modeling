#include <l3_terrain_model_generator/plugins/std/publisher/surface_mesh_publisher.h>

#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>

#include <l3_terrain_model_generator/utils/pcl/pcl_utils.h>

namespace l3_terrain_modeling
{
SurfaceMeshPublisher::SurfaceMeshPublisher()
  : PublisherPlugin("surface_mesh_publisher")
{}

bool SurfaceMeshPublisher::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::loadParams(params))
    return false;

  return true;
}

bool SurfaceMeshPublisher::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("surface_mesh"), true);
  surface_mesh_pub_ = nh_.advertise<pcl_msgs::PolygonMesh>(topic, 1, latch_topics_);
  surface_mesh_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(topic + "_marker", 1, latch_topics_);

  return true;
}

bool SurfaceMeshPublisher::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::postInitialize(params))
    return false;

  const std::string& input_data_name = param("input_data", std::string("surface_mesh"), true);

  surface_mesh_handle_ = DataManager::getHandle<pcl::PolygonMesh>(input_data_name);
  if (!surface_mesh_handle_)
    return false;

  return true;
}

void SurfaceMeshPublisher::publish(const UpdatedHandles& /*input*/) const
{
  bool has_published = false;

  // generate pcl mesh msg
  if (initial_publish_ || surface_mesh_pub_.getNumSubscribers() > 0)
  {
    if (surface_mesh_handle_)
    {
      pcl_msgs::PolygonMesh mesh_msg;
      l3::SharedLockPtr lock;
      pcl_conversions::fromPCL(surface_mesh_handle_->value<pcl::PolygonMesh>(lock), mesh_msg);
      lock.reset();

      surface_mesh_pub_.publish(mesh_msg);
      has_published = true;
    }
  }

  // generate rviz mesh (as marker)
  if (initial_publish_ || surface_mesh_marker_pub_.getNumSubscribers() > 0)
  {
    if (surface_mesh_handle_)
    {
      std_msgs::ColorRGBA color;
      color.r = 0.75;
      color.g = 0.75;
      color.b = 0.75;
      color.a = 1.0;

      visualization_msgs::Marker marker_msg;
      l3::SharedLockPtr lock;
      fromPCL(surface_mesh_handle_->value<pcl::PolygonMesh>(lock), marker_msg, color, true);
      lock.reset();

      surface_mesh_marker_pub_.publish(marker_msg);
      has_published = true;
    }
  }

  if (has_published)
    initial_publish_ = false;
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::SurfaceMeshPublisher, l3_terrain_modeling::ProcessPlugin)
