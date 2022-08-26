#include <l3_terrain_model_generator/plugins/std/publisher/normals_publisher.h>

#include <geometry_msgs/PoseArray.h>

#include <pcl_conversions/pcl_conversions.h>

#include <l3_math/math.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3_terrain_modeling
{
NormalsPublisher::NormalsPublisher()
  : PublisherPlugin("normals_publisher")
{}

bool NormalsPublisher::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::loadParams(params))
    return false;

  return true;
}

bool NormalsPublisher::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("normals"), true);
  normals_pub_ = nh_.advertise<geometry_msgs::PoseArray>(topic, 1, latch_topics_);

  return true;
}

bool NormalsPublisher::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!PublisherPlugin::postInitialize(params))
    return false;

  const std::string& input_data_name = param("input_data", std::string("normals_cloud"), true);

  normals_cloud_handle_ = getHandleT<pcl::PointCloud<pcl::PointNormal>::Ptr>(input_data_name);
  if (!normals_cloud_handle_)
    return false;

  return true;
}

void NormalsPublisher::publish(const UpdatedHandles& input) const
{
  // generate pcl mesh msg
  if (initial_publish_ || normals_pub_.getNumSubscribers() > 0)
  {
    if (normals_cloud_handle_)
    {
      l3::SharedLockPtr lock;
      pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud = normals_cloud_handle_->value<pcl::PointCloud<pcl::PointNormal>::Ptr>(lock);

      geometry_msgs::PoseArray msg;
      pcl_conversions::fromPCL(cloud->header, msg.header);

      for (size_t i = 0; i < cloud->size(); i++)
      {
        // determine roll and pitch
        const pcl::PointNormal& p_n = cloud->at(i);

        l3::Vector3 n(static_cast<double>(p_n.normal_x), static_cast<double>(p_n.normal_y), static_cast<double>(p_n.normal_z));

        double r, p;
        l3::normalToRP(n, 0.0, r, p);

        // transformation from normal vector (z-direction) to pose array (x-direction)
        l3::Transform trans_to_pose_array(0.0, 0.0, 0.0, -M_PI_2, -M_PI_2, 0.0);
        l3::Transform t(0.0, 0.0, 0.0, r, p, 0.0);
        t = trans_to_pose_array * t;

        // compose pose msg
        geometry_msgs::Pose pose_msg;
        l3::poseL3ToMsg(t, pose_msg);
        pose_msg.position.x = static_cast<double>(p_n.x);
        pose_msg.position.y = static_cast<double>(p_n.y);
        pose_msg.position.z = static_cast<double>(p_n.z);

        msg.poses.push_back(pose_msg);
      }

      lock.reset();

      normals_pub_.publish(msg);
      initial_publish_ = false;
    }
  }
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::NormalsPublisher, l3_terrain_modeling::ProcessPlugin)
