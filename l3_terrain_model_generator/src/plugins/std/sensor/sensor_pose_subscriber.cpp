#include <l3_terrain_model_generator/plugins/std/sensor/sensor_pose_subscriber.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3_terrain_modeling
{
template <class T, class M>
bool isInstance(const M& msg)
{
  return msg->getDataType() == ros::message_traits::datatype<T>();
}

SensorPoseSubscriber::SensorPoseSubscriber()
  : ProcessPlugin("sensor_pose_subscriber")
{}

bool SensorPoseSubscriber::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!ProcessPlugin::initialize(params))
    return false;

  std::string topic = param("topic", std::string("pose"), true);
  pose_sub_ = nh_.subscribe(topic, 1, &SensorPoseSubscriber::poseCb, this);

  // load sensors to update
  std::vector<std::string> sensors = param("sensors", std::vector<std::string>(), true);
  sensors_ = boost::make_shared<Sensors>(getName() + "::Sensors");
  sensors_->loadPlugins(sensors, false);

  return true;
}

void SensorPoseSubscriber::poseCb(topic_tools::ShapeShifter::ConstPtr msg)
{
  l3::StampedPose pose;

  // dispatch pose type and deserialize message
  if (isInstance<geometry_msgs::Pose>(msg))
  {
    geometry_msgs::Pose::Ptr p = msg->instantiate<geometry_msgs::Pose>();
    pose.header.stamp = ros::Time::now();
    l3::poseMsgToL3(*p, pose.data);
  }
  else if (isInstance<geometry_msgs::PoseStamped>(msg))
  {
    geometry_msgs::PoseStamped::Ptr p = msg->instantiate<geometry_msgs::PoseStamped>();
    pose.header = p->header;
    l3::poseMsgToL3(p->pose, pose.data);
  }
  else if (isInstance<geometry_msgs::PoseWithCovariance>(msg))
  {
    geometry_msgs::PoseWithCovariance::Ptr p = msg->instantiate<geometry_msgs::PoseWithCovariance>();
    pose.header.stamp = ros::Time::now();
    l3::poseMsgToL3(p->pose, pose.data);
  }
  else if (isInstance<geometry_msgs::PoseWithCovarianceStamped>(msg))
  {
    geometry_msgs::PoseWithCovarianceStamped::Ptr p = msg->instantiate<geometry_msgs::PoseWithCovarianceStamped>();
    pose.header = p->header;
    l3::poseMsgToL3(p->pose.pose, pose.data);
  }
  else
  {
    ROS_WARN("[%s] Received unsupported pose type (%s)!", getName().c_str(), msg->getDataType().c_str());
    return;
  }

  // set sensor pose
  sensors_->setSensorPose(pose);
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::SensorPoseSubscriber, l3_terrain_modeling::ProcessPlugin)
