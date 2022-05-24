#include <l3_terrain_model_generator/plugins/base/filter_plugin.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3_terrain_modeling
{
FilterPlugin::FilterPlugin(const std::string& name)
  : ProcessPlugin(name)
  , tf_listener_(tf_buffer_)
{}

bool FilterPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!ProcessPlugin::initialize(params))
    return false;

  robot_pose_sub_ = nh_.subscribe("/initialpose", 1, &FilterPlugin::robotPoseCb, this);

  return true;
}

bool FilterPlugin::determineCurrentRobotPose(l3::Pose& pose) const
{
  // get position
  if (robot_pose_)
  {
    l3::poseMsgToL3(robot_pose_->pose, pose);
    return true;
  }
  else
  {
    ROS_WARN_THROTTLE(10.0, "[%s] determineCurrentPosition: No state estimation of feet available. Defaulting to origin!", getName().c_str());
    pose = l3::Pose();
    return false;
  }
}

void FilterPlugin::robotPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr pose)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = pose->header;
  pose_stamped.pose = pose->pose.pose;
  robot_pose_ = boost::make_shared<geometry_msgs::PoseStamped>(pose_stamped);
}
}  // namespace l3_terrain_modeling
