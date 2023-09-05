#include <l3_terrain_model_generator/utils/utils.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

namespace l3_terrain_modeling
{
bool getTransformAsPose(const tf2_ros::Buffer& tf_buffer, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, l3::Pose& pose)
{
  std::string error_msg;
  if (tf_buffer.canTransform(target_frame, source_frame, time, ros::Duration(1.0), &error_msg))
  {
    geometry_msgs::TransformStamped transform_msg = tf_buffer.lookupTransform(target_frame, source_frame, time);
    l3::transformMsgToL3(transform_msg.transform, pose);
    return true;
  }
  else
  {
    ROS_ERROR_THROTTLE(5.0, "Could not retrieve transform '%s' to '%s':\n%s", source_frame, target_frame, error_msg.c_str());
    return false;
  }
}

bool getTransformAsPose(const tf2_ros::Buffer& tf_buffer, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, l3::StampedPose& pose)
{
  if (getTransformAsPose(tf_buffer, target_frame, source_frame, time, pose.data))
  {
    pose.header.frame_id = target_frame;
    pose.header.stamp = time;
    return true;
  }
  else
    return false;
}

void resize(grid_map::GridMap& grid_map, const l3::Vector3& min, const l3::Vector3& max)
{
  grid_map::Length length = grid_map.getLength();
  l3::Vector2 map_min(-0.5 * length.x(), -0.5 * length.y());
  l3::Vector2 map_max(0.5 * length.x(), 0.5 * length.y());

  // check if resize is needed (enlargement only)
  if (min.x() >= map_min.x() && min.y() >= map_min.y() && max.x() <= map_max.x() && max.y() <= map_max.y())
    return;

  grid_map::GridMap new_grid_map;
  grid_map::Length new_length(2 * std::max(std::abs(max.x()), std::abs(min.x())), 2 * std::max(std::abs(max.x()), std::abs(min.x())));
  grid_map::Position new_pos(0.0, 0.0);
  new_grid_map.setGeometry(new_length, grid_map.getResolution(), new_pos);

  grid_map.extendToInclude(new_grid_map);
}
}  // namespace l3_terrain_modeling
