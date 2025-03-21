#include <l3_terrain_model_generator/plugins/std/generator/grid_map_pose_updater.h>
#include <grid_map_core/GridMapMath.hpp>

#include <l3_terrain_model_generator/utils/utils.h>

namespace l3_terrain_modeling
{
GridMapPoseUpdater::GridMapPoseUpdater()
  : GeneratorPlugin("grid_map_pose_updater")
  , tf_listener_(tf_buffer_)
{}

bool GridMapPoseUpdater::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  fixed_frame_id_ = param("fixed_frame", std::string("base_link"), true);
  propagate_frame_update_ = param("propagate_frame_update", true, true);

  return true;
}

bool GridMapPoseUpdater::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  GET_INPUT_HANDLE_DEFAULT(grid_map::GridMap, "grid_map", grid_map_handle_);

  return true;
}

void GridMapPoseUpdater::update(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor)
{
  l3::UniqueLockPtr lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(lock);

  if (propagate_frame_update_)
  {
    ros::Time time_past;
    time_past.fromNSec(grid_map.getTimestamp());
    ros::Time time_now = Timer::timeToRos(timer.current);

    // get transformation since last update
    l3::StampedPose pose;
    if (getDeltaTransformAsPose(tf_buffer_, grid_map.getFrameId(), fixed_frame_id_, time_past, time_now, pose))
    {
      // update grid map position
      grid_map.move(grid_map.getPosition() + grid_map::Position(pose.data.x(), pose.data.y()));
      grid_map.setTimestamp(time_now.toNSec());

      updates.insert(grid_map_handle_);
    }
    else
    {
      ROS_WARN_THROTTLE(5.0, "[%s] Could not update grid map pose!", getName().c_str());
    }
  }
  else
  {
    ROS_WARN_ONCE("[%s] Frame update propagation is disabled! No further modes are supported yet.", getName().c_str());
  }
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GridMapPoseUpdater, l3_terrain_modeling::ProcessorPlugin)
