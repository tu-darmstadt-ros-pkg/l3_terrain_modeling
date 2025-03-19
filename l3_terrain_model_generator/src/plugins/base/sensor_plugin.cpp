#include <l3_terrain_model_generator/plugins/base/sensor_plugin.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

#include <l3_terrain_model_generator/utils/helper.h>
#include <l3_terrain_model_generator/utils/utils.h>
#include <l3_terrain_model_generator/plugins/aggregator/process_chain.h>

namespace l3_terrain_modeling
{
SensorPlugin::SensorPlugin(const std::string& name)
  : Plugin(name)
  , tf_listener_(tf_buffer_)
{
}

bool SensorPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  sensor_frame_id_ = param("sensor_frame", std::string("lidar"), true);
  map_frame_id_ = param("map_frame", std::string("map"), true);
  auto_update_sensor_pose_ = param("auto_update_sensor_pose", true, true);

  double rate = param("rate", 0.0, true);
  process_intervall_ = rate > 0.0 ? static_cast<uint64_t>(1000.0 / rate) : 0llu;

  wait_for_all_ = param("wait_for_all", true, true);

  enable_timing_ = param("enable_timing", false, true);

  // set initial sensor pose
  sensor_pose_.header.frame_id = getMapFrame();
  sensor_pose_.header.stamp = ros::Time::now();
  sensor_pose_.data = l3::Pose();

  return true;
}

bool SensorPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  timer_ = Timer(ros::Time::now());
  last_processed_time_ = 0llu;

  return true;
}

bool SensorPlugin::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  // load filter chain
  std::vector<std::string> filter_chain = param("filter_chain", std::vector<std::string>(), true);
  filter_chain_ = boost::make_shared<ProcessChain>(getName() + "::FilterChain");
  filter_chain_->loadPlugins(filter_chain, false);

  // load processing chain
  std::vector<std::string> process_chain = param("process_chain", std::vector<std::string>(), true);
  process_chain_ = boost::make_shared<ProcessChain>(getName() + "::ProcessChain");
  process_chain_->loadPlugins(process_chain, false);

  return true;
}

void SensorPlugin::updateSensorPose(const Time& time)
{
  ros::Time ros_time = Timer::timeToRos(time);
  getTransformAsPose(tf_buffer_, getMapFrame(), getSensorFrame(), ros_time, sensor_pose_);
}

void SensorPlugin::process(const Time& time, UpdatedHandles::Ptr updates)
{
  // consider processing rate
  if (canProcess(time))
  {
    l3::UniqueLock lock(mutex_);
    last_processed_time_ = time;
    timer_.update(time);
  }
  else
    return;

  // update sensor poses
  if (canAutoUpdateSensorPose())
    updateSensorPose(time);

  ProcessingInfo::Ptr info = l3::makeShared<ProcessingInfo>(timer_, updates, nullptr, this);

  // apply filter
  filter_chain_->process(info);

  // call processes
  process_chain_->process(info, wait_for_all_);

  // print statistics
  if (enable_timing_)
  {
    // print filter chain timing
    if (filter_chain_->size() > 0)
      ROS_INFO("%s", getTotalTimingString(this->getName(), filter_chain_).c_str());

    // print process chain timing
    if (process_chain_->size() > 0)
      ROS_INFO("%s", getTotalTimingString(this->getName(), process_chain_).c_str());
  }
}
}  // namespace l3_terrain_modeling
