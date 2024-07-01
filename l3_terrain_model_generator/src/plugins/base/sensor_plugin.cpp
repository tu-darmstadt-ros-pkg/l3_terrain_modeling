#include <l3_terrain_model_generator/plugins/base/sensor_plugin.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

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
  getTransformAsPose(tf_buffer_, getMapFrame(), sensor_frame_id_, ros_time, sensor_pose_);
}

void SensorPlugin::process(const Time& time, UpdatedHandles& updates)
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

  // apply filter
  filter_chain_->process(timer_, updates, this);

  // call processes
  process_chain_->process(timer_, updates, this);

  // print statistics
  if (enable_timing_)
  {
    // print filter chain timing
    if (filter_chain_->size() > 0)
    {
      double timing = 0.0;
      std::string timing_string;
      filter_chain_->call([&](ProcessorPlugin::Ptr process) { timing += process->getTotalTiming(); timing_string += process->getTimingString(); });
      timing_string = "\n[" + getName() + "] Filter chain chain timing:\n" + timing_string
                      + "------------------------------------------------\n"
                      + "Total: " + std::to_string(timing * 1000.0) + " ms";
      ROS_INFO("%s", timing_string.c_str());
    }

    // print process chain timing
    if (process_chain_->size() > 0)
    {
      double timing = 0.0;
      std::string timing_string;
      process_chain_->call([&](ProcessorPlugin::Ptr process) { timing += process->getTotalTiming(); timing_string += process->getTimingString(); });
      timing_string = "\n[" + getName() + "] Process chain timing:\n" + timing_string
                      + "------------------------------------------------\n"
                      + "Total: " + std::to_string(timing * 1000.0) + " ms";
      ROS_INFO("%s", timing_string.c_str());
    }
  }
}
}  // namespace l3_terrain_modeling
