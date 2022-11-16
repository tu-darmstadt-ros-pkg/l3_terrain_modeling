#include <l3_terrain_model_generator/plugins/base/sensor_plugin.h>

#include <l3_libs/conversions/l3_msg_std_conversions.h>

#include <l3_terrain_model_generator/plugins/aggregator/process_chain.h>

namespace l3_terrain_modeling
{
SensorPlugin::SensorPlugin(const std::string& name)
  : Plugin(name)
  , tf_listener_(tf_buffer_)
{
  sensor_pose_.header.frame_id = "INVALID";
  sensor_pose_.header.stamp = ros::Time::now();
  sensor_pose_.data.setIdentity();
}

bool SensorPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  sensor_frame_id_ = param("sensor_frame", std::string("lidar"), true);
  map_frame_id_ = param("map_frame", std::string("map"), true);

  double rate = param("rate", 0.0, true);
  process_intervall_ = rate > 0.0 ? static_cast<uint64_t>(1000.0 / rate) : 0llu;

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

  std::string error_msg;
  if (tf_buffer_.canTransform(getMapFrame(), sensor_frame_id_, ros_time, ros::Duration(1.0), &error_msg))
  {
    l3::UniqueLock lock(mutex_);
    geometry_msgs::TransformStamped transform_msg = tf_buffer_.lookupTransform(getMapFrame(), sensor_frame_id_, ros_time);
    sensor_pose_.header.frame_id = sensor_frame_id_;
    sensor_pose_.header.stamp = ros_time;
    l3::transformMsgToL3(transform_msg.transform, sensor_pose_.data);
  }
  else
    ROS_ERROR_THROTTLE(5.0, "Could not retrieve transform '%s' to '%s':\n%s", sensor_frame_id_.c_str(), getMapFrame().c_str(), error_msg.c_str());
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
  updateSensorPose(time);

  // apply filter
  filter_chain_->process(timer_, updates, this);

  // call processes
  process_chain_->process(timer_, updates, this);
}
}  // namespace l3_terrain_modeling
