//=================================================================================================
// Copyright (c) 2023, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#pragma once

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>

#include <vigir_pluginlib/plugin.h>

#include <l3_libs/types/types.h>

#include <l3_terrain_model_generator/typedefs.h>
#include <l3_terrain_model_generator/utils/data_manager.h>
#include <l3_terrain_model_generator/utils/macros.h>

namespace l3_terrain_modeling
{
// forward declaration
class ProcessChain;

/**
 * @brief Represents a generic sensor that processes
 *
 * @param sensor_frame (default: "lidar") Frame in which the sensor is localized
 * @param map_frame (default: "map") Frame in which the data should be represented
 * @param auto_update_sensor_pose (default: true) Tries to update sensor pose based on tf
 * @param rate (default: 0.0) Maximum rate at which the input data should be processed, 0.0 for no limit
 */
class SensorPlugin : public vigir_pluginlib::Plugin
{
public:
  // typedefs
  typedef l3::SharedPtr<SensorPlugin> Ptr;
  typedef l3::SharedPtr<const SensorPlugin> ConstPtr;

  SensorPlugin(const std::string& name);

  virtual ~SensorPlugin() = default;

  virtual void reset() {}

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  bool postInitialize(const vigir_generic_params::ParameterSet& params) override;

  bool isUnique() const final { return false; }

  /**
   * @brief Returns configured sensor frame, which represents the location of the sensor
   * device and thus the origin of the data.
   * @return Sensor frame id
   */
  inline const std::string& getSensorFrame() const { return sensor_frame_id_; }

  /**
   * @brief Returns configured map frame all (generated) data should be represented in.
   * @return Map frame id
   */
  inline const std::string& getMapFrame() const { return map_frame_id_; }

  /**
   * @brief Sets new sensor pose
   * @param pose New sensor pose
   */
  inline void setSensorPose(const l3::StampedPose& pose)
  {
    l3::UniqueLock lock(mutex_);
    sensor_pose_ = pose;
  }

  /**
   * @brief Gets current sensor pose
   * @return Current sensor pose
   */
  inline l3::StampedPose getSensorPose() const
  {
    l3::SharedLock lock(mutex_);
    return sensor_pose_;
  }

protected:
  /**
   * @brief Checks if new sensor data should be processed based on configured
   * processing frequency.
   * @param time Current time [msec]
   * @return true when new data should be processed
   */
  inline bool canProcess(const Time& time) const
  {
    l3::SharedLock lock(mutex_);
    return (process_intervall_ == 0llu) || ((time - last_processed_time_) >= process_intervall_);
  }

  /**
   * @brief Main processing callback that must be called by concrete sensor implementation
   * to trigger processing of new data.
   * Calls also updateSensorPose(...).
   * @param time Current time [msec]
   * @param updates Pointers of data handles whose data have been updated
   */
  void process(const Time& time, UpdatedHandles::Ptr updates);

  /**
   * @brief Returns state if auto sensor update should be used
   * @return true if auto sensor update should be used
   */
  bool canAutoUpdateSensorPose() const { return auto_update_sensor_pose_; }

  /**
   * @brief Overwritable method how to obtain the sensor pose. The default implementation
   * relies on tf frames using the given sensor frame id and map frame id.
   * This method is automatically called by process(...).
   * @param time Current time [msec]
   */
  virtual void updateSensorPose(const Time& time);

  /**
   * Helper to retrieve easily data handles from the date manager while performing
   * type checking.
   * @param ValueType Type of data the handle stores
   * @param name Name of data entity within the data manager
   * @return true if handler for given type was found
   */
  template <class ValueType>
  DataHandle::Ptr getHandleT(const std::string& name) const
  {
    DataHandle::Ptr handle = DataManager::getHandle<ValueType>(this, name);

    if (!handle)
      ROS_ERROR("[%s] Could not fetch \"%s\" data of type \"%s\"!", getName().c_str(), name.c_str(), l3::getTypeName<ValueType>().c_str());

    return handle;
  }

  /**
   * Helper to retrieve easily data handles from the date manager.
   * @param name Name of data entity within the data manager
   * @return true if handler for given type was found
   */
  DataHandle::Ptr getHandle(const std::string& name) const
  {
    DataHandle::Ptr handle = DataManager::getHandle(this, name);

    if (!handle)
      ROS_ERROR("[%s] Could not fetch \"%s\" data!", getName().c_str(), name.c_str());

    return handle;
  }

protected:
  mutable l3::Mutex mutex_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

private:
  // subsequent process calls
  l3::SharedPtr<ProcessChain> filter_chain_;
  l3::SharedPtr<ProcessChain> process_chain_;

  // sensor pose
  l3::StampedPose sensor_pose_;

  // parameters
  std::string sensor_frame_id_;
  std::string map_frame_id_;
  bool auto_update_sensor_pose_;

  // variables used for throtteling
  Timer timer_;
  uint64_t process_intervall_;            // in [msec]
  mutable uint64_t last_processed_time_;  // in [msec]

  // variables for multi-threading
  bool wait_for_all_;

  bool enable_timing_;
};
}  // namespace l3_terrain_modeling
