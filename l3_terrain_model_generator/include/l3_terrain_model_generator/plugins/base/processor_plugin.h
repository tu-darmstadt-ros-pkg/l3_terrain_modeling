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

#include <vigir_pluginlib/plugin.h>

#include <l3_libs/types/types.h>

#include <l3_terrain_model_generator/typedefs.h>
#include <l3_terrain_model_generator/plugins/base/sensor_plugin.h>
#include <l3_terrain_model_generator/utils/data_manager.h>

namespace l3_terrain_modeling
{
// forward declaration
class ProcessChain;

class ProcessorPlugin : public vigir_pluginlib::Plugin
{
public:
  // typedefs
  typedef l3::SharedPtr<ProcessorPlugin> Ptr;
  typedef l3::SharedPtr<const ProcessorPlugin> ConstPtr;

  ProcessorPlugin(const std::string& name);

  virtual ~ProcessorPlugin() = default;

  virtual void reset() {}

  bool loadParams(const vigir_generic_params::ParameterSet& params) override;

  bool initialize(const vigir_generic_params::ParameterSet& params) override;

  bool postInitialize(const vigir_generic_params::ParameterSet& params) override;

  bool isUnique() const final { return false; }

  /**
   * @brief Triggers processing of this plugin. Afterward subsequent processes are called.
   * Processing is only performed when a pre-configured time is elapsed.
   * @param timer Timing data provided by the caller
   * @param updates Pointers of data handles whose data have been updated
   * @param sensor Sensor on which the data is based (may be nullptr)
   */
  void process(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor = nullptr);

protected:
  /**
   * @brief Method stub for concrete implementation of this process plugin.
   * This method is automatically called by process(...).
   * @param timer Timing data provided by the caller
   * @param updates Pointers of data handles whose data have been updated
   * @param sensor Sensor on which the data is based (may be nullptr)
   */
  virtual void processImpl(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor) {}

  /**
   * @brief Checks if new data should be processed based on configured
   * processing frequency. This method can be overwritten to alter the default
   * behavior.
   * @param time Current time [msec]
   * @return true when new data should be processed
   */
  virtual bool canProcess(const Time& time) const
  {
    l3::SharedLock lock(mutex_);
    return (throttle_intervall_ == 0llu) || ((time - last_processed_time_) >= throttle_intervall_);
  }

  /**
   * Helper to retrieve easily data handles from the date manager while performing
   * type checking.
   * @param ValueType Type of data the handle stores
   * @param name Name of data entity within the data manager
   * @return true if hander for given type was found
   */
  template <class ValueType>
  DataHandle::Ptr getHandleT(const std::string& name) const
  {
    DataHandle::Ptr handle = DataManager::getHandle<ValueType>(name);

    if (!handle)
      ROS_ERROR("[%s] Could not fetch \"%s\" data of type \"%s\"!", getName().c_str(), name.c_str(), l3::getTypeName<ValueType>().c_str());

    return handle;
  }

  /**
   * Helper to retrieve easily data handles from the data manager.
   * @param name Name of data entity within the data manager
   * @return true if hander for given type was found
   */
  DataHandle::Ptr getHandle(const std::string& name) const
  {
    DataHandle::Ptr handle = DataManager::getHandle(name);

    if (!handle)
      ROS_ERROR("[%s] Could not fetch \"%s\" data!", getName().c_str(), name.c_str());

    return handle;
  }

protected:
  mutable l3::Mutex mutex_;

private:
  l3::SharedPtr<ProcessChain> process_chain_;

  uint64_t throttle_intervall_;           // in [msec]
  mutable uint64_t last_processed_time_;  // in [msec]
};
}  // namespace l3_terrain_modeling
