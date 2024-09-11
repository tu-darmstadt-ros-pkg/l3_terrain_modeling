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

#include <condition_variable>

#include <vigir_pluginlib/plugin_aggregator.h>

#include <l3_terrain_model_generator/plugins/base/processor_plugin.h>

namespace l3_terrain_modeling
{
// forward declaration
class ProcessChain;
class SensorPlugin;

struct ProcessingInfo
{
  // typedefs
  typedef l3::SharedPtr<ProcessingInfo> Ptr;
  typedef l3::SharedPtr<const ProcessingInfo> ConstPtr;

  /**
   * @param timer Timing data provided by the caller
   * @param updates Pointers of data handles whose data have been updated
   * @param caller_chain Chain that called the process (may be nullptr)
   * @param sensor Sensor on which the data is based (may be nullptr)
   */
  ProcessingInfo(const Timer& timer, UpdatedHandles::Ptr updates, ProcessChain* caller_chain = nullptr, const SensorPlugin* sensor = nullptr)
    : timer(timer), updates(updates), caller_chain(caller_chain), sensor(sensor)
  {
  }

  const Timer timer;
  UpdatedHandles::Ptr updates;
  ProcessChain* const caller_chain;
  const SensorPlugin* const sensor;
};

class ProcessChain : public vigir_pluginlib::PluginAggregator<ProcessorPlugin>
{
public:
  ProcessChain(const std::string& name = "ProcessChain");

  ~ProcessChain();

  void reset();

  /**
   * @brief Processes the given info with all plugins in the chain
   * @param info Information to be processed
   * @param wait_for_all If true, waits for all sub processes to finish before returning
   */
  void process(ProcessingInfo::Ptr info, bool wait_for_all = true);

  /**
   * @brief Called by a sub process to signal that it has finished
   */
  void processFinished();

private:
  int processes_running_;
  boost::mutex process_mutex_;
  boost::condition_variable run_cond_;

  std::atomic<bool> exit_;
};
}  // namespace l3_terrain_modeling
