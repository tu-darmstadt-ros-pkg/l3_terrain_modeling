//=================================================================================================
// Copyright (c) 2022, Alexander Stumpf, TU Darmstadt
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

#include <initializer_list>

#include <ros/time.h>

#include <grid_map_core/GridMap.hpp>

#include <l3_libs/types/types.h>

#include <l3_terrain_model_generator/utils/data_handle.h>

#define INPUT_OCTREE_NAME "input_octree"
#define GRID_MAP_NAME "grid_map"

namespace l3_terrain_modeling
{
enum filterMask
{
  FILTER_NONE = 0,
  FILTER_PASS_THROUGH_BOX = 1,
  FILTER_PASS_THROUGH_ELLIPSE = 2,
  FILTER_STATISTICAL_OUTLIER = 4,
  FILTER_VOXEL_GRID = 8,
  FILTER_MLS_SMOOTH = 16,

  FILTER_ALL = 255
};

using GridMapPtr = l3::SharedPtr<grid_map::GridMap>;
using GridMapConstPtr = l3::SharedPtr<const grid_map::GridMap>;

class UpdatedHandles
{
public:
  UpdatedHandles() = default;
  UpdatedHandles(std::initializer_list<DataHandle::ConstPtr> list)
    : handles_(list)
  {}

  void insert(DataHandle::ConstPtr handle)
  {
    l3::UniqueLock lock(mutex_);
    handles_.insert(handle);
  }

  void insert(std::initializer_list<DataHandle::ConstPtr>& list)
  {
    l3::UniqueLock lock(mutex_);
    handles_.insert(list.begin(), list.end());
  }

  bool has(DataHandle::ConstPtr handle) const
  {
    l3::SharedLock lock(mutex_);
    return handles_.find(handle) != handles_.end();
  }

private:
  mutable l3::Mutex mutex_;

  std::set<DataHandle::ConstPtr> handles_;
};

using Time = uint64_t;

struct Timer
{
  Timer()
    : current(0llu)
    , last(0llu)
    , delta(0llu)
  {}

  Timer(const Time& current, const Time& last = 0llu)
    : current(current)
    , last(last)
    , delta(current - last)
  {}

  inline Timer(const ros::Time& current, const ros::Time& last = ros::Time())
    : Timer(timeFromRos(current), timeFromRos(last))
  {}

  inline Timer(const ros::TimerEvent& event)
    : Timer(timeFromRos(event.current_real), timeFromRos(event.last_real))
  {}

  void update(const Time& current)
  {
    this->last = this->current;
    this->current = current;
    this->delta = this->current - this->last;
  }

  inline void update(const ros::Time& current) { update(timeFromRos(current)); }

  Time current;  // [msec]
  Time last;     // [msec]
  Time delta;    // [msec]

  inline static Time timeFromRos(const ros::Time& time) { return (ros::Time::now().toNSec() / 1e6); }
  inline static ros::Time timeToRos(const Time& time)
  {
    ros::Time ros_time;
    ros_time.fromNSec(static_cast<uint64_t>(time * 1e6));
    return ros_time;
  }
};
}  // namespace l3_terrain_modeling
