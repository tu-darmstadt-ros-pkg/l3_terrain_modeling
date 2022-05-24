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

#include <l3_libs/types/types.h>
#include <l3_libs/types/variant_data.h>

namespace l3_terrain_modeling
{
class DataHandle
{
public:
  // typedefs
  typedef l3::SharedPtr<DataHandle> Ptr;
  typedef l3::SharedPtr<const DataHandle> ConstPtr;

  DataHandle();

  template <class ValueType>
  DataHandle(const std::string& name, const ValueType& in)
    : name_(name)
    , data_(in)
  {}

  template <class ValueType>
  DataHandle(const std::string& name, ValueType&& in)
    : name_(name)
    , data_(std::move(in))
  {}

  inline const std::string& type() const { return data_.type(); }

  template <class ValueType>
  constexpr bool isType() const
  {
    return data_.isType<ValueType>();
  }

  template <class ValueType>
  constexpr bool isType(ValueType val) const
  {
    return data_.isType(val);
  }

  const std::string& name() const { return name_; }

  template <class ValueType>
  inline const ValueType& value(l3::SharedLockPtr& lock) const
  {
    lock = l3::makeShared<l3::SharedLock>(mutex_);
    return data_.value<ValueType>();
  }

  template <class ValueType>
  inline ValueType& value(l3::UniqueLockPtr& lock)
  {
    lock = l3::makeShared<l3::UniqueLock>(mutex_);
    return data_.value<ValueType>();
  }

private:
  mutable l3::Mutex mutex_;

  std::string name_;
  l3::VariantData data_;
};
}  // namespace l3_terrain_modeling
