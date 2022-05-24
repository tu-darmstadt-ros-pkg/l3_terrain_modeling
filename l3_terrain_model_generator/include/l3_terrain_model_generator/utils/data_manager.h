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
#include <l3_libs/singleton.h>

#include <l3_terrain_model_generator/utils/data_handle.h>

namespace l3_terrain_modeling
{
class DataManager : public l3::Singleton<DataManager>
{
  using DatabaseType = std::unordered_map<std::string, DataHandle::Ptr>;

public:
  DataManager();

  /**
   * Adds data to the data manager using copy operation and returns the created data handler.
   * If data does already exist no new data handler will be created and the exisitng will
   * be returned instead of. If the new type mismatches the exisiting type, a nullptr is
   * returned.
   * @param name Name of data
   * @param data Data to add
   * @return Corresponding data handler
   */
  template <class ValueType>
  static DataHandle::Ptr addData(const std::string& name, ValueType& data)
  {
    // check for existing handle
    DataHandle::Ptr handle = getHandle(name);

    if (handle)
    {
      if (handle->isType<ValueType>())
        return handle;
      else
      {
        ROS_ERROR("[DataManager] Element \"%s\" was added twice but with different type!", name.c_str());
        return DataHandle::Ptr();
      }
    }

    // create new handle
    handle = l3::makeShared<DataHandle>(name, ValueType(data));

    l3::UniqueLock lock(instance().mutex_);
    mutableInstance().database_[name] = handle;

    return handle;
  }

  /**
   * Adds data to the data manager using move operation and returns the created data handler.
   * If data does already exist no new data handler will be created and the exisitng will
   * be returned instead of. If the new type mismatches the exisiting type, a nullptr is
   * returned.
   * @param name Name of data
   * @param data Data to add using std::move
   * @return Corresponding data handler
   */
  template <class ValueType>
  static DataHandle::Ptr addData(const std::string& name, ValueType&& data)
  {    
    // check for existing handle
    DataHandle::Ptr handle = getHandle(name);

    if (handle)
    {
      if (handle->isType<ValueType>())
        return handle;
      else
      {
        ROS_ERROR("[DataManager] Element \"%s\" was added twice but with different type!", name.c_str());
        return DataHandle::Ptr();
      }
    }

    // create new handle
    handle = l3::makeShared<DataHandle>(name, std::move(data));

    l3::UniqueLock lock(instance().mutex_);
    mutableInstance().database_[name] = handle;

    return handle;
  }

  /**
   * @brief Checks if data with given name exists in data manager
   * @param name Name of data to check existence
   * @return true if name exists
   */
  static bool hasEntry(const std::string& name)
  {
    l3::SharedLock lock(instance().mutex_);
    return instance().database_.find(name) != instance().database_.end();
  }

  /**
   * @brief Checks if data with given name and type exists in data manager
   * @param ValueType Type of data to check existence
   * @param name Name of data to check existence
   * @return true if data exists
   */
  template <class ValueType>
  static bool hasData(const std::string& name)
  {
    DataHandle::Ptr handle = getHandle(name);
    return handle && handle->isType<ValueType>();
  }

  /**
   * @brief Retrieves data handle with given name from data manager
   * @param name Name of data to retrieve
   * @return true DataHandle if exists otherwise nullptr
   */
  static DataHandle::Ptr getHandle(const std::string& name)
  {
    l3::SharedLock lock(instance().mutex_);

    DatabaseType::const_iterator itr = instance().database_.find(name);
    if (itr != instance().database_.end())
      return itr->second;
    else
      return DataHandle::Ptr();
  }

  /**
   * @brief Retrieves data handle with given name from data manager and checks matching type.
   * @param ValueType Type of data to check existence
   * @param name Name of data to retrieve
   * @return true DataHandle if exists otherwise nullptr
   */
  template <class ValueType>
  static DataHandle::Ptr getHandle(const std::string& name)
  {
    l3::SharedLock lock(instance().mutex_);

    DatabaseType::const_iterator itr = instance().database_.find(name);
    if (itr != instance().database_.end() && itr->second->isType<ValueType>())
      return itr->second;
    else
      return DataHandle::Ptr();
  }

  /**
   * @brief Retrieves read-only data with given name and from sepcified type from data manager.
   * This method will not perform any checks and may throw an exception.
   * @param ValueType Type of data to retrieve
   * @param lock [out] SharedLock for this data
   * @return Reference to data
   */
  template <class ValueType>
  static const ValueType& getData(const std::string& name, l3::SharedLockPtr& lock)
  {
    return getHandle(name)->value<ValueType>(lock);
  }

  /**
   * @brief Retrieves writable data with given name and from sepcified type from data manager.
   * This method will not perform any checks and may throw an exception.
   * @param ValueType Type of data to retrieve
   * @param lock [out] UniqueLock for this data
   * @return Reference to data
   */
  template <class ValueType>
  static ValueType& getData(const std::string& name, l3::UniqueLockPtr& lock)
  {
    return getHandle(name)->value<ValueType>(lock);
  }

  static std::string printDatabase();

private:
  mutable l3::Mutex mutex_;

  DatabaseType database_;
};
}  // namespace l3_terrain_modeling
