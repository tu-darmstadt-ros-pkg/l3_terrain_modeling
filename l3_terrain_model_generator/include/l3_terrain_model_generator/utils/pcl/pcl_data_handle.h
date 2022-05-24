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

#include <pcl/pcl_config.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <l3_terrain_model_generator/utils/data_handle.h>
#include <l3_terrain_model_generator/utils/data_manager.h>

// check PCL version
#if PCL_VERSION_COMPARE(>=, 1, 10, 0)
#define POINTXYZI_SUPPORT
#endif

namespace l3_terrain_modeling
{
enum PointType
{
  Invalid = -1,
  PointXYZ,
#ifdef POINTXYZI_SUPPORT
  PointXYZI,
#endif
  PointXYZRGB,
  PointNormal
};

PointType dispatchPointType(const std::string& point_type);

inline bool isValidPointType(const std::string& point_type) { return dispatchPointType(point_type) != PointType::Invalid; }

std::string toString(const PointType& point_type);

template <template <typename> class PclType>
class PclDataHandle
{
public:
  // typedefs
  typedef l3::SharedPtr<PclDataHandle> Ptr;
  typedef l3::SharedPtr<const PclDataHandle> ConstPtr;

  /**
   * @brief Creates PclDataHandle using given data handle. The point type is automatically
   * dispatched.
   * @param handle Data handle to wrap
   */
  PclDataHandle(DataHandle::Ptr handle)
  {
    if (handle)
    {
      PointType point_type = resolvePointType(handle);
      if (point_type != PointType::Invalid)
      {
        handle_ = handle;
        point_type_ = point_type;
      }
    }
    else
      point_type_ = PointType::Invalid;
  }

  /**
   * @brief Creates PclDataHandle and either creates new data handle or retrieves the corresponding
   * exisiting handle from the data manager.
   * @param name Name of internally used data handler
   * @param point_type Point type (see PointType enum)
   */
  template <class... Args>
  PclDataHandle(const std::string& name, const PointType& point_type, Args&&... args)
    : handle_(createDataHandle(name, point_type, boost::forward<Args>(args)...))
    , point_type_(point_type)
  {}

  /**
   * @brief Creates PclDataHandle and either creates new data handle or retrieves the corresponding
   * exisiting handle from the data manager.
   * @param name Name of internally used data handler
   * @param point_type Point type as string (see PointType enum)
   */
  template <class... Args>
  PclDataHandle(const std::string& name, const std::string& point_type, Args&&... args)
    : PclDataHandle(name, dispatchPointType(point_type), boost::forward<Args>(args)...)
  {}

  /**
   * @brief Creates shared pointer using given data handle. The point type is automatically
   * dispatched.
   * @param handle Data handle to wrap
   * @return New PclDataHandle shared pointer if handle is compatible. Otherwise nullptr.
   */
  static PclDataHandle::Ptr makeHandle(DataHandle::Ptr handle)
  {
    PclDataHandle::Ptr pcl_handle = boost::make_shared<PclDataHandle<PclType>>(handle);
    if (pcl_handle->handle())
      return pcl_handle;
    else
      return PclDataHandle::Ptr();
  }

  /**
   * @brief Creates shared pointer of PclDataHandle and retrieves the corresponding exisiting handle
   * from the data manager.
   * @param name Name of internally used data handler
   * @return New PclDataHandle shared pointer if handle is successfully created. Otherwise nullptr.
   */
  static PclDataHandle::Ptr makeHandle(const std::string& name)
  {
    DataHandle::Ptr handle = DataManager::getHandle(name);
    if (handle)
    {
      PclDataHandle::Ptr pcl_handle = makeHandle(handle);
      if (pcl_handle->handle())
        return pcl_handle;
    }
    return PclDataHandle::Ptr();
  }

  /**
   * @brief Creates shared pointer of PclDataHandle and either creates new data handle or retrieves
   * the corresponding exisiting handle from the data manager.
   * @param name Name of internally used data handler
   * @param point_type Point type (see PointType enum)
   * @return New PclDataHandle shared pointer if handle is successfully created. Otherwise nullptr.
   */
  template <class... Args>
  static PclDataHandle::Ptr makeHandle(const std::string& name, const PointType& point_type, Args&&... args)
  {
    PclDataHandle::Ptr pcl_handle = boost::make_shared<PclDataHandle<PclType>>(name, point_type, boost::forward<Args>(args)...);
    if (pcl_handle->handle())
      return pcl_handle;
    else
      return PclDataHandle::Ptr();
  }

  /**
   * @brief Creates shared pointer of PclDataHandle and either creates new data handle or retrieves
   * the corresponding exisiting handle from the data manager.
   * @param name Name of internally used data handler
   * @param point_type Point type as string (see PointType enum)
   * @return New PclDataHandle shared pointer if handle is successfully created. Otherwise nullptr.
   */
  template <class... Args>
  inline static PclDataHandle::Ptr makeHandle(const std::string& name, const std::string& point_type, Args&&... args)
  {
    return makeHandle(name, dispatchPointType(point_type), boost::forward<Args>(args)...);
  }

  /**
   * @brief Returns internally stored data handle.
   * @return Internally stored data handle.
   */
  inline DataHandle::Ptr handle() { return handle_; }

  /**
   * @brief Returns data type stored by the internal data handle.
   * @return Data type stored by the internal data handle as string
   */
  inline const std::string& type() const { return handle_->type(); }

  /**
   * Compares if the internal data handle object and the given input object are
   * the same type.
   * @param PointT Point type used by val
   * @param val Object to perform type checking against
   * @return true if types are equal
   */
  template <class PointT>
  constexpr bool isType(typename boost::shared_ptr<PclType<PointT>> val) const
  {
    return handle_->isType(val);
  }

  /**
   * @brief Returns the internally used point type.
   * @return Internally used point type
   */
  const PointType& pointType() const { return point_type_; }

  /**
   * Compares if the internal data handle object uses a specific point type.
   * @param PointT Point type to check against
   * @return true if point types are equal
   */
  template <class PointT>
  constexpr bool isPointType() const
  {
    return handle_->isType<typename boost::shared_ptr<PclType<PointT>>>();
  }

  /**
   * @brief Name of internally stored data handle.
   * @return Name of internally stored data handle.
   */
  const std::string& name() const { return handle_->name(); }

  /**
   * @brief Returns const reference to the internal object pointer hold by the data handle.
   * @param lock [out] Shared lock granting safe read access in concurrency environments.
   * @return Reference to internal object pointer
   */
  template <class PointT>
  inline const typename boost::shared_ptr<PclType<PointT>>& value(l3::SharedLockPtr& lock) const
  {
    return handle_->value<typename boost::shared_ptr<PclType<PointT>>>(lock);
  }

  /**
   * @brief Returns non-const reference to the internal object pointer hold by the data handle.
   * @param lock [out] Unique lock granting safe write access in concurrency environments.
   * @return Reference to internal object pointer
   */
  template <class PointT>
  inline typename boost::shared_ptr<PclType<PointT>>& value(l3::UniqueLockPtr& lock)
  {
    return handle_->value<typename boost::shared_ptr<PclType<PointT>>>(lock);
  }

  /**
   * @brief Lambda function caller to generically call a specific member function
   * of the internally stored pcl-based object without manually deducing the internal
   * point type.
   * Calling is thread-safe due to auto-locking of the data's mutex based on given lock type.
   * Example: handle.call<l3::SharedLock>([](auto& pcl, auto type_trait) { pcl->trigger(); })
   * @param Lock Lock type to use, can be either l3::SharedLock or l3::UniqueLock
   * @param fun Functor to call
   */
  template <class Lock, class Function>
  void dispatch(Function fun) const
  {
    if (handle_)
    {
      boost::shared_ptr<Lock> lock;
      switch (pointType())
      {
        case PointType::PointXYZ:
          fun(handle_->value<boost::shared_ptr<PclType<pcl::PointXYZ>>>(lock), pcl::PointXYZ());
          break;
#ifdef POINTXYZI_SUPPORT
        case PointType::PointXYZI:
          fun(handle_->value<boost::shared_ptr<PclType<pcl::PointXYZI>>>(lock), pcl::PointXYZI());
          break;
#endif
        case PointType::PointXYZRGB:
          fun(handle_->value<boost::shared_ptr<PclType<pcl::PointXYZRGB>>>(lock), pcl::PointXYZRGB());
          break;
        case PointType::PointNormal:
          fun(handle_->value<boost::shared_ptr<PclType<pcl::PointNormal>>>(lock), pcl::PointNormal());
          break;
        default:
          ROS_WARN("Invalid dispatching attempt of data \"%s\" (\"%s\")!", name().c_str(), type().c_str());
          break;
      }
    }
  }

protected:
  /**
   * @brief Creates new data handle according to the given template parameters.
   * @param name Name of new data handle
   * @param point_type Point type to use as template argument for PclType.
   * @return New data handle pointer on success, otherwise nullptr.
   */
  template <class... Args>
  static DataHandle::Ptr createDataHandle(const std::string& name, const PointType& point_type, Args&&... args)
  {
    switch (point_type)
    {
      case PointType::PointXYZ:
        return DataManager::addData(name, boost::make_shared<PclType<pcl::PointXYZ>>(boost::forward<Args>(args)...));
#ifdef POINTXYZI_SUPPORT
      case PointType::PointXYZI:
        return DataManager::addData(name, boost::make_shared<PclType<pcl::PointXYZI>>(boost::forward<Args>(args)...));
#endif
      case PointType::PointXYZRGB:
        return DataManager::addData(name, boost::make_shared<PclType<pcl::PointXYZRGB>>(boost::forward<Args>(args)...));
      case PointType::PointNormal:
        return DataManager::addData(name, boost::make_shared<PclType<pcl::PointNormal>>(boost::forward<Args>(args)...));
      default:
        return DataHandle::Ptr();
    }
  }

  /**
   * @brief Tries to resolves point type used by the object stored in the data handle.
   * It is assumed that the template class equals to PclType.
   * @param handle Data handle whose point type should be resolved.
   * @return Resolved point type (see PointType enum)
   */
  static PointType resolvePointType(DataHandle::Ptr handle)
  {
    if (handle->isType<typename PclType<pcl::PointXYZ>::Ptr>())
      return PointType::PointXYZ;
#ifdef POINTXYZI_SUPPORT
    else if (handle->isType<typename PclType<pcl::PointXYZI>::Ptr>())
      return PointType::PointXYZI;
#endif
    else if (handle->isType<typename PclType<pcl::PointXYZRGB>::Ptr>())
      return PointType::PointXYZRGB;
    else if (handle->isType<typename PclType<pcl::PointNormal>::Ptr>())
      return PointType::PointNormal;
    else
      return PointType::Invalid;
  }

private:
  DataHandle::Ptr handle_;
  PointType point_type_;
};
}  // namespace l3_terrain_modeling
