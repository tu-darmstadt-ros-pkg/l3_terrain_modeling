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

#include <pcl/point_types.h>
#include <pcl/search/octree.h>
#include <pcl/search/impl/search.hpp>

#include <l3_libs/types/types.h>

namespace l3_terrain_modeling
{
template <typename PointT>
class IncrementalOctreeSearch : public pcl::search::Octree<PointT>
{
public:
  // typedefs
  typedef l3::SharedPtr<IncrementalOctreeSearch> Ptr;
  typedef l3::SharedPtr<const IncrementalOctreeSearch> ConstPtr;

  IncrementalOctreeSearch(const double resolution)
    : pcl::search::Octree<PointT>(resolution)
  {}

  virtual ~IncrementalOctreeSearch() {}

  void clear()
  {
    this->tree_->deleteTree();
    this->input_->clear();
    this->indices_->clear();
  }

  void insertPointCloud(const typename pcl::search::Octree<PointT>::PointCloudConstPtr& cloud)
  {
    if (!cloud || cloud->empty())
      return;

    this->tree_->setInputCloud(cloud);
    this->tree_->addPointsFromInputCloud();

    this->input_ = cloud;
  }

  void insertPointCloud(const typename pcl::search::Octree<PointT>::PointCloudConstPtr& cloud, const typename pcl::search::Octree<PointT>::IndicesConstPtr& indices)
  {
    if (!indices || indices->empty())
      return;

    setInputCloud(cloud);
    this->indices_ = indices;
  }
};
}  // namespace l3_terrain_modeling
