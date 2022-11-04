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

#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>

#include <grid_map_core/GridMap.hpp>

#include <l3_libs/types/typedefs.h>

#include <l3_math/math.h>

#include <l3_terrain_model_generator/utils/pcl/pcl_utils.h>

namespace l3_terrain_modeling
{
/**
 * @brief Function caller to filter a point cloud. The resulting
 * point cloud does not contain any invalid points (inf or nans values).
 * @param cloud to filter
 * @param filter Functor to filter; Must return true when point should be kept
 * @return filtered cloud
 */
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterDense(typename pcl::PointCloud<PointT>::ConstPtr cloud, std::function<bool(const l3::Point&)> filter)
{
  if (!cloud)
  {
    ROS_ERROR("[filterDense] Nullptr to point cloud was given! Fix it immediatly!");
    return boost::make_shared<typename pcl::PointCloud<PointT>>();
  }

  std::vector<int> indices;
  indices.reserve(cloud->size());

  for (int i = 0; i < cloud->size(); i++)
  {
    const PointT& p = cloud->at(i);

    if (filter(l3::Point(p.x, p.y, p.z)))
      indices.push_back(i);
  }

  return boost::make_shared<pcl::PointCloud<PointT>>(*cloud, indices);
}

/**
 * @brief Function caller to filter a point cloud. The resulting
 * point cloud may contain invalid points (inf or nans values) and
 * preserves the organization. This method is usually more efficient
 * than filterDense as it does not perform a deep copy.
 * @param cloud Input and output variable of the cloud to filter
 * @param filter Functor to filter; Must return true when point should be kept
 * @return filtered cloud
 */
template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterOrganized(typename pcl::PointCloud<PointT>::Ptr cloud, std::function<bool(const l3::Point&)> filter)
{
  if (!cloud)
  {
    ROS_ERROR("[filterOrganized] Nullptr to point cloud was given! Fix it immediatly!");
    return boost::make_shared<typename pcl::PointCloud<PointT>>();
  }

  // generate point with nan fields
  PointT nan_point = createNanPoint<PointT>();

  for (PointT& p : *cloud)
  {
    if (!filter(l3::Point(p.x, p.y, p.z)))
      p = nan_point;
  }

  cloud->is_dense = false;

  return cloud;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterPassThroughBox(typename pcl::PointCloud<PointT>::ConstPtr cloud, const std::string& field_name, double min, double max)
{
  if (!cloud)
  {
    ROS_ERROR("[filterPassThroughBox] No point cloud is available!");
    return boost::make_shared<typename pcl::PointCloud<PointT>>();
  }

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  pcl::PassThrough<PointT> pass;
  pass.setKeepOrganized(cloud->isOrganized() || !cloud->is_dense);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName(field_name);
  pass.setFilterLimits(min, max);
  pass.filter(*cloud_filtered);

  return cloud_filtered;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterPassThroughEllipse(typename pcl::PointCloud<PointT>::Ptr cloud, const l3::Pose& center, double rx, double ry)
{
  if (!cloud)
  {
    ROS_ERROR("[filterPassThroughEllipse] No point cloud is available!");
    return boost::make_shared<typename pcl::PointCloud<PointT>>();
  }

  // setup filter
  l3::Point size(rx, ry, 0.0);
  double cos_yaw = cos(center.yaw());
  double sin_yaw = sin(center.yaw());
  auto filter = [&](const l3::Point& p) -> bool { return l3::isPointInEllipse(p, center.getPosition(), size, cos_yaw, sin_yaw); };

  // filter
  if (cloud->isOrganized() || !cloud->is_dense)
    return filterOrganized<PointT>(cloud, filter);
  else
    return filterDense<PointT>(cloud, filter);
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterVoxelGrid(typename pcl::PointCloud<PointT>::ConstPtr cloud, float lx, float ly, float lz)
{
  if (!cloud)
  {
    ROS_ERROR("[filterVoxelGrid] No point cloud is available!");
    return boost::make_shared<typename pcl::PointCloud<PointT>>();
  }

  if (cloud->isOrganized())
    ROS_WARN_ONCE("[filterVoxelGrid] Cloud is not orgranized after downsampling. This warning is printed only once.");

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  pcl::VoxelGrid<PointT> vox;
  vox.setInputCloud(cloud);
  vox.setLeafSize(lx, ly, lz);
  vox.filter(*cloud_filtered);

  return cloud_filtered;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterMlsSmooth(typename pcl::PointCloud<PointT>::ConstPtr cloud, double radius)
{
  if (!cloud)
  {
    ROS_ERROR("[filterMlsSmooth] No point cloud is available!");
    return boost::make_shared<typename pcl::PointCloud<PointT>>();
  }

  if (cloud->isOrganized())
    ROS_WARN_ONCE("[filterMlsSmooth] Cloud is not orgranized after smoothing. This warning is printed only once.");

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
  pcl::MovingLeastSquares<PointT, PointT> mls;
  mls.setInputCloud(cloud);
  mls.setSearchMethod(tree);
  // mls.setComputeNormals(false);
  // mls.setPolynomialFit(false);
  mls.setPolynomialOrder(1);
  mls.setSearchRadius(radius);
  //  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
  //  mls.setUpsamplingRadius(0.025);
  //  mls.setUpsamplingStepSize(0.01);
  //  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::RANDOM_UNIFORM_DENSITY);
  //  mls.setPointDensity(radius*5000);
  //  mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::VOXEL_GRID_DILATION);
  //  mls.setDilationVoxelSize(0.01);
  //  mls.setDilationIterations(1);
  mls.process(*cloud_filtered);

  return cloud_filtered;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterStatisticalOutlier(typename pcl::PointCloud<PointT>::ConstPtr cloud, int k, double radius, bool set_negative = false)
{
  if (!cloud)
  {
    ROS_ERROR("[filterStatisticalOutlier] No point cloud is available!");
    return boost::make_shared<typename pcl::PointCloud<PointT>>();
  }

  if (cloud->isOrganized())
    ROS_WARN_ONCE("[filterStatisticalOutlier] Cloud is not orgranized after outlier removal. This warning is printed only once.");

  typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>());
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud(cloud);
  sor.setNegative(set_negative);
  sor.setMeanK(k);
  sor.setStddevMulThresh(radius);
  sor.filter(*cloud_filtered);

  return cloud_filtered;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr filterInGridMap(typename pcl::PointCloud<PointT>::Ptr cloud, const grid_map::GridMap& map)
{
  if (!cloud)
  {
    ROS_ERROR("[filterInGridMap] No point cloud is available!");
    return boost::make_shared<typename pcl::PointCloud<PointT>>();
  }

  if (cloud->header.frame_id != map.getFrameId())
  {
    ROS_ERROR("[filterInGridMap] Cloud (%s) and grid map (%s) frame id are different!", cloud->header.frame_id.c_str(), map.getFrameId().c_str());
    return boost::make_shared<typename pcl::PointCloud<PointT>>(*cloud);
  }

  // setup filter
  auto filter = [&](const l3::Point& p) -> bool { return map.isInside(grid_map::Position(p.x(), p.y())); };

  // filter
  if (cloud->isOrganized() || !cloud->is_dense)
    return filterOrganized<PointT>(cloud, filter);
  else
    return filterDense<PointT>(cloud, filter);
}
}  // namespace l3_terrain_modeling
