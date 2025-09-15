#include <l3_terrain_model_generator/plugins/std/generator/occupancy_map_generator.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <l3_terrain_model/typedefs.h>

namespace l3_terrain_modeling
{
OccupancyMapGenerator::OccupancyMapGenerator()
  : GeneratorPlugin("occupancy_map_generator")
  , tf_listener_(tf_buffer_)
{}

bool OccupancyMapGenerator::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  layer_ = param("layer", ELEVATION_LAYER, true);

  ref_frame_id_ = param("ref_frame", std::string(), true);
  use_z_ref_frame_ = param("use_z_ref_frame", true, true);
  transform_to_ref_frame_ = param("transform_to_ref_frame", false, true);

  min_height_ = param("min_height", -0.1, true);
  max_height_ = param("max_height", 0.9, true);

  binarize_ = param("binarize", true, true);
  binary_threshold_ = param("binary_threshold", 50, true);

  publish_debug_map_ = param("publish_debug_map", false, true);

  return true;
}

bool OccupancyMapGenerator::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::initialize(params))
    return false;

  GET_OUTPUT_HANDLE_DEFAULT(nav_msgs::OccupancyGrid(), "occupancy_map", occupancy_map_handle_);

  ros::NodeHandle pnh("~");
  if (publish_debug_map_)
  {
    std::string debug_grid_map_topic = ros::names::append("debug_" + getName(), "debug_grid_map");
    debug_grid_map_pub_ = pnh.advertise<grid_map_msgs::GridMap>(debug_grid_map_topic, 1, true);
  }

  return true;
}

bool OccupancyMapGenerator::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::postInitialize(params))
    return false;

  GET_INPUT_HANDLE_DEFAULT(grid_map::GridMap, GRID_MAP_NAME, grid_map_handle_);

  return true;
}

void OccupancyMapGenerator::update(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor)
{
  l3::UniqueLockPtr occupancy_map_lock;
  nav_msgs::OccupancyGrid& occupancy_map = occupancy_map_handle_->value<nav_msgs::OccupancyGrid>(occupancy_map_lock);

  l3::SharedLockPtr grid_map_lock;
  const grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(grid_map_lock);

  // determine z reference height
  l3::Pose ref_pose; // base_link to map transform
  if (use_z_ref_frame_)
  {
    if (!ref_frame_id_.empty())
    {
      if (!getTransformAsPose(tf_buffer_, grid_map.getFrameId(), ref_frame_id_, ros::Time().fromNSec(grid_map.getTimestamp()), ref_pose))
        ROS_WARN("[%s] Failed to adjust occupancy map z position to reference frame \"%s\"!", getName().c_str(),
                ref_frame_id_.c_str());
    }
    else if (sensor)
      ref_pose = sensor->getSensorPose().data;
  }

  // check if layer exists
  if (!grid_map.exists(layer_))
  {
    ROS_WARN_THROTTLE(5.0, "[%s] Layer \"%s\" does not exist in grid map!", getName().c_str(), layer_.c_str());
    return;
  }

  // transform to reference frame if required
  if (use_z_ref_frame_ && transform_to_ref_frame_)
  {
    // create a new grid map with the same size and resolution
    grid_map::GridMap transformed_grid_map;
    transformed_grid_map.setGeometry(grid_map.getLength(), grid_map.getResolution(), grid_map.getPosition());
    transformed_grid_map.setStartIndex(grid_map.getStartIndex());
    transformed_grid_map.setFrameId(grid_map.getFrameId());
    transformed_grid_map.setTimestamp(grid_map.getTimestamp());
    transformed_grid_map.add(layer_, grid_map.get(layer_));

    // create debug map if required
    if (publish_debug_map_)
    {
      debug_grid_map_ = grid_map::GridMap();
      debug_grid_map_.setGeometry(grid_map.getLength(), grid_map.getResolution(), grid_map.getPosition());
      debug_grid_map_.setStartIndex(grid_map.getStartIndex());
      debug_grid_map_.setFrameId(grid_map.getFrameId());
      debug_grid_map_.setTimestamp(grid_map.getTimestamp());
      debug_grid_map_.add("debug_threshold_plane");
    }

    // Pre-compute plane coefficients
    Eigen::Vector3d t = ref_pose.translation();
    Eigen::Vector3d n = ref_pose.rotation().col(2);

    // Plane equation: z = a*x + b*y + c
    double a = -n.x() / n.z();
    double b = -n.y() / n.z();
    double c = t.z() + (n.x() / n.z()) * t.x() + (n.y() / n.z()) * t.y();

    // Collect valid cells
    std::vector<grid_map::Index> indices;
    std::vector<grid_map::Position> positions;
    indices.reserve(transformed_grid_map.getSize().prod());
    positions.reserve(transformed_grid_map.getSize().prod());

    for (grid_map::GridMapIterator it(transformed_grid_map); !it.isPastEnd(); ++it)
    {
      if (!transformed_grid_map.isValid(*it, layer_))
        continue;

      grid_map::Position pos;
      transformed_grid_map.getPosition(*it, pos);

      indices.push_back(*it);
      positions.push_back(pos);
    }

    // Vectorized Z computation; We avoid heap allocations (using big Eigen Matrix) for small N
    const size_t N = positions.size();
    Eigen::VectorXd Z(N);

    for (size_t i = 0; i < N; ++i)
    {
      Z(i) = a * positions[i].x() + b * positions[i].y() + c;
    }

    // Write back to grid map
    for (size_t i = 0; i < N; ++i)
    {
      transformed_grid_map.at(layer_, indices[i]) -= Z(i);

      if (publish_debug_map_)
        debug_grid_map_.at("debug_threshold_plane", indices[i]) = Z(i);
    }

    // copy layer to debug map if enabled
    if (publish_debug_map_)
      debug_grid_map_.add(layer_, transformed_grid_map.get(layer_));

    // convert to occupancy grid
    grid_map::GridMapRosConverter::toOccupancyGrid(transformed_grid_map, layer_, min_height_, max_height_, occupancy_map);
  }
  else
  {
    // convert grid map to occupancy grid
    grid_map::GridMapRosConverter::toOccupancyGrid(grid_map, layer_, ref_pose.z() + min_height_,
                                                   ref_pose.z() + max_height_, occupancy_map);
  }

  // set correct z position for occupancy map
  occupancy_map.info.origin.position.z = ref_pose.z();

  // generate binary occupancy grid
  if (binarize_)
  {
    toBinaryOccupancyGrid(occupancy_map);
  }

  // publish debug grid map if enabled
  if (publish_debug_map_ && debug_grid_map_pub_.getNumSubscribers() > 0)
  {
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(debug_grid_map_, msg);
    debug_grid_map_pub_.publish(msg);
  }
}

void OccupancyMapGenerator::toBinaryOccupancyGrid(nav_msgs::OccupancyGrid& occupancy_map) const
{
  std::transform(occupancy_map.data.begin(), occupancy_map.data.end(), occupancy_map.data.begin(),
    [this](int8_t value) {
      return value > binary_threshold_ ? 100 : 0;
    }
  );
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::OccupancyMapGenerator, l3_terrain_modeling::ProcessorPlugin)
