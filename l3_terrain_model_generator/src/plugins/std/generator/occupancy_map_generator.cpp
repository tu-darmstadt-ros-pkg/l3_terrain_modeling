#include <l3_terrain_model_generator/plugins/std/generator/occupancy_map_generator.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <hector_math/iterators/eigen_iterator.h>

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

  min_height_ = param("min_height", -0.3, true);
  max_height_ = param("max_height", 0.9, true);

  binarize_ = param("binarize", true, true);

  if (getParam("binary_threshold", upper_threshold_, 100, true))
  {
    lower_threshold_ = 0;
  }
  else
  {
    // Get thresholds in meters and convert to percentage of the height range
    float upper_threshold_abs;
    if (getParam("upper_threshold", upper_threshold_abs, 0.5f, true))  // Upper threshold in meters
      upper_threshold_ = static_cast<int>(std::round((upper_threshold_abs - min_height_) / (max_height_ - min_height_) * 100.0f));
    else
      upper_threshold_ = 100; // Default to 100 if not set

    float lower_threshold_abs;
    if (getParam("lower_threshold", lower_threshold_abs, -0.1f, true))  // Lower threshold in meters
      lower_threshold_ = static_cast<int>(std::round((lower_threshold_abs - min_height_) / (max_height_ - min_height_) * 100.0f));
    else
      lower_threshold_ = 0; // Default to 0 if not set
  }

  ROS_WARN("Thresholds set to: lower = %d, upper = %d", lower_threshold_, upper_threshold_);

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
    // create debug map if required
    if (publish_debug_map_)
    {
      debug_grid_map_ = grid_map::GridMap();
      debug_grid_map_.setGeometry(grid_map.getLength(), grid_map.getResolution(), grid_map.getPosition());
      debug_grid_map_.setStartIndex(grid_map.getStartIndex());
      debug_grid_map_.setFrameId(grid_map.getFrameId());
      debug_grid_map_.setTimestamp(grid_map.getTimestamp());
      debug_grid_map_.add(layer_, grid_map.get(layer_));
      debug_grid_map_.add("debug_threshold_plane");
    }

    // Pre-compute plane coefficients
    Eigen::Vector3d t = ref_pose.translation();
    Eigen::Vector3d n = ref_pose.rotation().col(2);

    // Plane equation: z = a*x + b*y + c
    float a = -n.x() / n.z();
    float b = -n.y() / n.z();
    float c = t.z() + (n.x() / n.z()) * t.x() + (n.y() / n.z()) * t.y();

    // initialize occupancy map
    grid_map::Size size = grid_map.getSize();  // (rows, cols)
    size_t n_cells = size.prod();
    initializeOccupancyMap(occupancy_map, grid_map);

    // Access the data matrix directly instead of string lookups
    const Eigen::MatrixXf& data_layer = grid_map.get(layer_);

    // Pointers to debug layers if required
    Eigen::MatrixXf* debug_layer = nullptr;
    Eigen::MatrixXf* plane_layer = nullptr;

    if (publish_debug_map_)
    {
      debug_layer = &debug_grid_map_.get(layer_);
      plane_layer = &debug_grid_map_.get("debug_threshold_plane");
    }

    // iterate through all cells
    hector_math::iterateDenseBase(data_layer, [&](int row, int col)
      {
        const grid_map::Index index(row, col);

        // adjusted value: difference between cell value and plane
        int occ_value;
        float z = data_layer(index(0), index(1));

        if (!std::isfinite(z))
          occ_value = 0;
        else
        {
          grid_map::Position pos;
          grid_map.getPosition(index, pos);

          // plane value at this cell
          float plane_z = a * pos.x() + b * pos.y() + c;
          z -= plane_z;

          // normalize into occupancy grid value [0,100]
          if (z < min_height_ || z > max_height_)
            occ_value = 100;
          else
            occ_value = static_cast<int>(100.0f * (z - min_height_) / (max_height_ - min_height_));

          // Write debug grid map if required
          if (publish_debug_map_)
          {
            (*debug_layer)(index(0), index(1)) = z;
            (*plane_layer)(index(0), index(1)) = plane_z;
          }
        }

        // convert index to occpupancy map coordinates; need to unwrap index
        const size_t idx = grid_map::getLinearIndexFromIndex(grid_map::getIndexFromBufferIndex(index, size, grid_map.getStartIndex()), size);
        // occupancy map is stored in row-major order starting with bottom left cell
        occupancy_map.data[n_cells - idx - 1] = occ_value;
      }
    );
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

void OccupancyMapGenerator::initializeOccupancyMap(nav_msgs::OccupancyGrid& occupancy_map, const grid_map::GridMap& grid_map) const
{
  occupancy_map.header.frame_id = grid_map.getFrameId();
  occupancy_map.header.stamp.fromNSec(grid_map.getTimestamp());
  occupancy_map.info.map_load_time = occupancy_map.header.stamp;  // Same as header stamp as we do not load the map.
  occupancy_map.info.resolution = grid_map.getResolution();
  occupancy_map.info.width = grid_map.getSize()(0);
  occupancy_map.info.height = grid_map.getSize()(1);
  grid_map::Position position = grid_map.getPosition() - 0.5 * grid_map.getLength().matrix();
  occupancy_map.info.origin.position.x = position.x();
  occupancy_map.info.origin.position.y = position.y();
  occupancy_map.info.origin.position.z = 0.0;
  occupancy_map.info.origin.orientation.x = 0.0;
  occupancy_map.info.origin.orientation.y = 0.0;
  occupancy_map.info.origin.orientation.z = 0.0;
  occupancy_map.info.origin.orientation.w = 1.0;
  occupancy_map.data.clear();
  occupancy_map.data.resize(grid_map.getSize().prod());
}

void OccupancyMapGenerator::toBinaryOccupancyGrid(nav_msgs::OccupancyGrid& occupancy_map) const
{
  std::transform(occupancy_map.data.begin(), occupancy_map.data.end(), occupancy_map.data.begin(),
    [this](int8_t value) {
      // Mark as occupied if value is above the upper threshold or below the lower threshold
      return ((value > 0 && value < lower_threshold_) || value > upper_threshold_) ? 100 : 0;
    }
  );
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::OccupancyMapGenerator, l3_terrain_modeling::ProcessorPlugin)
