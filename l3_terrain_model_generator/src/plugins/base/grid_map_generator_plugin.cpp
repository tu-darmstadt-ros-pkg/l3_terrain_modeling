#include <l3_terrain_model_generator/plugins/base/grid_map_generator_plugin.h>

#include <pcl_conversions/pcl_conversions.h>

#include <l3_terrain_model_generator/utils/utils.h>
#include <l3_terrain_model_generator/utils/pcl/pcl_utils.h>

namespace l3_terrain_modeling
{
GridMapGeneratorPlugin::GridMapGeneratorPlugin(const std::string& name)
  : GeneratorPlugin(name)
  , tf_listener_(tf_buffer_)
{}

bool GridMapGeneratorPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::loadParams(params))
    return false;

  expand_map_ = param("expand_map", false, true);
  robot_centric_map_ = param("robot_centric_map", false, true);
  robot_frame_id_ = param("robot_frame", std::string("base_link"), true);
  use_color_ = param("colored", false, true);

  // check parameter consistency
  if (expand_map_ && robot_centric_map_)
  {
    ROS_ERROR("[%s] Parameters \"expand_map\" and \"robot_centric_map\" are mutually exclusive!", getName().c_str());
    return false;
  }

  // warn as expand map is not implemented yet
  if (expand_map_)
    ROS_WARN("[%s] Parameter \"expand_map\" is not implemented yet.", getName().c_str());

  return true;
}

bool GridMapGeneratorPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GeneratorPlugin::initialize(params))
    return false;

  // get grid map configuration
  bool init_map = param("init_map", false, true);

  const std::string& map_frame_id = param("map_frame", std::string("map"), true);
  std::vector<int> size = param("size", std::vector<int>{1, 1}, true);
  if (size.size() != 2)
  {
    ROS_ERROR("[%s] Initialization failed! Parameter \"size\" must be a vector of size 2!", getName().c_str());
    return false;
  }
  double resolution = param("resolution", 0.01);

  const std::string& output_data_name = getOutputDataParam(getParams(), "output_data", GRID_MAP_NAME);

  // create grid map if not already available
  if (init_map && !DataManager::hasData<grid_map::GridMap>(output_data_name))
  {
    // init grid map
    grid_map::GridMap grid_map;
    grid_map.setFrameId(map_frame_id);
    grid_map.setGeometry(grid_map::Length(size[0], size[1]), resolution);
    grid_map.setTimestamp(ros::Time::now().toNSec());

    grid_map_handle_ = DataManager::addData(this, output_data_name, std::move(grid_map));
  }
  else
  {
    grid_map_handle_ = getHandleT<grid_map::GridMap>(output_data_name);

    l3::SharedLockPtr lock;
    const grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(lock);

    if (grid_map.getFrameId() != map_frame_id)
      ROS_WARN("[%s] Provided grid_map has frame id \"%s\" but this plugin requested frame id \"%s\".", getName().c_str(), grid_map.getFrameId().c_str(), map_frame_id.c_str());
    if (grid_map.getResolution() != resolution)
      ROS_WARN("[%s] Provided grid_map has different resolution as configured by this plugin.", getName().c_str());
  }

  if (!grid_map_handle_)
  {
    ROS_ERROR("[%s] Initialization failed! No handle to grid map available!", getName().c_str());
    return false;
  }

  return true;
}

void GridMapGeneratorPlugin::reset()
{
  GeneratorPlugin::reset();

  if (grid_map_handle_)
  {
    l3::UniqueLockPtr lock;
    grid_map_handle_->value<grid_map::GridMap>(lock).clearAll();
  }
}

void GridMapGeneratorPlugin::processImpl(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor)
{
  if (!input_handle_ || !grid_map_handle_)
    return;

  // run only on changes
  if (!updates.has(input_handle_))
    return;

  // get data header
  std_msgs::Header header = getDataHeader();

  l3::UniqueLockPtr grid_map_lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(grid_map_lock);

  // check frame id
  std::string input_frame_id = l3::strip_const(header.frame_id, '/');
  if (grid_map.getFrameId() != input_frame_id)
  {
    ROS_ERROR_THROTTLE(5.0, "[%s] update: Frame of input data (\"%s\") mismatch! Should be \"%s\".", getName().c_str(), input_frame_id.c_str(),
                       grid_map.getFrameId().c_str());
    return;
  }

  // resize grid map to contain all points
  /// @todo should be done by terrain model GeneratorPlugin and not by its sub handler
  if (expand_map_)
  {
    l3::Vector3 update_min;
    l3::Vector3 update_max;
    getDataBoundary(update_min, update_max);
    resize(grid_map, update_min, update_max);
  }

  // update grid map position based on robot motion
  if (robot_centric_map_)
  {
    l3::Pose robot_pose;

    if (sensor && sensor->getSensorFrame() == robot_frame_id_)
      robot_pose = sensor->getSensorPose().data;
    else if (!getTransformAsPose(tf_buffer_, grid_map.getFrameId(), robot_frame_id_, header.stamp, robot_pose))
      return;

    grid_map.move(l3::Position2D(robot_pose.x(), robot_pose.y()));
  }

  // update grid map timestamp
  grid_map.setTimestamp(header.stamp.toNSec());

  // release mutex
  grid_map_lock.reset();

  // call grid map processing update routine implemented by derived class
  update(timer, updates, sensor);

  // add grid map to the list of updated data
  updates.insert(grid_map_handle_);
}


GridCellGridMapGeneratorPlugin::GridCellGridMapGeneratorPlugin(const std::string& name)
  : GridMapGeneratorPlugin(name)
{}

bool GridCellGridMapGeneratorPlugin::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapGeneratorPlugin::postInitialize(params))
    return false;

  GET_INPUT_HANDLE_DEFAULT(GridCellUpdates, "grid_cell_updates", input_handle_);

  return true;
}

std_msgs::Header GridCellGridMapGeneratorPlugin::getDataHeader()
{
  l3::SharedLockPtr lock;
  return input_handle_->value<GridCellUpdates>(lock).header;
}

void GridCellGridMapGeneratorPlugin::getDataBoundary(l3::Vector3& min, l3::Vector3& max)
{
  l3::SharedLockPtr lock;
  getBoundary(input_handle_->value<GridCellUpdates>(lock).cells, min, max);
}


PclGridMapGeneratorPlugin::PclGridMapGeneratorPlugin(const std::string& name)
  : GridMapGeneratorPlugin(name)
{}

bool PclGridMapGeneratorPlugin::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  if (!GridMapGeneratorPlugin::postInitialize(params))
    return false;

  GET_INPUT_PCL_HANDLE_DEFAULT("cloud", cloud_pcl_handle_);

  input_handle_ = cloud_pcl_handle_->handle();

  return true;
}

std_msgs::Header PclGridMapGeneratorPlugin::getDataHeader()
{
  std_msgs::Header header;
  cloud_pcl_handle_->dispatch<l3::SharedLock>([&](auto& cloud, auto type_trait) {
    header = pcl_conversions::fromPCL(cloud->header);
  });
  return header;
}

void PclGridMapGeneratorPlugin::getDataBoundary(l3::Vector3& min, l3::Vector3& max)
{
  cloud_pcl_handle_->dispatch<l3::SharedLock>([&](auto& cloud, auto type_trait) {
    getPointCloudBoundary<decltype(type_trait)>(cloud, min, max);
  });
}
}  // namespace l3_terrain_modeling
