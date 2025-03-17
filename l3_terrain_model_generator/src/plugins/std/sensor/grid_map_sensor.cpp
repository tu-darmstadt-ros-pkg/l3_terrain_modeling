#include <l3_terrain_model_generator/plugins/std/sensor/grid_map_sensor.h>

#include <l3_terrain_model/typedefs.h>

namespace l3_terrain_modeling
{
GridMapSensor::GridMapSensor() : SensorPlugin("grid_map_sensor")
{
}

bool GridMapSensor::loadParams(const vigir_generic_params::ParameterSet& params)
{
  if (!SensorPlugin::loadParams(params))
    return false;

  input_layer_ = param("input_layer", ELEVATION_LAYER, true);
  output_layer_ = param("output_layer", output_layer_, true);
  provide_grid_cell_updates_ = param("provide_grid_cell_updates", false, true);
  provide_cloud_ = param("provide_cloud", false, true);

  return true;
}

bool GridMapSensor::initialize(const vigir_generic_params::ParameterSet& params)
{
  if (!SensorPlugin::initialize(params))
    return false;

  // pre set map properties so other dependend plugins can check for it
  const std::string& map_frame_id = param("map_frame", std::string("map"), true);
  std::vector<int> size = param("size", std::vector<int>{1, 1}, true);
  if (size.size() != 2)
  {
    ROS_ERROR("[%s] Initialization failed! Parameter \"size\" must be a vector of size 2!", getName().c_str());
    return false;
  }
  double resolution = param("resolution", 0.01);

  grid_map::GridMap grid_map;
  grid_map.setFrameId(map_frame_id);
  grid_map.setGeometry(grid_map::Length(size[0], size[1]), resolution);
  grid_map.setTimestamp(ros::Time::now().toNSec());

  // init grid map handle
  const std::string& grid_map_output_data_name = getOutputDataParam(getParams(), "grid_map", GRID_MAP_NAME);
  GET_OUTPUT_HANDLE(grid_map, grid_map_output_data_name, "grid_map", grid_map_handle_);

  // init grid cell updates handle
  if (provide_grid_cell_updates_)
  {
    const std::string& grid_map_cell_updates_output_data_name = getOutputDataParam(getParams(), "grid_map_cell_updates", GRID_MAP_NAME + "_cell_updates");
    GET_OUTPUT_HANDLE(GridCellUpdates(), grid_map_cell_updates_output_data_name, "grid_cell_updates", grid_cell_updates_handle_);
  }

  // init cloud handle
  if (provide_cloud_)
  {
    const std::string& grid_map_cloud_output_data_name = getOutputDataParam(getParams(), "grid_map_cloud", GRID_MAP_NAME + "_cloud");
    GET_OUTPUT_PCL_HANDLE(grid_map_cloud_output_data_name, "cloud", "PointXYZ", cloud_pcl_handle_);
  }

  // subscribe to grid map topic
  std::string topic = param("topic", std::string("/grid_map"), true);
  grid_map_sub_ = nh_.subscribe(topic, 1, &GridMapSensor::gridMapCb, this);

  return true;
}

void GridMapSensor::gridMapCb(const grid_map_msgs::GridMap& msg)
{
  if (msg.data.empty())
    return;

  ros::Time time = msg.info.header.stamp;

  // convert msg to grid map
  l3::UniqueLockPtr lock;
  grid_map::GridMap& grid_map = grid_map_handle_->value<grid_map::GridMap>(lock);
  grid_map::GridMapRosConverter::fromMessage(msg, grid_map, { input_layer_ });
  if (input_layer_ != output_layer_)
  {
    if (grid_map.exists(output_layer_))
      ROS_WARN_THROTTLE(1.0, "[%s] Output layer \"%s\" already exists in grid map!", getName().c_str(), output_layer_.c_str());
    grid_map.add(output_layer_, grid_map[input_layer_]);
  }

  l3::SharedPtr<UpdatedHandles> updated_handles = l3::makeShared<UpdatedHandles>(std::initializer_list<DataHandle::ConstPtr>{ grid_map_handle_ });

  // convert grid map to grid cell updates
  if (provide_grid_cell_updates_)
  {
    l3::UniqueLockPtr lock;
    GridCellUpdates& grid_cell_updates = grid_cell_updates_handle_->value<GridCellUpdates>(lock);

    grid_cell_updates.header = msg.info.header;
    grid_cell_updates.cells.clear();

    for (grid_map::GridMapIterator iterator(grid_map); !iterator.isPastEnd(); ++iterator)
    {
      const grid_map::Index index(*iterator);
      grid_map::Position position;
      grid_map.getPosition(index, position);

      GridCell cell;
      cell.position.x() = position.x();
      cell.position.y() = position.y();
      cell.position.z() = grid_map.at(input_layer_, index);

      grid_cell_updates.cells.push_back(cell);
    }

    updated_handles->insert(grid_cell_updates_handle_);
  }

  // convert grid map to point cloud
  if (provide_cloud_)
  {
    sensor_msgs::PointCloud2 point_cloud_msg;
    grid_map::GridMapRosConverter::toPointCloud(grid_map, input_layer_, point_cloud_msg);
    point_cloud_msg.header.frame_id = getMapFrame();
    point_cloud_msg.header.stamp = ros::Time::now();

    // update pointcloud
    cloud_pcl_handle_->dispatch<l3::UniqueLock>(
        [&](auto& cloud, auto type_trait) { pcl::fromROSMsg(point_cloud_msg, *cloud); });

    updated_handles->insert(cloud_pcl_handle_->handle());
  }

  lock->unlock();

  // call default processing pipeline
  SensorPlugin::process(Timer::timeFromRos(time), updated_handles);
}
}  // namespace l3_terrain_modeling

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(l3_terrain_modeling::GridMapSensor, l3_terrain_modeling::SensorPlugin)
