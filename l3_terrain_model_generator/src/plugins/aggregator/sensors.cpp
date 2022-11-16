#include <l3_terrain_model_generator/plugins/aggregator/sensors.h>

namespace l3_terrain_modeling
{
Sensors::Sensors()
  : PluginAggregator<SensorPlugin>("Sensors")
{
}
}  // namespace l3_terrain_modeling
