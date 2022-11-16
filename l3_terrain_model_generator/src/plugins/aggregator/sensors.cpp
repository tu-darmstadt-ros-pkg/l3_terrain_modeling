#include <l3_terrain_model_generator/plugins/aggregator/sensors.h>

namespace l3_terrain_modeling
{
Sensors::Sensors(const std::string& name)
  : PluginAggregator<SensorPlugin>(name)
{
}
}  // namespace l3_terrain_modeling
