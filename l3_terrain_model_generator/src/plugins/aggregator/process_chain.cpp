#include <l3_terrain_model_generator/plugins/aggregator/process_chain.h>

namespace l3_terrain_modeling
{
ProcessChain::ProcessChain(const std::string& name)
  : PluginAggregator<ProcessorPlugin>(name)
{
}
}  // namespace l3_terrain_modeling
