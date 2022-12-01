#include <l3_terrain_model_generator/plugins/base/processor_plugin.h>

#include <l3_terrain_model_generator/plugins/aggregator/process_chain.h>

namespace l3_terrain_modeling
{
ProcessorPlugin::ProcessorPlugin(const std::string& name)
  : Plugin(name)
{}

bool ProcessorPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  double rate = param("rate", 0.0, true);
  throttle_intervall_ = rate > 0.0 ? static_cast<uint64_t>(1000.0 / rate) : 0llu;
  return true;
}

bool ProcessorPlugin::initialize(const vigir_generic_params::ParameterSet& params)
{
  last_processed_time_ = 0llu;
  return true;
}

bool ProcessorPlugin::postInitialize(const vigir_generic_params::ParameterSet& params)
{
  // load processing chain
  std::vector<std::string> process_chain = param("process_chain", std::vector<std::string>(), true);
  process_chain_ = boost::make_shared<ProcessChain>(getName() + "::ProcessChain");
  process_chain_->loadPlugins(process_chain, false);

  return true;
}

void ProcessorPlugin::process(const Timer& timer, UpdatedHandles& updates, const SensorPlugin* sensor)
{
  // consider processing rate
  if (canProcess(timer.current))
  {
    l3::UniqueLock lock(mutex_);
    last_processed_time_ = timer.current;
  }
  else
    return;

  // call plugin specific implementation
  processImpl(timer, updates, sensor);

  // call subseqent process chains
  process_chain_->process(timer, updates, sensor);
}
}  // namespace l3_terrain_modeling
