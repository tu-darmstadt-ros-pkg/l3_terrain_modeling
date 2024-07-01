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

  enable_timing_ = param("enable_timing", false, true);

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

double ProcessorPlugin::getTotalTiming() const
{
  double timing = getTiming();
  process_chain_->call([&](ProcessorPlugin::Ptr process) { timing += process->getTotalTiming(); });
  return timing;
}

std::string ProcessorPlugin::getTimingString(unsigned int level) const
{
  std::string timing_string_out;
  if (enable_timing_)
    timing_string_out = std::string(level+1, '>') + " " + getName() + ": " + std::to_string(getTiming() * 1000.0) + " ms\n";
  else
    timing_string_out = std::string(level+1, '>') + " " + getName() + ": N/A\n";

  process_chain_->call([&](ProcessorPlugin::Ptr process) { timing_string_out += process->getTimingString(level+1); });
  return timing_string_out;
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

  if (enable_timing_)
    process_start_ = std::chrono::high_resolution_clock::now();

  // call plugin specific implementation
  processImpl(timer, updates, sensor);

  if (enable_timing_)
    process_end_ = std::chrono::high_resolution_clock::now();

  // call subseqent process chains
  process_chain_->process(timer, updates, sensor);
}
}  // namespace l3_terrain_modeling
