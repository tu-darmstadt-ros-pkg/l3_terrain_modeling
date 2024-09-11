#include <l3_terrain_model_generator/plugins/aggregator/process_chain.h>

namespace l3_terrain_modeling
{
ProcessChain::ProcessChain(const std::string& name)
  : PluginAggregator<ProcessorPlugin>(name)
  , processes_running_(0)
{
  exit_ = false;
}

ProcessChain::~ProcessChain()
{
  exit_ = true;
  run_cond_.notify_all();
}

void ProcessChain::reset()
{
  PluginAggregator::call([](ProcessorPlugin::Ptr process) { process->reset(); });
}

void ProcessChain::process(ProcessingInfo::Ptr info, bool wait_for_all)
{
  PluginAggregator::call([&](ProcessorPlugin::Ptr process)
  {
    boost::unique_lock<boost::mutex> lock(process_mutex_);
    processes_running_++;
    lock.unlock();
    process->process(l3::makeShared<ProcessingInfo>(info->timer, info->updates, this, info->sensor));
  });

  // in case a called process is running async, wait for all to finish
  boost::unique_lock<boost::mutex> lock(process_mutex_);
  if (wait_for_all && processes_running_ > 0 && !exit_)
    run_cond_.wait(lock, [&]() { return plugins_.empty() || processes_running_ == 0 || exit_; });
}

void ProcessChain::processFinished()
{
  if (exit_)
    return;

  boost::unique_lock<boost::mutex> lock(process_mutex_);
  processes_running_--;
  if (processes_running_ == 0)
    run_cond_.notify_one();
}
}  // namespace l3_terrain_modeling
