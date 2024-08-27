#include <l3_terrain_model_generator/plugins/base/processor_plugin.h>

#include <l3_terrain_model_generator/plugins/aggregator/process_chain.h>

namespace l3_terrain_modeling
{
ProcessorPlugin::ProcessorPlugin(const std::string& name)
  : Plugin(name)
{
  exit_ = false;
  is_running_ = false;
}

ProcessorPlugin::~ProcessorPlugin()
{
  exit_ = true;

  if (run_async_)
  {
    run_cv_.notify_all();
    if (worker_thread_.joinable())
      worker_thread_.join();
  }

  is_running_ = false;
}

bool ProcessorPlugin::loadParams(const vigir_generic_params::ParameterSet& params)
{
  double rate = param("rate", 0.0, true);
  throttle_intervall_ = rate > 0.0 ? static_cast<uint64_t>(1000.0 / rate) : 0llu;

  wait_for_all_ = param("wait_for_all", true, true);
  run_async_ = param("run_async", false, true);

  enable_timing_ = param("enable_timing", false, true);

  // start worker thread if async enabled
  if (run_async_)
  {
    worker_thread_ = boost::thread(&ProcessorPlugin::processAsync, this);

    if (enable_timing_)
      ROS_WARN("[%s] Timing is enabled but not supported for async processing.", getName().c_str());
  }

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

void ProcessorPlugin::process(l3::SharedPtr<ProcessingInfo> info, ProcessChain* caller_chain)
{
  // consider processing rate
  if (canProcess(info->timer.current))
  {
    l3::UniqueLock lock(mutex_);
    last_processed_time_ = info->timer.current;
  }
  else
    return;

  if (enable_timing_)
    profiler_.start();

  // initialize runner
  boost::unique_lock<boost::mutex> lock(run_mutex_);
  is_running_ = true;
  run_info_ = info;
  lock.unlock();

  // call plugin specific implementation
  // Run the processImpl function in a separate thread
  if (run_async_)
    run_cv_.notify_one();
  // Run the processImpl function in the main thread
  else
    runProcessChain();

  if (enable_timing_)
    profiler_.stop();
}

void ProcessorPlugin::processAsync()
{
  while (!exit_)
  {
    boost::unique_lock<boost::mutex> lock(run_mutex_);
    run_cv_.wait(lock, [this]{ return is_running_ || exit_; });
    runProcessChain();
  }
}

void ProcessorPlugin::runProcessChain()
{
  if (!run_info_)
  {
    ROS_ERROR("[%s] No processing info available!", getName().c_str());
    return;
  }

  if (exit_)
    return;

  // call plugin specific implementation
  processImpl(run_info_->timer, *run_info_->updates, run_info_->sensor);

  // call subseqent process chains
  process_chain_->process(run_info_, wait_for_all_);
  is_running_ = false;

  if (exit_)
    return;

  // inform caller chain that this process is finished
  if (run_info_->caller_chain)
    run_info_->caller_chain->processFinished();

  run_info_.reset();
}
}  // namespace l3_terrain_modeling
