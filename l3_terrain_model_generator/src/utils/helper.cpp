#include <l3_terrain_model_generator/utils/helper.h>

#include <l3_terrain_model_generator/plugins/aggregator/process_chain.h>

namespace l3_terrain_modeling
{
constexpr int width = 7;

double getTotalTimingStatistics(ProcessorPlugin::ConstPtr process, double& mean, double& min, double& max)
{
  double last = process->getProfiler().getStatistics(mean, min, max);

  // collect timing statistics of all processes
  process->getProcessChain()->call([&](ProcessorPlugin::Ptr sub_process)
  {
    double sub_mean = 0.0;
    double sub_min = 0.0;
    double sub_max = 0.0;

    last += getTotalTimingStatistics(sub_process, sub_mean, sub_min, sub_max);
    mean += sub_mean;
    min += sub_min;
    max += sub_max;
  });

  return last;
}

std::string getTimingString(ProcessorPlugin::ConstPtr process, int level)
{
  std::stringstream timing_string_out;
  timing_string_out << std::fixed << std::setprecision(2);

  double last, mean, min, max;
  last = process->getProfiler().getStatistics(mean, min, max);

  // generate timing string if last timing is finite
  if (std::isfinite(last))
  {
    std::string truncated_name = std::string(level+1, '>') + " " + process->getName();
    if (truncated_name.size() > 20)
      truncated_name = truncated_name.substr(0, 17) + "...";

    timing_string_out << truncated_name << ":\t"
                      << std::setw(width) << last * 1000.0 << "\t"
                      << std::setw(width) << mean * 1000.0 << "\t"
                      << std::setw(width) << min * 1000.0 << "\t"
                      << std::setw(width) << max * 1000.0 << "\n";
  }
  else
    timing_string_out << std::string(level+1, '>') << " " << process->getName() << ": N/A\n";

  // append timing statistics of all processes
  process->getProcessChain()->call([&](ProcessorPlugin::Ptr sub_process) { timing_string_out << getTimingString(sub_process, level+1); });

  return timing_string_out.str();
}

std::string getTotalTimingString(const std::string& process_name, l3::SharedPtr<ProcessChain> process_chain)
{
  std::stringstream timing_string_out;

  // print process chain timing
  if (process_chain->size() > 0)
  {
    double last = 0.0;
    double mean = 0.0;
    double min = 0.0;
    double max = 0.0;

    std::string timing_string;

    // collect timing statistics of all processes
    process_chain->call([&](ProcessorPlugin::Ptr sub_process)
    {
      double sub_last, sub_mean, sub_min, sub_max;
      sub_last = getTotalTimingStatistics(sub_process, sub_mean, sub_min, sub_max);
      if (std::isfinite(sub_last))
      {
        last += sub_last;
        mean += sub_mean;
        min += sub_min;
        max += sub_max;

        timing_string += getTimingString(sub_process);
      }
    });

    // compose string that contains timing statistics
    timing_string_out << std::fixed << std::setprecision(2);
    timing_string_out << "\n[" << process_name << "] Process chain timing:\n"
                      << "Process - Unit [ms]\t"
                      << std::setw(width) << "last" << "\t"
                      << std::setw(width) << "mean" << "\t"
                      << std::setw(width) << "min" << "\t"
                      << std::setw(width) << "max" << "\n"
                      << timing_string
                      << "---------------------------------------------------------\n"
                      << "Total:            \t"
                      << std::setw(width) << last * 1000.0 << "\t"
                      << std::setw(width) << mean * 1000.0 << "\t"
                      << std::setw(width) << min * 1000.0 << "\t"
                      << std::setw(width) << max * 1000.0 << "\n";
  }

  return timing_string_out.str();
}
}  // namespace l3_terrain_modeling
