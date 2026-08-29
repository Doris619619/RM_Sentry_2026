#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include "ocs2_sentry/SentryRobotInterface.h"

int main() {
  SentryRobotInterface robot;
  robot.init(ament_index_cpp::get_package_share_directory("trajectory_tracking") + "/cfg/task.info");
  robot.sqpSettings().hpipmSettings.hpipmMode = hpipm_mode::ROBUST;
  robot.sqpSettings().hpipmSettings.reg_prim = 1e-8;
  ocs2::SqpMpc solver(robot.mpcSettings(), robot.sqpSettings(), robot.getOptimalControlProblem(), robot.getInitializer());
  solver.getSolverPtr()->setReferenceManager(robot.getReferenceManagerPtr());
  ocs2::scalar_array_t times; ocs2::vector_array_t states, inputs;
  for (int i = 0; i < 20; ++i) {
    times.push_back(0.1 * i);
    states.emplace_back((ocs2::vector_t(4) << 0.1 * i, 0.0, 1.0, 0.0).finished());
    inputs.emplace_back(ocs2::vector_t::Zero(2));
  }
  robot.getReferenceManagerPtr()->setTargetTrajectories(ocs2::TargetTrajectories(times, states, inputs));
  const ocs2::vector_t observation = (ocs2::vector_t(4) << 0.0, 0.0, 0.0, 0.0).finished();
  for (int i = 0; i < 50; ++i) if (!solver.run(0.0, observation)) return 2;
  std::vector<double> samples; samples.reserve(1000);
  const auto cpuStart = std::clock();
  for (int i = 0; i < 1000; ++i) {
    const auto start = std::chrono::steady_clock::now();
    if (!solver.run(0.0, observation)) return 3;
    samples.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
  }
  const auto cpuSeconds = double(std::clock() - cpuStart) / CLOCKS_PER_SEC;
  std::sort(samples.begin(), samples.end());
  double sum = 0.0; for (double value : samples) sum += value;
  const auto p95 = samples[static_cast<size_t>(0.95 * (samples.size() - 1))];
  const auto over10 = std::count_if(samples.begin(), samples.end(), [](double value) { return value > 10.0; });
  std::cout << "mean_ms=" << sum / samples.size() << " p95_ms=" << p95
            << " max_ms=" << samples.back() << " over_10ms=" << over10
            << " cpu_s=" << cpuSeconds << std::endl;
  return 0;
}
