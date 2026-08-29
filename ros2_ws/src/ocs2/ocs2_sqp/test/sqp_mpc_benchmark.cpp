#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <vector>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
namespace {
std::unique_ptr<ocs2::SqpMpc> makeMpc() {
  ocs2::matrix_t A = ocs2::matrix_t::Zero(1, 1), B = ocs2::matrix_t::Identity(1, 1);
  ocs2::OptimalControlProblem problem;
  problem.dynamicsPtr = std::make_unique<ocs2::LinearSystemDynamics>(A, B);
  const ocs2::matrix_t Q = 4.0 * ocs2::matrix_t::Identity(1, 1), R = 0.2 * ocs2::matrix_t::Identity(1, 1), P = ocs2::matrix_t::Zero(1, 1), Qf = 30.0 * ocs2::matrix_t::Identity(1, 1);
  problem.costPtr->add("running_cost", std::make_unique<ocs2::QuadraticStateInputCost>(Q, R, P));
  problem.finalCostPtr->add("terminal_cost", std::make_unique<ocs2::QuadraticStateCost>(Qf));
  const ocs2::vector_t targetState = ocs2::vector_t::Zero(1), targetInput = ocs2::vector_t::Zero(1);
  auto referenceManager = std::make_shared<ocs2::ReferenceManager>(ocs2::TargetTrajectories({0.0}, {targetState}, {targetInput}));
  problem.targetTrajectoriesPtr = &referenceManager->getTargetTrajectories();
  ocs2::DefaultInitializer initializer(1);
  ocs2::mpc::Settings mpcSettings; mpcSettings.timeHorizon_ = 1.0;
  ocs2::sqp::Settings sqpSettings; sqpSettings.dt = 0.1; sqpSettings.sqpIteration = 8; sqpSettings.nThreads = 2; sqpSettings.enableLogging = false;
  sqpSettings.hpipmSettings.hpipmMode = hpipm_mode::ROBUST; sqpSettings.hpipmSettings.reg_prim = 1e-8;
  auto mpc = std::make_unique<ocs2::SqpMpc>(mpcSettings, sqpSettings, problem, initializer); mpc->getSolverPtr()->setReferenceManager(referenceManager); return mpc;
}
void runOrThrow(ocs2::SqpMpc& mpc, const ocs2::vector_t& initialState) {
  if (!mpc.run(0.0, initialState)) throw std::runtime_error("SqpMpc::run() returned false");
  const auto solution = mpc.getSolverPtr()->primalSolution(1.0);
  if (solution.timeTrajectory_.empty() || solution.timeTrajectory_.size() != solution.stateTrajectory_.size() || solution.timeTrajectory_.size() != solution.inputTrajectory_.size()) throw std::runtime_error("invalid primal solution");
  for (std::size_t i = 0; i < solution.timeTrajectory_.size(); ++i) if (!std::isfinite(solution.timeTrajectory_[i]) || !solution.stateTrajectory_[i].array().isFinite().all() || !solution.inputTrajectory_[i].array().isFinite().all()) throw std::runtime_error("non-finite primal solution");
}
}
int main() {
  constexpr int kWarmupIterations = 50, kMeasuredIterations = 1000;
  auto mpc = makeMpc(); ocs2::vector_t initialState(1); initialState << 1.0;
  for (int i = 0; i < kWarmupIterations; ++i) runOrThrow(*mpc, initialState);
  std::vector<double> samplesMs; samplesMs.reserve(kMeasuredIterations); const std::clock_t cpuStart = std::clock();
  for (int i = 0; i < kMeasuredIterations; ++i) { const auto start = std::chrono::steady_clock::now(); runOrThrow(*mpc, initialState); samplesMs.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count()); }
  const double cpuSeconds = static_cast<double>(std::clock() - cpuStart) / CLOCKS_PER_SEC, sum = std::accumulate(samplesMs.begin(), samplesMs.end(), 0.0);
  std::vector<double> ordered = samplesMs; std::sort(ordered.begin(), ordered.end()); const std::size_t p95Index = static_cast<std::size_t>(std::ceil(0.95 * ordered.size())) - 1; const auto over10ms = std::count_if(samplesMs.begin(), samplesMs.end(), [](double value) { return value > 10.0; });
  std::cout << "warmup_iterations=" << kWarmupIterations << '\n' << "measured_iterations=" << kMeasuredIterations << '\n' << "mean_ms=" << sum / samplesMs.size() << '\n' << "p95_ms=" << ordered[p95Index] << '\n' << "max_ms=" << ordered.back() << '\n' << "over_10ms=" << over10ms << '\n' << "cpu_seconds=" << cpuSeconds << std::endl;
}