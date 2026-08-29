#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

namespace {

bool finite(const ocs2::vector_t& value) {
  return value.array().isFinite().all();
}

TEST(SqpMpcSmoke, solves_linear_quadratic_problem_with_hpipm) {
  constexpr int stateDim = 1;
  constexpr int inputDim = 1;

  ocs2::matrix_t A = ocs2::matrix_t::Zero(stateDim, stateDim);
  ocs2::matrix_t B = ocs2::matrix_t::Identity(stateDim, inputDim);
  ocs2::OptimalControlProblem problem;
  problem.dynamicsPtr = std::make_unique<ocs2::LinearSystemDynamics>(A, B);

  const ocs2::matrix_t Q = 4.0 * ocs2::matrix_t::Identity(stateDim, stateDim);
  const ocs2::matrix_t R = 0.2 * ocs2::matrix_t::Identity(inputDim, inputDim);
  const ocs2::matrix_t P = ocs2::matrix_t::Zero(inputDim, stateDim);
  const ocs2::matrix_t Qf = 30.0 * ocs2::matrix_t::Identity(stateDim, stateDim);
  problem.costPtr->add("running_cost", std::make_unique<ocs2::QuadraticStateInputCost>(Q, R, P));
  problem.finalCostPtr->add("terminal_cost", std::make_unique<ocs2::QuadraticStateCost>(Qf));

  const ocs2::vector_t targetState = ocs2::vector_t::Zero(stateDim);
  const ocs2::vector_t targetInput = ocs2::vector_t::Zero(inputDim);
  auto referenceManager = std::make_shared<ocs2::ReferenceManager>(
      ocs2::TargetTrajectories({0.0}, {targetState}, {targetInput}));
  problem.targetTrajectoriesPtr = &referenceManager->getTargetTrajectories();

  ocs2::DefaultInitializer initializer(inputDim);
  ocs2::mpc::Settings mpcSettings;
  mpcSettings.timeHorizon_ = 1.0;
  ocs2::sqp::Settings sqpSettings;
  sqpSettings.dt = 0.1;
  sqpSettings.sqpIteration = 8;
  sqpSettings.nThreads = 2;
  sqpSettings.enableLogging = true;
  sqpSettings.hpipmSettings.hpipmMode = hpipm_mode::ROBUST;
  sqpSettings.hpipmSettings.reg_prim = 1e-8;

  ocs2::SqpMpc mpc(mpcSettings, sqpSettings, problem, initializer);
  mpc.getSolverPtr()->setReferenceManager(referenceManager);

  ocs2::vector_t initialState(stateDim);
  initialState << 1.0;
  ASSERT_TRUE(mpc.run(0.0, initialState));

  const auto solution = mpc.getSolverPtr()->primalSolution(1.0);
  ASSERT_FALSE(solution.timeTrajectory_.empty());
  ASSERT_EQ(solution.timeTrajectory_.size(), solution.stateTrajectory_.size());
  ASSERT_EQ(solution.timeTrajectory_.size(), solution.inputTrajectory_.size());
  ASSERT_FALSE(mpc.getSolverPtr()->getIterationsLog().empty());

  for (size_t i = 0; i < solution.timeTrajectory_.size(); ++i) {
    EXPECT_TRUE(std::isfinite(solution.timeTrajectory_[i]));
    EXPECT_TRUE(finite(solution.stateTrajectory_[i]));
    EXPECT_TRUE(finite(solution.inputTrajectory_[i]));
    if (i != 0) {
      EXPECT_GT(solution.timeTrajectory_[i], solution.timeTrajectory_[i - 1]);
    }
  }
  EXPECT_TRUE(std::isfinite(mpc.getSolverPtr()->getIterationsLog().back().cost));
  EXPECT_LT(solution.stateTrajectory_.back().norm(), initialState.norm());
}

}  // namespace
