/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Andy Zelenak */
#include <fstream>

#include <gtest/gtest.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/utils/robot_model_test_utils.h>

using trajectory_processing::TimeOptimalTrajectoryGeneration;

namespace
{
constexpr double DEFAULT_TIMESTEP = 0.1;  // sec
constexpr char JOINT_GROUP[] = "panda_arm";

class RuckigTests : public testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = moveit::core::loadTestingRobotModel("panda");
    trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model_, JOINT_GROUP);
  }

  moveit::core::RobotModelPtr robot_model_;
  robot_trajectory::RobotTrajectoryPtr trajectory_;
  trajectory_processing::RuckigSmoothing smoother_;
};

}  // namespace

#if 0
TEST_F(RuckigTests, basic_trajectory)
{
  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // First waypoint is default joint positions
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // Second waypoint has slightly-different joint positions
  std::vector<double> joint_positions;
  robot_state.copyJointGroupPositions(JOINT_GROUP, joint_positions);
  joint_positions.at(0) += 0.05;
  robot_state.setJointGroupPositions(JOINT_GROUP, joint_positions);
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  EXPECT_TRUE(
      smoother_.applySmoothing(*trajectory_, 1.0 /* max vel scaling factor */, 1.0 /* max accel scaling factor */));
}
#endif

TEST_F(RuckigTests, longer_trajectory)
{
  size_t JOINT_IDX = 0;

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // First waypoint is default joint positions
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // add step wise change to waypoints
  std::vector<double> joint_positions;
  for (int i = 0; i < 10; i++)
  {
    robot_state.copyJointGroupPositions(JOINT_GROUP, joint_positions);
    joint_positions.at(JOINT_IDX) += 0.1 * (i % 2 - 0.5);
    robot_state.setJointGroupPositions(JOINT_GROUP, joint_positions);
    trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);
  }

  // robot_model_->printModelInfo(std::cerr);

  std::ofstream file1("orig_trajectory.csv");
  file1 << "t,p,v,a\n";  // Write the header

  std::cout << "Original trajectory:" << std::endl;
  std::vector<double> joint_velocities, joint_accelerations;
  for (size_t i = 0; i < trajectory_->getWayPointCount(); i++)
  {
    trajectory_->getWayPoint(i).copyJointGroupPositions(JOINT_GROUP, joint_positions);
    trajectory_->getWayPoint(i).copyJointGroupVelocities(JOINT_GROUP, joint_velocities);
    trajectory_->getWayPoint(i).copyJointGroupAccelerations(JOINT_GROUP, joint_accelerations);
    std::cout << "t: " << trajectory_->getWayPointDurationFromStart(i) 
    << ", p: " << joint_positions.at(JOINT_IDX) 
    << ", v: " << joint_velocities.at(JOINT_IDX) 
    << ", a: " << joint_accelerations.at(JOINT_IDX) 
    << std::endl;
    file1 << trajectory_->getWayPointDurationFromStart(i) << "," 
    << joint_positions.at(JOINT_IDX) << "," 
    << joint_velocities.at(JOINT_IDX) << "," 
    << joint_accelerations.at(JOINT_IDX) << "\n";
  }
  file1.close();

  EXPECT_TRUE(smoother_.applySmoothing(*trajectory_, 1.0 /* max vel scaling factor */,
                                       1.0 /* max accel scaling factor */, true, 0.01));

  std::ofstream file2("smoothed_trajectory.csv");
  file2 << "t,p,v,a\n";  // Write the header
  std::cout << "After ruckig smoothing:" << std::endl;
  for (size_t i = 0; i < trajectory_->getWayPointCount(); i++)
  {
    trajectory_->getWayPoint(i).copyJointGroupPositions(JOINT_GROUP, joint_positions);
    trajectory_->getWayPoint(i).copyJointGroupVelocities(JOINT_GROUP, joint_velocities);
    trajectory_->getWayPoint(i).copyJointGroupAccelerations(JOINT_GROUP, joint_accelerations);
    std::cout << "t: " << trajectory_->getWayPointDurationFromStart(i) 
    << ", p: " << joint_positions.at(JOINT_IDX) 
    << ", v: " << joint_velocities.at(JOINT_IDX) 
    << ", a: " << joint_accelerations.at(JOINT_IDX) 
    << std::endl;
    file2 << trajectory_->getWayPointDurationFromStart(i) << "," 
    << joint_positions.at(JOINT_IDX) << "," 
    << joint_velocities.at(JOINT_IDX) << "," 
    << joint_accelerations.at(JOINT_IDX) << "\n";
  }
  file2.close();
}

TEST_F(RuckigTests, longer_trajectory_totg)
{
  size_t JOINT_IDX = 0;

  // Custom velocity & acceleration limits for some joints
  std::unordered_map<std::string, double> vel_limits{ { "panda_joint1", 1.3 }, { "panda_joint2", 2.3 },
                                                      { "panda_joint3", 3.3 }, { "panda_joint4", 4.3 },
                                                      { "panda_joint5", 5.3 }, { "panda_joint6", 6.3 },
                                                      { "panda_joint7", 7.3 } };
  std::unordered_map<std::string, double> accel_limits{ { "panda_joint1", 1.5 }, { "panda_joint2", 2.3 },
                                                        { "panda_joint3", 3.3 }, { "panda_joint4", 4.3 },
                                                        { "panda_joint5", 5.3 }, { "panda_joint6", 6.3 },
                                                        { "panda_joint7", 7.3 } };
  std::unordered_map<std::string, double> jerk_limits{ { "panda_joint1", 1.0 }, { "panda_joint2", 2.3 },
                                                        { "panda_joint3", 3.3 }, { "panda_joint4", 4.3 },
                                                        { "panda_joint5", 5.3 }, { "panda_joint6", 6.3 },
                                                        { "panda_joint7", 7.3 } };

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // First waypoint is default joint positions
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // add step wise change to waypoints
  std::vector<double> joint_positions;
  for (int i = 0; i < 10; i++)
  {
    robot_state.copyJointGroupPositions(JOINT_GROUP, joint_positions);
    joint_positions.at(JOINT_IDX) += 0.1 * (i % 2 - 0.5);
    joint_positions.at(JOINT_IDX+1) += 0.1 * (i % 4 - 2.0);
    robot_state.setJointGroupPositions(JOINT_GROUP, joint_positions);
    trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);
  }

  robot_model_->printModelInfo(std::cerr);

  std::ofstream file1("orig_trajectory.csv");
  file1 << "t,p,v,a\n";  // Write the header

  std::cout << "Original trajectory:" << std::endl;
  std::vector<double> joint_velocities, joint_accelerations;
  for (size_t i = 0; i < trajectory_->getWayPointCount(); i++)
  {
    trajectory_->getWayPoint(i).copyJointGroupPositions(JOINT_GROUP, joint_positions);
    trajectory_->getWayPoint(i).copyJointGroupVelocities(JOINT_GROUP, joint_velocities);
    trajectory_->getWayPoint(i).copyJointGroupAccelerations(JOINT_GROUP, joint_accelerations);
    std::cout << "t: " << trajectory_->getWayPointDurationFromStart(i) 
    << ", p: " << joint_positions.at(JOINT_IDX) 
    << ", v: " << joint_velocities.at(JOINT_IDX) 
    << ", a: " << joint_accelerations.at(JOINT_IDX) 
    << std::endl;
    file1 << trajectory_->getWayPointDurationFromStart(i) << "," 
    << joint_positions.at(JOINT_IDX) << "," 
    << joint_velocities.at(JOINT_IDX) << "," 
    << joint_accelerations.at(JOINT_IDX) << "\n";
  }
  file1.close();

  // ####### TOTG ######## 
  TimeOptimalTrajectoryGeneration totg;
  ASSERT_TRUE(totg.computeTimeStamps(*trajectory_, vel_limits, accel_limits)) << "Failed to compute time stamps";

  std::ofstream file_totg("totg_trajectory.csv");
  file_totg << "t,p,v,a,j\n";  // Write the header
  std::cout << "After TOTG parameterization:" << std::endl;
  double old_acc = 0.0;
  for (size_t i = 0; i < trajectory_->getWayPointCount(); i++)
  {
    trajectory_->getWayPoint(i).copyJointGroupPositions(JOINT_GROUP, joint_positions);
    trajectory_->getWayPoint(i).copyJointGroupVelocities(JOINT_GROUP, joint_velocities);
    trajectory_->getWayPoint(i).copyJointGroupAccelerations(JOINT_GROUP, joint_accelerations);
    double jerk = i > 0 ? (joint_accelerations.at(JOINT_IDX) - old_acc) / trajectory_->getWayPointDurationFromPrevious(i) : 0.0;    
    std::cout << "t: " << trajectory_->getWayPointDurationFromStart(i) 
    << ", p: " << joint_positions.at(JOINT_IDX) 
    << ", v: " << joint_velocities.at(JOINT_IDX) 
    << ", a: " << joint_accelerations.at(JOINT_IDX) 
    << ", j: " << jerk 
    << std::endl;
    file_totg << trajectory_->getWayPointDurationFromStart(i) << "," 
    << joint_positions.at(JOINT_IDX) << "," 
    << joint_velocities.at(JOINT_IDX) << "," 
    << joint_accelerations.at(JOINT_IDX) << "," 
    << jerk << "\n";

    old_acc = joint_accelerations.at(JOINT_IDX) ;
  }
  file_totg.close();

  // ####### ruckig ######## 
  EXPECT_TRUE(smoother_.applySmoothing(*trajectory_, vel_limits, accel_limits, jerk_limits));

  std::ofstream file_ruckig("ruckig_trajectory.csv");
  file_ruckig << "t,p,v,a,j\n";  // Write the header
  std::cout << "After ruckig smoothing:" << std::endl;
  old_acc = 0.0;
  for (size_t i = 0; i < trajectory_->getWayPointCount(); i++)
  {
    trajectory_->getWayPoint(i).copyJointGroupPositions(JOINT_GROUP, joint_positions);
    trajectory_->getWayPoint(i).copyJointGroupVelocities(JOINT_GROUP, joint_velocities);
    trajectory_->getWayPoint(i).copyJointGroupAccelerations(JOINT_GROUP, joint_accelerations);
    double jerk = i > 0 ? (joint_accelerations.at(JOINT_IDX) - old_acc) / trajectory_->getWayPointDurationFromPrevious(i) : 0.0;    
    std::cout << "t: " << trajectory_->getWayPointDurationFromStart(i) 
    << ", p: " << joint_positions.at(JOINT_IDX) 
    << ", v: " << joint_velocities.at(JOINT_IDX) 
    << ", a: " << joint_accelerations.at(JOINT_IDX) 
    << ", j: " << jerk 
    << std::endl;
    file_ruckig << trajectory_->getWayPointDurationFromStart(i) << "," 
    << joint_positions.at(JOINT_IDX) << "," 
    << joint_velocities.at(JOINT_IDX) << "," 
    << joint_accelerations.at(JOINT_IDX) << "," 
    << jerk << "\n";

    old_acc = joint_accelerations.at(JOINT_IDX) ;
  }
  file_ruckig.close();
}

TEST_F(RuckigTests, basic_trajectory_with_custom_limits)
{
  // Check the version of computeTimeStamps that takes custom velocity/acceleration limits

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // First waypoint is default joint positions
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // Second waypoint has slightly-different joint positions
  std::vector<double> joint_positions;
  robot_state.copyJointGroupPositions(JOINT_GROUP, joint_positions);
  joint_positions.at(0) += 0.05;
  robot_state.setJointGroupPositions(JOINT_GROUP, joint_positions);
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // Custom velocity & acceleration limits for some joints
  std::unordered_map<std::string, double> vel_limits{ { "panda_joint1", 1.3 } };
  std::unordered_map<std::string, double> accel_limits{ { "panda_joint2", 2.3 }, { "panda_joint3", 3.3 } };
  std::unordered_map<std::string, double> jerk_limits{ { "panda_joint5", 100.0 } };

  EXPECT_TRUE(smoother_.applySmoothing(*trajectory_, vel_limits, accel_limits, jerk_limits));
}

TEST_F(RuckigTests, trajectory_duration)
{
  // Compare against the OJET online trajectory generator: https://www.trajectorygenerator.com/ojet-online/
  // Limits can be verified like this. Note that Ruckig applies defaults if the RobotModel has none.
  // robot_model_->printModelInfo(std::cerr);
  const double ideal_duration = 0.210;

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // Special attention to Joint 0. It is the only joint to move in this test.
  // Zero velocities and accelerations at the endpoints
  robot_state.setVariablePosition("panda_joint1", 0.0);
  trajectory_->addSuffixWayPoint(robot_state, 0.0);

  robot_state.setVariablePosition("panda_joint1", 0.1);
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  EXPECT_TRUE(
      smoother_.applySmoothing(*trajectory_, 1.0 /* max vel scaling factor */, 1.0 /* max accel scaling factor */));

  // No waypoint durations of zero except the first
  for (size_t waypoint_idx = 1; waypoint_idx < trajectory_->getWayPointCount() - 1; ++waypoint_idx)
  {
    EXPECT_NE(trajectory_->getWayPointDurationFromPrevious(waypoint_idx), 0);
  }

  // The trajectory duration should be within 10% of the analytical solution since the implementation here extends
  // the duration by 10% at every iteration.
  EXPECT_GT(trajectory_->getWayPointDurationFromStart(trajectory_->getWayPointCount() - 1), 0.9999 * ideal_duration);
  EXPECT_LT(trajectory_->getWayPointDurationFromStart(trajectory_->getWayPointCount() - 1), 1.11 * ideal_duration);
}

TEST_F(RuckigTests, single_waypoint)
{
  // With only one waypoint, Ruckig cannot smooth the trajectory.
  // It should simply pass the trajectory through unmodified and return true.

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
  robot_state.zeroVelocities();
  robot_state.zeroAccelerations();
  // First waypoint is default joint positions
  trajectory_->addSuffixWayPoint(robot_state, DEFAULT_TIMESTEP);

  // Trajectory should not change
  auto first_waypoint_input = robot_state;

  // Only one waypoint is acceptable. True is returned.
  EXPECT_TRUE(
      smoother_.applySmoothing(*trajectory_, 1.0 /* max vel scaling factor */, 1.0 /* max accel scaling factor */));
  // And the waypoint did not change
  const auto new_first_waypoint = trajectory_->getFirstWayPointPtr();
  const auto& variable_names = new_first_waypoint->getVariableNames();
  for (const std::string& variable_name : variable_names)
  {
    EXPECT_EQ(first_waypoint_input.getVariablePosition(variable_name),
              new_first_waypoint->getVariablePosition(variable_name));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
