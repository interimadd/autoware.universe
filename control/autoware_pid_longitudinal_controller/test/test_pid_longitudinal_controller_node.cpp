// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller.hpp"
#include "autoware/trajectory_follower_base/input_data.hpp"
#include "autoware/trajectory_follower_base/longitudinal_controller_base.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace trajectory_follower = autoware::motion::control::trajectory_follower;
using autoware::motion::control::pid_longitudinal_controller::PidLongitudinalController;
using autoware_adapi_v1_msgs::msg::OperationModeState;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

// `PidLongitudinalController` is not itself a node; it is a controller plugin that the trajectory
// follower node loads. It takes a node reference (only for parameter/component initialization) and
// is driven through the `LongitudinalControllerBase::run(InputData) -> LongitudinalOutput` API.
// These characterization tests build the input messages, call `run()`, and assert on the returned
// longitudinal control command, capturing the current input/output behavior before refactoring.

// Builds node options the controller constructor needs: the package default parameters, the vehicle
// dimensions VehicleInfoUtils reads, and the ego nearest search thresholds. The vehicle info and
// nearest search parameters come from the shared autoware_test_utils config files.
// `enable_keep_stopped_until_steer_convergence` is overridden to false so the controller can leave
// the initial STOPPED state and reach the DRIVE state (the longest path); the override takes
// precedence over the value loaded from the parameter files.
rclcpp::NodeOptions make_node_options()
{
  const auto controller_share_dir =
    ament_index_cpp::get_package_share_directory("autoware_pid_longitudinal_controller");
  const auto test_utils_share_dir =
    ament_index_cpp::get_package_share_directory("autoware_test_utils");

  rclcpp::NodeOptions node_options;
  node_options.append_parameter_override("enable_keep_stopped_until_steer_convergence", false);
  autoware::test_utils::updateNodeOptions(
    node_options,
    {test_utils_share_dir + "/config/test_vehicle_info.param.yaml",
     test_utils_share_dir + "/config/test_nearest_search.param.yaml",
     controller_share_dir + "/config/autoware_pid_longitudinal_controller.param.yaml"});
  return node_options;
}

TrajectoryPoint make_trajectory_point(const double x, const double y, const float velocity)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = velocity;
  return point;
}

/// @brief Create a straight reference trajectory along +x at y = 0 with a uniform target velocity.
///
///   x=0            x=50           x=100
///    *--------------*--------------*   longitudinal_velocity_mps = `velocity` at every point
///   ego
///
/// With a positive velocity the stop distance reaches the far end (~100 m), which is well beyond
/// `drive_state_stop_dist`, so the controller departs from STOPPED into DRIVE.
Trajectory make_straight_trajectory(const float velocity)
{
  Trajectory trajectory;
  trajectory.points.push_back(make_trajectory_point(0.0, 0.0, velocity));
  trajectory.points.push_back(make_trajectory_point(50.0, 0.0, velocity));
  trajectory.points.push_back(make_trajectory_point(100.0, 0.0, velocity));
  return trajectory;
}

Odometry make_odometry(const double position_x, const double velocity_x)
{
  Odometry odometry;
  odometry.pose.pose.position.x = position_x;
  odometry.pose.pose.orientation.w = 1.0;
  odometry.twist.twist.linear.x = velocity_x;
  return odometry;
}

AccelWithCovarianceStamped make_acceleration(const double acceleration_x)
{
  AccelWithCovarianceStamped acceleration;
  acceleration.accel.accel.linear.x = acceleration_x;
  return acceleration;
}

OperationModeState make_autonomous_operation_mode()
{
  OperationModeState operation_mode;
  operation_mode.mode = OperationModeState::AUTONOMOUS;
  operation_mode.is_autoware_control_enabled = true;
  return operation_mode;
}

trajectory_follower::InputData make_input_data(
  const Trajectory & trajectory, const Odometry & odometry,
  const AccelWithCovarianceStamped & acceleration, const OperationModeState & operation_mode)
{
  trajectory_follower::InputData input_data;
  input_data.current_trajectory = trajectory;
  input_data.current_odometry = odometry;
  input_data.current_accel = acceleration;
  input_data.current_operation_mode = operation_mode;
  return input_data;
}

class PidLongitudinalControllerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_pid_longitudinal_controller", make_node_options());
    // `ctrl_period` is read with get_parameter(), so it must be declared on the node before the
    // controller is constructed (the controller itself does not declare it).
    node_->declare_parameter<double>("ctrl_period", 0.03);
    diag_updater_ = std::make_shared<diagnostic_updater::Updater>(node_.get());
    controller_ = std::make_shared<PidLongitudinalController>(*node_, diag_updater_);
  }

  void TearDown() override
  {
    controller_.reset();
    diag_updater_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  trajectory_follower::LongitudinalOutput run_controller(
    const trajectory_follower::InputData & input_data)
  {
    return controller_->run(input_data);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<diagnostic_updater::Updater> diag_updater_;
  std::shared_ptr<trajectory_follower::LongitudinalControllerBase> controller_;
};

// A stopping trajectory (all zero target velocity) with the ego already stopped keeps the
// controller in the STOPPED state: it commands zero velocity and a negative (braking) acceleration.
TEST_F(PidLongitudinalControllerTest, StoppedTrajectoryOutputsBrakingStopCommand)
{
  // Arrange
  const auto input_data = make_input_data(
    make_straight_trajectory(0.0f), make_odometry(0.0, 0.0), make_acceleration(0.0),
    make_autonomous_operation_mode());

  // Act
  const auto output = run_controller(input_data);

  // Assert
  EXPECT_FLOAT_EQ(output.control_cmd.velocity, 0.0f);
  EXPECT_LT(output.control_cmd.acceleration, 0.0f);
}

// Longest path: with the ego under autonomous control, moving, and far from the stop point, the
// controller departs into the DRIVE state and runs the full PID velocity feedback. A target
// velocity higher than the current one yields a positive (accelerating) acceleration command.
TEST_F(PidLongitudinalControllerTest, AcceleratingTrajectoryDrivesTowardHigherTargetVelocity)
{
  // Arrange
  constexpr double current_velocity = 0.5;
  const auto input_data = make_input_data(
    make_straight_trajectory(1.0f), make_odometry(0.0, current_velocity), make_acceleration(0.0),
    make_autonomous_operation_mode());

  // Act
  const auto output = run_controller(input_data);

  // Assert
  EXPECT_GT(output.control_cmd.velocity, static_cast<float>(current_velocity));
  EXPECT_GT(output.control_cmd.acceleration, 0.0f);
}

// DRIVE state with a target velocity lower than the current one: the PID velocity feedback yields a
// negative (decelerating) acceleration command.
TEST_F(PidLongitudinalControllerTest, DeceleratingTrajectoryDrivesTowardLowerTargetVelocity)
{
  // Arrange
  constexpr double current_velocity = 1.0;
  const auto input_data = make_input_data(
    make_straight_trajectory(0.5f), make_odometry(0.0, current_velocity), make_acceleration(0.0),
    make_autonomous_operation_mode());

  // Act
  const auto output = run_controller(input_data);

  // Assert
  EXPECT_LT(output.control_cmd.velocity, static_cast<float>(current_velocity));
  EXPECT_LT(output.control_cmd.acceleration, 0.0f);
}

// Steady-state periodic control: once in DRIVE, repeated calls reuse the stored command history,
// so the delay compensation predicts the state from past commands. The controller stays in DRIVE
// and keeps commanding the target velocity with a positive (accelerating) acceleration.
TEST_F(PidLongitudinalControllerTest, RepeatedControlCallsSustainDriveWithDelayCompensation)
{
  // Arrange
  constexpr double current_velocity = 0.5;
  constexpr int control_cycle_count = 10;
  const auto input_data = make_input_data(
    make_straight_trajectory(1.0f), make_odometry(0.0, current_velocity), make_acceleration(0.0),
    make_autonomous_operation_mode());

  // Act
  trajectory_follower::LongitudinalOutput output;
  for (int control_cycle = 0; control_cycle < control_cycle_count; ++control_cycle) {
    output = run_controller(input_data);
  }

  // Assert
  EXPECT_GT(output.control_cmd.velocity, static_cast<float>(current_velocity));
  EXPECT_GT(output.control_cmd.acceleration, 0.0f);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
