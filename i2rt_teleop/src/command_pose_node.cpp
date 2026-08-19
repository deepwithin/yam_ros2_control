// Minimal debug tool: publishes exactly ONE JointTrajectory command to a
// fixed pose shortly after startup, then does nothing else ever again - no
// continuous republishing, no velocity streaming, no per-cycle clamping.
//
// Written to isolate whether leader_follower_node's continuous re-streaming
// of mirrored targets (a fresh single-point trajectory every ~33ms) is
// itself what causes struggling/vibration against gravity on a heavily-
// loaded joint, or whether that's a gravity-compensation/torque-limit issue
// that would show up regardless of how the target got there. If THIS node
// holds the pose cleanly (matching what a MoveIt-executed trajectory does)
// while leader_follower_node's live mirroring does not, the difference is
// in the streaming/re-publishing behavior, not gravity comp itself.
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace i2rt_teleop
{

class CommandPoseNode : public rclcpp::Node
{
public:
  CommandPoseNode() : Node("command_pose_node")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"});
    // Default is the exact pose captured from /joint_states at a MoveIt-
    // commanded pose that held stably against gravity - override with
    // -p positions:="[...]" to try others.
    positions_ = declare_parameter<std::vector<double>>(
      "positions",
      {0.27752346074616874, 0.2958342870222008, 0.22716868848706717, -1.323910887312124, -0.03643091477836258,
       -0.12340733958953187});
    // Relative by default so a namespace remap (-r __ns:=/follower) targets
    // the right arm's controller without editing this default.
    trajectory_topic_ =
      declare_parameter<std::string>("trajectory_topic", "joint_trajectory_controller/joint_trajectory");
    move_duration_s_ = declare_parameter<double>("move_duration_s", 4.0);
    startup_delay_s_ = declare_parameter<double>("startup_delay_s", 2.0);

    if (positions_.size() != joint_names_.size()) {
      throw std::runtime_error("positions must be the same length as joint_names");
    }

    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(trajectory_topic_, 10);

    RCLCPP_INFO(
      get_logger(),
      "Will publish exactly one trajectory to '%s' in %.1fs: a single point at the configured pose, %.1fs "
      "move duration. Nothing further will ever be sent by this node - Ctrl+C once you're done observing.",
      trajectory_topic_.c_str(), startup_delay_s_, move_duration_s_);

    timer_ = create_wall_timer(std::chrono::duration<double>(startup_delay_s_), [this]() {
      trajectory_msgs::msg::JointTrajectory traj;
      traj.joint_names = joint_names_;
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = positions_;
      point.time_from_start = rclcpp::Duration::from_seconds(move_duration_s_);
      traj.points.push_back(point);
      pub_->publish(traj);
      RCLCPP_INFO(get_logger(), "Published. This node will not publish anything else - watch the arm now.");
      timer_->cancel();
    });
  }

private:
  std::vector<std::string> joint_names_;
  std::vector<double> positions_;
  std::string trajectory_topic_;
  double move_duration_s_ = 4.0;
  double startup_delay_s_ = 2.0;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace i2rt_teleop

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<i2rt_teleop::CommandPoseNode>());
  rclcpp::shutdown();
  return 0;
}
