#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mutex>
#include <thread>
#include <cmath>
#include <atomic>
#include <string>

using namespace std::chrono_literals;

enum class RobotState {
  INIT_SCAN_POSE,
  IDLE,
  CHECK_STABILITY,
  EXECUTE_TASK,
  COOLDOWN
};

class IiwaWatchdog : public rclcpp::Node {
public:
  IiwaWatchdog(const rclcpp::NodeOptions & options)
  : Node("iiwa_cube_routine", options)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    aruco_sub_ = this->create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
      "aruco_detections", 10,
      std::bind(&IiwaWatchdog::aruco_callback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/iiwa/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!msg->name.empty()) {
          joint_states_received_ = true;
        }
      });

    RCLCPP_INFO(this->get_logger(), "IIWA routine node ready (ns: %s).", this->get_namespace());
  }

  bool wait_for_joint_states(std::chrono::seconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok() && !joint_states_received_ &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(100ms);
    }

    if (!joint_states_received_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for /iiwa/joint_states. Is simulation.launch.py running?");
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "Received /iiwa/joint_states.");
    return true;
  }

  bool setup_moveit() {
    RCLCPP_INFO(this->get_logger(), "Connecting to move_group action server...");

    moveit::planning_interface::MoveGroupInterface::Options options("iiwa_arm");
    options.move_group_namespace_ = "/iiwa";

    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), options);

    move_group_->setPlanningTime(10.0);
    move_group_->setNumPlanningAttempts(5);
    move_group_->setPoseReferenceFrame("iiwa_iiwa_base");
    move_group_->setEndEffectorLink("iiwa_tool0");
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);
    move_group_->setGoalPositionTolerance(0.02);
    move_group_->setGoalOrientationTolerance(M_PI);

    RCLCPP_INFO(
      this->get_logger(), "MoveIt connected. Planning frame: %s",
      move_group_->getPlanningFrame().c_str());
    return true;
  }

  void start_fsm() {
    fsm_timer_ = this->create_wall_timer(100ms, std::bind(&IiwaWatchdog::fsm_loop, this));
    RCLCPP_INFO(this->get_logger(), "FSM started. Moving to scan pose...");
  }

private:
  static constexpr const char * kBaseFrame = "iiwa_iiwa_base";
  static constexpr const char * kCameraFrame = "iiwa_camera_link";
  static constexpr double kCooldownSeconds = 20.0;

  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr aruco_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  RobotState current_state_ = RobotState::INIT_SCAN_POSE;
  std::atomic<bool> motion_in_progress_{false};
  std::atomic<bool> joint_states_received_{false};

  std::mutex data_mutex_;
  bool target_visible_ = false;
  int current_target_id_ = -1;
  geometry_msgs::msg::Pose target_pose_;

  rclcpp::Time stability_start_time_;
  geometry_msgs::msg::Pose initial_stability_pose_;

  static int cube_id_from_tag(int tag_id) {
    if (tag_id >= 1 && tag_id <= 3) {
      return tag_id;
    }
    if (tag_id == 6) {
      return 2;
    }
    return -1;
  }

  static std::string cube_state_name(int cube_id) {
    return "cube_" + std::to_string(cube_id);
  }

  void aruco_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!msg->markers.empty()) {
      target_visible_ = true;
      current_target_id_ = msg->markers[0].marker_id;
      target_pose_ = msg->markers[0].pose;
    } else {
      target_visible_ = false;
    }
  }

  bool execute_move() {
    auto err = move_group_->move();
    if (err != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        this->get_logger(), "MoveIt move() failed (error code: %d)", err.val);
      return false;
    }
    return true;
  }

  bool move_to_named_state(const std::string & state_name) {
    if (!move_group_->setNamedTarget(state_name)) {
      RCLCPP_ERROR(this->get_logger(), "Unknown named target: %s", state_name.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Moving to named state '%s'...", state_name.c_str());
    return execute_move();
  }

  bool move_to_scan_pose() {
    return move_to_named_state("scan");
  }

  bool move_to_cube(int cube_id) {
    return move_to_named_state(cube_state_name(cube_id));
  }

  void finish_task(bool success) {
    move_to_scan_pose();
    stability_start_time_ = this->now();
    current_state_ = RobotState::COOLDOWN;
    motion_in_progress_ = false;
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Pick-and-place complete. Cooldown started.");
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "Pick-and-place failed. Returning to scan pose; cooldown %.0fs.",
        kCooldownSeconds);
    }
  }

  void fsm_loop() {
    if (motion_in_progress_) {
      return;
    }

    switch (current_state_) {
      case RobotState::INIT_SCAN_POSE:
        motion_in_progress_ = true;
        std::thread([this]() {
          move_to_scan_pose();
          motion_in_progress_ = false;
          current_state_ = RobotState::IDLE;
        }).detach();
        break;

      case RobotState::IDLE: {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const int cube_id = cube_id_from_tag(current_target_id_);
        if (target_visible_ && cube_id > 0) {
          initial_stability_pose_ = target_pose_;
          stability_start_time_ = this->now();
          current_state_ = RobotState::CHECK_STABILITY;
          RCLCPP_INFO(
            this->get_logger(),
            "ArUco tag %d detected (cube %d). Checking stability...",
            current_target_id_, cube_id);
        }
        break;
      }

      case RobotState::CHECK_STABILITY: {
        int cube_id = -1;
        geometry_msgs::msg::Pose stable_pose;
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          if (!target_visible_) {
            current_state_ = RobotState::IDLE;
            return;
          }

          cube_id = cube_id_from_tag(current_target_id_);
          stable_pose = target_pose_;

          const double dx = target_pose_.position.x - initial_stability_pose_.position.x;
          const double dy = target_pose_.position.y - initial_stability_pose_.position.y;
          const double distance = std::hypot(dx, dy);

          if (distance > 0.02) {
            initial_stability_pose_ = target_pose_;
            stability_start_time_ = this->now();
            return;
          }

          if ((this->now() - stability_start_time_).seconds() <= 2.0) {
            return;
          }
        }

        RCLCPP_INFO(this->get_logger(), "Target stable. Starting pick-and-place.");
        current_state_ = RobotState::EXECUTE_TASK;
        motion_in_progress_ = true;
        std::thread(
          &IiwaWatchdog::execute_pick_and_place_sequence, this, cube_id).detach();
        break;
      }

      case RobotState::EXECUTE_TASK:
        break;

      case RobotState::COOLDOWN:
        if ((this->now() - stability_start_time_).seconds() > kCooldownSeconds) {
          current_state_ = RobotState::IDLE;
          RCLCPP_INFO(this->get_logger(), "Cooldown finished. Listening again.");
        }
        break;
    }
  }

  void execute_pick_and_place_sequence(int cube_id) {
    bool success = false;

    try {
      RCLCPP_INFO(this->get_logger(), "Approaching cube %d...", cube_id);
      if (!move_to_cube(cube_id)) {
        finish_task(false);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Simulating grasp...");
      std::this_thread::sleep_for(2s);

      RCLCPP_INFO(this->get_logger(), "Moving toward fra2mo to release...");
      if (!move_to_named_state("drop")) {
        finish_task(false);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Simulating release on fra2mo...");
      std::this_thread::sleep_for(2s);
      success = true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "TF error during pick-and-place: %s", ex.what());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Pick-and-place error: %s", ex.what());
    }

    finish_task(success);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<IiwaWatchdog>(node_options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  std::this_thread::sleep_for(3s);

  if (!node->wait_for_joint_states(15s)) {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  if (!node->setup_moveit()) {
    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  node->start_fsm();

  spinner.join();
  rclcpp::shutdown();
  return 0;
}
