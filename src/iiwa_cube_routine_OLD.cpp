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

  bool move_to_scan_pose() {
    std::vector<double> scan_joint_positions = {0.785, 0.0, 0.0, -1.5, 0.0, 1.0, 0.0};
    move_group_->setJointValueTarget(scan_joint_positions);
    auto err = move_group_->move();
    if (err == moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Scan pose reached. Waiting for fra2mo...");
      return true;
    }
    RCLCPP_ERROR(
      this->get_logger(), "Failed to reach scan pose (MoveIt error code: %d)",
      err.val);
    return false;
  }

  geometry_msgs::msg::Pose lookup_cube_pose_in_base(int cube_id) {
    geometry_msgs::msg::PoseStamped cube_world;
    cube_world.header.frame_id = "world";
    cube_world.header.stamp = rclcpp::Time(0);

    switch (cube_id) {
      case 1:
        cube_world.pose.position.x = -0.75;
        cube_world.pose.position.y = -7.5;
        break;
      case 2:
        cube_world.pose.position.x = -1.5;
        cube_world.pose.position.y = -7.5;
        break;
      case 3:
        cube_world.pose.position.x = -1.5;
        cube_world.pose.position.y = -6.75;
        break;
      default:
        throw std::runtime_error("invalid cube id");
    }

    cube_world.pose.position.z = 0.45;
    cube_world.pose.orientation.w = 1.0;

    auto cube_in_base = tf_buffer_->transform(cube_world, kBaseFrame, tf2::durationFromSec(2.0));
    geometry_msgs::msg::Pose approach_pose = cube_in_base.pose;
    approach_pose.orientation.x = 0.0;
    approach_pose.orientation.y = 1.0;
    approach_pose.orientation.z = 0.0;
    approach_pose.orientation.w = 0.0;
    return approach_pose;
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
          &IiwaWatchdog::execute_pick_and_place_sequence, this, cube_id, stable_pose).detach();
        break;
      }

      case RobotState::EXECUTE_TASK:
        break;

      case RobotState::COOLDOWN:
        if ((this->now() - stability_start_time_).seconds() > 15.0) {
          current_state_ = RobotState::IDLE;
          RCLCPP_INFO(this->get_logger(), "Cooldown finished. Listening again.");
        }
        break;
    }
  }

  void execute_pick_and_place_sequence(int cube_id, geometry_msgs::msg::Pose drop_pose_in_camera) {
    try {
      RCLCPP_INFO(this->get_logger(), "Approaching cube %d...", cube_id);
      geometry_msgs::msg::Pose target_cube_pose = lookup_cube_pose_in_base(cube_id);

      move_group_->setPoseTarget(target_cube_pose);
      if (move_group_->move() != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach cube %d.", cube_id);
        current_state_ = RobotState::IDLE;
        motion_in_progress_ = false;
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Simulating grasp...");
      std::this_thread::sleep_for(2s);

      geometry_msgs::msg::PoseStamped pose_in_camera;
      pose_in_camera.header.frame_id = kCameraFrame;
      pose_in_camera.header.stamp = rclcpp::Time(0);
      pose_in_camera.pose = drop_pose_in_camera;

      geometry_msgs::msg::PoseStamped pose_in_base =
        tf_buffer_->transform(pose_in_camera, kBaseFrame, tf2::durationFromSec(2.0));

      geometry_msgs::msg::Pose place_pose = pose_in_base.pose;
      place_pose.position.z += 0.25;
      place_pose.orientation = target_cube_pose.orientation;

      move_group_->setPoseTarget(place_pose);
      if (move_group_->move() == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Simulating release on fra2mo...");
        std::this_thread::sleep_for(2s);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to reach fra2mo drop pose.");
      }
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "TF error during pick-and-place: %s", ex.what());
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Pick-and-place error: %s", ex.what());
    }

    move_to_scan_pose();
    stability_start_time_ = this->now();
    current_state_ = RobotState::COOLDOWN;
    motion_in_progress_ = false;
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
