#include <memory>
#include <chrono>
#include <cmath>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

#include "people_msgs/msg/people.hpp"
#include <mutex>

using namespace std::chrono_literals;

class Fra2moNavigator : public rclcpp::Node
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  Fra2moNavigator() : Node("fra2mo_navigator")
  {
    this->declare_parameter("goal_name", "iiwa");

    // Client per l'azione del Planner globale
    this->planner_action_client_ = rclcpp_action::create_client<ComputePathToPose>(this, "/compute_path_to_pose");
    
    // Publisher diretto ai motori del robot
    this->cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/fra2mo/cmd_vel", 10);

    this->people_sub_ = this->create_subscription<people_msgs::msg::People>(
      "/people", 10, std::bind(&Fra2moNavigator::people_callback, this, std::placeholders::_1));

    // Listener per conoscere la posizione attuale del robot sulla mappa
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer per il ciclo di controllo (20 Hz)
    this->control_timer_ = this->create_wall_timer(50ms, std::bind(&Fra2moNavigator::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Navigatore diretto inizializzato.");
  }

  void send_target()
  {
    if (!this->planner_action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Planner non disponibile");
      return;
    }

    auto goal_name = this->get_parameter("goal_name").as_string();
    auto it = accepted_goals_.find(goal_name);
    auto coords = it->second;

    auto goal_msg = ComputePathToPose::Goal();
    goal_msg.goal.header.frame_id = "map";
    goal_msg.goal.header.stamp = this->now();
    goal_msg.goal.pose.position.x = coords[0];
    goal_msg.goal.pose.position.y = coords[1];
    goal_msg.goal.pose.orientation.w = 1.0;
    goal_msg.use_start = false;

    RCLCPP_INFO(this->get_logger(), "Chiedo un percorso sicuro per X=%.2f, Y=%.2f", coords[0], coords[1]);
    
    auto send_goal_options = rclcpp_action::Client<ComputePathToPose>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&Fra2moNavigator::planner_callback, this, std::placeholders::_1);
    this->planner_action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<ComputePathToPose>::SharedPtr planner_action_client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
  std::vector<people_msgs::msg::Person> current_people_;
  std::mutex people_mutex_;

  nav2_msgs::action::ComputePathToPose::Result::SharedPtr current_path_;
  size_t current_waypoint_index_ = 0;

  const std::unordered_map<std::string, std::vector<double>> accepted_goals_ {
    {"meccanico",{-7.5, -6.8}},
    {"iiwa",{0,-6}}
  };

  void planner_callback(const GoalHandleComputePath::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      current_path_ = result.result;
      current_waypoint_index_ = 0;
      RCLCPP_INFO(this->get_logger(), "Percorso sociale ricevuto (%zu pose). Mi muovo.", current_path_->path.poses.size());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Il planner ha rifiutato la richiesta.");
    }
  }

  void people_callback(const people_msgs::msg::People::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(people_mutex_);
    current_people_ = msg->people;
  }

  int planner_tick_count = 0;

  void control_loop()
  {
    planner_tick_count++;

    if (planner_tick_count >= 100) {
    planner_tick_count = 0;
    this->send_target(); 
    }

    if (!current_path_ || current_waypoint_index_ >= current_path_->path.poses.size()) {
      return; // Nessun percorso attivo o target raggiunto
    }

    // Ricevi la posizione attuale del robot rispetto alla mappa
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer_->lookupTransform("map", "base_footprint", this->now(), rclcpp::Duration(100ms));
    } catch (tf2::TransformException & ex) {
      return; // Salta il ciclo se la TF ha un piccolo lag
    }

    double robot_x = transformStamped.transform.translation.x;
    double robot_y = transformStamped.transform.translation.y;

  //Blocco
  bool emergency_brake = false;
  double safe_threshold = 0.6; // Metri (Raggio robot + tolleranza)

  {
    std::lock_guard<std::mutex> lock(people_mutex_);
    for (const auto& person : current_people_) {
      double obstacle_dist = std::hypot(person.position.x - robot_x, person.position.y - robot_y);
      if (obstacle_dist < safe_threshold) {
        emergency_brake = true;
        break;
      }
    }
  }

  if (emergency_brake) {
    geometry_msgs::msg::Twist emergency_cmd;
    emergency_cmd.linear.x = 0.0;
    emergency_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(emergency_cmd);
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "FRENATA D'EMERGENZA: Maratoneta nell'area di sicurezza!");
    return; // Congela l'avanzamento fino all'allontanamento dell'ostacolo
  }

    // Estrai il prossimo punto del percorso da raggiungere
    auto target_pose = current_path_->path.poses[current_waypoint_index_].pose;
    double target_x = target_pose.position.x;
    double target_y = target_pose.position.y;

    // Calcola l'errore di distanza
    double error_x = target_x - robot_x;
    double error_y = target_y - robot_y;
    double distance = std::hypot(error_x, error_y);

    // Se siamo vicini a questo waypoint, passa al successivo
    if (distance < 0.2 && current_waypoint_index_ < current_path_->path.poses.size() - 1) {
      current_waypoint_index_++;
      return;
    }

    // Semplice controllo geometrico per orientare il robot verso il punto
    double desired_yaw = std::atan2(error_y, error_x);
    
    // Estrazione grossolana dello yaw attuale del robot dal quaternione della TF
    double q_z = transformStamped.transform.rotation.z;
    double q_w = transformStamped.transform.rotation.w;
    double current_yaw = 2.0 * std::atan2(q_z, q_w);

    double angle_error = desired_yaw - current_yaw;
    // Normalizzazione dell'angolo tra -PI e +PI
    angle_error = std::atan2(std::sin(angle_error), std::cos(angle_error));

    // Generazione del comando di velocità (Controllo Proporzionale)
    geometry_msgs::msg::Twist cmd;
    if (std::abs(angle_error) > 0.4) {
      // Se il robot è molto girato, prima ruota su se stesso
      cmd.linear.x = 0.0;
      cmd.angular.z = angle_error > 0 ? 0.3 : -0.3;
    } else {
      // Altrimenti avanza e corregge la traiettoria
      cmd.linear.x = std::min(0.2, 0.5 * distance);
      cmd.angular.z = 0.8 * angle_error;
    }

    // Se siamo arrivati all'ultimissimo punto del percorso complessivo, ferma il robot
    if (current_waypoint_index_ == current_path_->path.poses.size() - 1 && distance < 0.1) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      current_path_ = nullptr;
      RCLCPP_INFO(this->get_logger(), "Target finale raggiunto con successo!");
    }

    cmd_vel_pub_->publish(cmd);
  }
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Fra2moNavigator>();
  
  std::this_thread::sleep_for(std::chrono::seconds(2));

  node->send_target();
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}