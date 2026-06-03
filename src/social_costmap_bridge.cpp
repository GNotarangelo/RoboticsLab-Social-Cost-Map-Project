#include <memory>
#include <string>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/person.hpp"

class SocialCostmapBridge : public rclcpp::Node {
public:
    SocialCostmapBridge() : Node("social_costmap_bridge") {
        people_pub_ = this->create_publisher<people_msgs::msg::People>("/people", 10);

        tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/world/social_cost_map_world/pose/info", 10,
            std::bind(&SocialCostmapBridge::pose_callback, this, std::placeholders::_1));

        timer_ = rclcpp::create_timer(
            this,
            this->get_clock(),
            std::chrono::milliseconds(50),
            std::bind(&SocialCostmapBridge::update_dynamic_obstacles, this)
        );

        RCLCPP_INFO(this->get_logger(), "Social Costmap Bridge avviato correttamente.");
    }

private:
    // 1. Gestione dei modelli reali tracciati da Gazebo (Bambino, Meccanico, Tifoso)
    void pose_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_detected_people_.clear();

        for (const auto & transform : msg->transforms) {
            std::string name = transform.child_frame_id;

            if (name == "Bambino" || name == "Meccanico" || name == "Tifoso_Accanito_Fisico") {
                people_msgs::msg::Person person;
                person.name = name;
                person.position.x = transform.transform.translation.x;
                person.position.y = transform.transform.translation.y;
                person.position.z = transform.transform.translation.z;
                person.velocity.x = 0.0;
                person.velocity.y = 0.0;
                current_detected_people_.push_back(person);
            }
        }
    }

    // 2. Unione dei dati e generazione della traiettoria matematica del Maratoneta
    void update_dynamic_obstacles() {
        people_msgs::msg::People final_people_msg;
        final_people_msg.header.stamp = this->get_clock()->now();
        final_people_msg.header.frame_id = "map";

        /* lock_guard distrugge il lucchetto appena usciti dalle parentesi graffe
        che ne contengono la dichiarazione. Aggiungendole qui, evitiamo di bloccare
        pose_callback per più tempo di quello necessario ad evitare una race condition. */
        {
            std::lock_guard<std::mutex> lock(mutex_);
            final_people_msg.people = current_detected_people_;
        }

        // Simuliamo la posizione del maratoneta in quanto gli actor non pubblicano TF
        // (Meglio, non sono riuscito a fargliera pubblicare)
        double sim_time = this->get_clock()->now().seconds();
        double t = std::fmod(sim_time, 18.0);
        
        people_msgs::msg::Person maratoneta;
        maratoneta.name = "Maratoneta";
        maratoneta.position.y = 0.5;
        maratoneta.position.z = 0.9;

        if (t >= 0.0 && t < 9.0) {
            //Andata
            maratoneta.position.x = -8.0 + (4.0 / 9.0) * t;
            maratoneta.velocity.x = 4.0 / 9.0;
        } else {
            //Ritorno
            maratoneta.position.x = -4.0 - (4.0 / 9.0) * (t - 9.0);
            maratoneta.velocity.x = -(4.0 / 9.0);
        }
        maratoneta.velocity.y = 0.0;

        // Aggiungi il maratoneta dinamico al vettore globale
        final_people_msg.people.push_back(maratoneta);

        // Pubblica tutto il set di manichini alla costmap sociale
        people_pub_->publish(final_people_msg);
    }

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
    rclcpp::Publisher<people_msgs::msg::People>::SharedPtr people_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<people_msgs::msg::Person> current_detected_people_;
    std::mutex mutex_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocialCostmapBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}