#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <sensor_msgs/msg/joy.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "robomas_plugins/msg/robomas_target.hpp"
#include "robomas_plugins/msg/robomas_frame.hpp"
#include "shoko/shoko_utils.hpp"

using std::placeholders::_1;

class Shoko : public rclcpp::Node
{
private:
  void controller_callback(const sensor_msgs::msg::Joy & msg) const
  {
    if(msg.buttons[7]){
      shoko_setting_->publish(shoko::robomas_utils::to_velocity_mode(3));
    }
    if(msg.buttons[6]){
      shoko_setting_->publish(shoko::robomas_utils::to_disable_mode(3));
    }

    constexpr float Velocity = 400;

    auto message = robomas_plugins::msg::RobomasTarget{};
    if(msg.buttons[1] == 1 && msg.buttons[0] == 1){
      message.target = 0;
    }
    else if(msg.buttons[1] == 1 && msg.buttons[0] == 0){
      message.target = Velocity;
    }
    else if(msg.buttons[1] == 0 && msg.buttons[0] == 1){
      message.target = -Velocity;
    }
    else{
      message.target = 0;
    } 

    shoko_->publish(message);
  } 
  

public:
  Shoko()
  : Node("shoko")
  {
    this->controller_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&Shoko::controller_callback, this, std::placeholders::_1));
    this->shoko_ = this->create_publisher<robomas_plugins::msg::RobomasTarget>("robomas_target4", 10);
    this->shoko_setting_  = this->create_publisher<robomas_plugins::msg::RobomasFrame>("robomas_frame", 10);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasTarget>::SharedPtr shoko_;
  rclcpp::Publisher<robomas_plugins::msg::RobomasFrame>::SharedPtr shoko_setting_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Shoko>());
  rclcpp::shutdown();
  return 0;
}