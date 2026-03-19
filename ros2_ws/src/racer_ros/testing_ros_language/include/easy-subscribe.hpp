#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
class string_subscriber: public rclcpp::Node{
    public:
    string_subscriber(): Node("listener"){
        auto executor_callback = [this](std_msgs::msg::String::UniquePtr msg)->void{
            RCLCPP_INFO(this->get_logger(), "I hear....", msg->data.c_str());
        };
        subscriber_object = this->create_subscription<std_msgs::msg::String>("personal_topic", 10, executor_callback);
    }
    private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_object;  
};