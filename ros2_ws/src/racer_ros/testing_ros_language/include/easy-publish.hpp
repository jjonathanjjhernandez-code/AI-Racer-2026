#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <memory>
#include <chrono>
using namespace std::chrono_literals;
class string_publisher: public rclcpp::Node{
    public:
    string_publisher() : Node("talker"){
        publisher_object = this->create_publisher<std_msgs::msg::String>("personal_topic", 10);
        auto executor_callback = [this]()->void{
        auto message = std_msgs::msg::String();
        message.data = "Hello World!";
        RCLCPP_INFO(this->get_logger(), "Publishing this message: ", message.data.c_str());
        this->publisher_object->publish(message);
    }; 
    timer = this->create_wall_timer(500ms, executor_callback);
    }
    private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_object;
};