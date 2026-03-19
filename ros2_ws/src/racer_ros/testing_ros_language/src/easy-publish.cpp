#include "easy-publish.hpp"
int main(int argc, char *argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<string_publisher>());
    rclcpp::shutdown();
    return 0;
}