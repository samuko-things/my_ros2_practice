// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>     // Date and time
#include <memory>     // Dynamic memory management
#include <functional>
#include <iostream> // input and output on terminal
#include <string>   // String functions

// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"

// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"



// using namespace std::placeholders;

// // create a new node class to inherit from the rclcpp::Node class
// class ListenerNode : public rclcpp::Node
// {
// public:
//     using StringMsg = std_msgs::msg::String;
//     // create the node constructor
//     ListenerNode() : Node("listener_cpp_node")
//     {
//         auto _listener_callback =
//             [this](const StringMsg::SharedPtr _msg) -> void
//         {
//             RCLCPP_INFO(rclcpp::get_logger(), "listening: %s ", _msg->data.c_str());
            
//         };

//         _listener = this->create_subscription<StringMsg>( "sample_topic", 100, _listener_callback);
//     }

// private:
//     //  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _listen;
//     std::shared_ptr<rclcpp::Subscription<StringMsg>> _listener;
// };





// int main(int argc, char *argv[])
// {
//     // Initialize ROS 2
//     rclcpp::init(argc, argv);

//     // Start the TalkerNode
//     rclcpp::spin(std::make_shared<ListenerNode>());

//     // Shutdown the node when finished
//     rclcpp::shutdown();

//     return 0;
// }





int main(int argc, char *argv[])
{
    using StringMsg = std_msgs::msg::String;
    
    rclcpp::init(argc, argv);

    // init node pointer with name
    std::shared_ptr<rclcpp::Node> _ListenerNode = nullptr; 
    _ListenerNode = std::make_shared<rclcpp::Node>("listener_cpp_node");

    //  rclcpp::Subscription<StringMsg>::SharedPtr _listen;
    std::shared_ptr<rclcpp::Subscription<StringMsg>> _listener;

    auto _listener_callback =
        [&](const StringMsg::SharedPtr _msg) -> void
    {
        RCLCPP_INFO(_ListenerNode->get_logger(), "listening: %s ", _msg->data.c_str());   
    };

    _listener = _ListenerNode->create_subscription<StringMsg>( "sample_topic", 100, _listener_callback);


    rclcpp::spin(_ListenerNode);

    rclcpp::shutdown();
    return 0;
}