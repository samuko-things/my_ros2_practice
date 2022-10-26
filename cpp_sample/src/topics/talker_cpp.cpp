// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <memory> // Dynamic memory management
#include <functional>
#include <iostream> // input and output on terminal
#include <string>   // String functions
#include <sstream>

// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"

// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"

// // create a new node class to inherit from the rclcpp::Node class
// class TalkerNode : public rclcpp::Node
// {
// public:
//     using StringMsg = std_msgs::msg::String;
//     // create the node constructor
//     TalkerNode() : Node("talker_cpp_node"), count(0)
//     {
//         _talker = this->create_publisher<StringMsg>("sample_topic", 100);

//         auto _talker_timer_callback =
//             [this]() -> void{
//             // where the publishing takes place
//             _message = std::make_shared<StringMsg>();
//             _message->data = "hello " + std::to_string(count);
//             RCLCPP_INFO(this->get_logger(), "talking: %s ", _message->data.c_str());
//             _talker->publish(*_message);
//             count += 1;
//         };

//         _talker_timer = this->create_wall_timer(500ms, _talker_timer_callback);
//     }

// private:

//     //  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
//     std::shared_ptr<rclcpp::Publisher<StringMsg>> _talker;

//     // rclcpp::TimerBase::SharedPtr _timer;
//     std::shared_ptr<rclcpp::TimerBase> _talker_timer;

//     // pointer used for publishing the string message
//     // std_msgs::msg::String::SharedPtr _message;
//     std::shared_ptr<StringMsg> _message;

//     size_t count;
// };

// int main(int argc, char *argv[])
// {
//     // Initialize ROS 2
//     rclcpp::init(argc, argv);

//     // Start the TalkerNode
//     rclcpp::spin(std::make_shared<TalkerNode>());

//     // Shutdown the node when finished
//     rclcpp::shutdown();

//     return 0;
// }

// int main(int argc, char *argv[])
// {
//     using namespace std::chrono_literals;
//     using StringMsg = std_msgs::msg::String;

//     rclcpp::init(argc, argv); // start ROS

//     // init node pointer with name
//     std::shared_ptr<rclcpp::Node> _TalkerNode = nullptr;
//     _TalkerNode = std::make_shared<rclcpp::Node>("talker_cpp_node");

//     //  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
//     std::shared_ptr<rclcpp::Publisher<StringMsg>> _talker;

//     // rclcpp::TimerBase::SharedPtr _timer;
//     std::shared_ptr<rclcpp::TimerBase> _talker_timer;

//     // pointer used for publishing the string message
//     // std_msgs::msg::String::SharedPtr _message;
//     std::shared_ptr<StringMsg> _message;
//     size_t count;

//     _talker = _TalkerNode->create_publisher<StringMsg>("sample_topic", 100);

//     auto _talker_timer_callback =
//         [&]() -> void{
//         // where the publishing takes place
//         _message = std::make_shared<StringMsg>();
//         _message->data = "hello " + std::to_string(count);
//         RCLCPP_INFO(_TalkerNode->get_logger(), "talking: %s ", _message->data.c_str());
//         _talker->publish(*_message);
//         count += 1;
//     };

//     _talker_timer = _TalkerNode->create_wall_timer(500ms, _talker_timer_callback);

//     rclcpp::spin(_TalkerNode);

//     rclcpp::shutdown();
//     return 0;
// }


int main(int argc, char *argv[])
{
    using namespace std::chrono_literals;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("talker_cpp_node");
    auto publisher = node->create_publisher<std_msgs::msg::String>("topic", 10);
    std_msgs::msg::String message;
    auto publish_count = 0;
    rclcpp::WallRate loop_rate(500ms);

    while (rclcpp::ok())
    {
        message.data = "Hello, world! ";
        std::stringstream ss;
        ss << message.data.c_str() << publish_count++ << std::endl;
        RCLCPP_INFO(node->get_logger(), ss.str().c_str());
        try
        {
            publisher->publish(message);
            rclcpp::spin_some(node);
        }
        catch (const rclcpp::exceptions::RCLError &e)
        {
            RCLCPP_ERROR(
                node->get_logger(),
                "unexpectedly failed with %s",
                e.what());
        }
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
