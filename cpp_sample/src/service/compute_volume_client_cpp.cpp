// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono> // Date and time
#include <memory> // Dynamic memory management
#include <functional>
#include <iostream> // input and output on terminal
#include <string>   // String functions

// ROS Client Library for C++
// Allows use of the most common elements of ROS 2
#include "rclcpp/rclcpp.hpp"

// Built-in message type that will be used to publish data
#include "std_msgs/msg/string.hpp"
#include "sample_srvs/srv/compute_volume.hpp"



int main(int argc, char *argv[])
{
    using namespace std::chrono_literals;
    using ComputeVolumeSrv = sample_srvs::srv::ComputeVolume;

    rclcpp::init(argc, argv);

    if (argc != 4)
    {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
        std::cout << "usage: add three float values X Y Z" << std::endl;
        return 1;
    }

    // init node pointer with name
    std::shared_ptr<rclcpp::Node> _ComputeVolumeClientNode = nullptr; 
    _ComputeVolumeClientNode = std::make_shared<rclcpp::Node>("compute_volume_client_cpp_node");

    std::shared_ptr<rclcpp::Client<ComputeVolumeSrv>> _client = nullptr; // client pointer
    _client = _ComputeVolumeClientNode->create_client<ComputeVolumeSrv>("compute_volume");

    // wait_for_service() returns false if no service is seen after the spcified time
    // and true if it sees a service within the time it is waiting (i.e before the timeout)
    // negating it means the loop will continue to run until service is seen
    // and when a service is seen it will exit the while loop and continue execution
    while (!_client->wait_for_service(2s))
    {
        // rclcpp::ok() returns true if shutdown has been called, false otherwise
        //  negating it means when shutdown is called it will exit the whole program
        if (!rclcpp::ok())
        {
            std::cout << "interrupted while waiting for service" << std::endl;
            return 0;
        }
        std::cout << _client->get_service_name() <<" not available, waiting again ..." << std::endl;
    }

    // receive command line request
    auto _request = std::make_shared<ComputeVolumeSrv::Request>();
    _request->x = std::stof(argv[1]);
    _request->y = std::stof(argv[2]);
    _request->z = std::stof(argv[3]);

    // send request to service server
    auto _response = _client->async_send_request(_request);

    // Wait for the response.
    if (rclcpp::spin_until_future_complete(_ComputeVolumeClientNode, _response) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(_ComputeVolumeClientNode->get_logger(), "Volume = %f", _response.get()->volume);
    }
    else
    {
        RCLCPP_ERROR(_ComputeVolumeClientNode->get_logger(), "Failed to call service");
    }

    rclcpp::shutdown();
    return 0;
}