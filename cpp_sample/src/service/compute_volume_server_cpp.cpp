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
#include "sample_srvs/srv/compute_volume.hpp"



int main(int argc, char *argv[])
{
    using ComputeVolumeSrv = sample_srvs::srv::ComputeVolume;

    rclcpp::init(argc, argv); // start ROS2

    // create and init node pointer with name
    std::shared_ptr<rclcpp::Node> _ComputeVolumeServerNode; 
    _ComputeVolumeServerNode = std::make_shared<rclcpp::Node>("compute_volume_server_cpp_node");

    std::shared_ptr<rclcpp::Service<ComputeVolumeSrv>> _server; // server pointer

    auto compute_volume_callback = 
        [&](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<ComputeVolumeSrv::Request> _request,
        std::shared_ptr<ComputeVolumeSrv::Response> _response) -> void 
    {
        (void)request_header;
        _response->volume = _request->x * _request->y * _request->z;
        RCLCPP_INFO(_ComputeVolumeServerNode->get_logger(), "incomming messages: %f x %f x %f", _request->x, _request->y, _request->z);
    };

    _server = _ComputeVolumeServerNode->create_service<ComputeVolumeSrv>("compute_volume", compute_volume_callback);


    rclcpp::spin(_ComputeVolumeServerNode);

    rclcpp::shutdown();
    return 0;
}