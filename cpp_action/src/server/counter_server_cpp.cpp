// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sample_actions/action/counter.hpp"

// class CounterActionServer : public rclcpp::Node
// {
// public:
//     using CounterAction = sample_actions::action::Counter;
//     using GoalHandleCounterAction = rclcpp_action::ServerGoalHandle<CounterAction>;

//     explicit CounterActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
//         : Node("counter_action_server_node", options)
//     {
//         using namespace std::placeholders;

//         action_server = rclcpp_action::create_server<CounterAction>(
//             this,
//             "counter_action",
//             std::bind(&CounterActionServer::handle_goal, this, _1, _2),
//             std::bind(&CounterActionServer::handle_cancel, this, _1),
//             std::bind(&CounterActionServer::handle_accepted, this, _1));
//     }

// private:
//     rclcpp_action::Server<CounterAction>::SharedPtr action_server;

//     // handles receiving goal request from the action client
//     rclcpp_action::GoalResponse handle_goal(
//         const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CounterAction::Goal> goal)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("counter action server"), "Got goal request with order %d", goal->no_of_count);
//         (void)uuid;
//         // Let's reject conter goal of more than 500
//         if (goal->no_of_count > 500)
//         {
//             return rclcpp_action::GoalResponse::REJECT;
//         }
//         return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//     }

//     // handles request to cancel action from action client
//     rclcpp_action::CancelResponse handle_cancel(
//         const std::shared_ptr<GoalHandleCounterAction> goal_handle)
//     {
//         RCLCPP_INFO(rclcpp::get_logger("counter action server"), "Got request to cancel goal");
//         (void)goal_handle;
//         return rclcpp_action::CancelResponse::ACCEPT;
//     }

//     void execute(const std::shared_ptr<GoalHandleCounterAction> goal_handle)
//     {
//         using namespace std::chrono_literals;

//         RCLCPP_INFO(this->get_logger(), "Executing goal....");
//         rclcpp::Rate loop_rate(500ms);

//         // create and initalize goal feedback and result
//         const auto goal = goal_handle->get_goal();
//         auto feedback = std::make_shared<CounterAction::Feedback>();
//         auto result = std::make_shared<CounterAction::Result>();

//         std::stringstream ss;

//         // perform action process until goal is achieved or cancelled while publishing feedback
//         int counter = 0;
//         for (int i = 0; (i < goal->no_of_count) && rclcpp::ok(); i += 1)
//         {
//             // Check if there is a cancel request
//             if (goal_handle->is_canceling())
//             {
//                 result->final_count = counter;
//                 goal_handle->canceled(result);

//                 ss << "action canelled. result = " << result->final_count << std::endl;
//                 RCLCPP_INFO(this->get_logger(), ss.str().c_str());

//                 return; // exit the execute function
//             }
//             // Update counter and feedback
//             counter += 1;
//             feedback->current_count = counter;
//             // Publish feedback
//             goal_handle->publish_feedback(feedback);
//             ss << "action in progress. feedback = " << feedback->current_count << std::endl;
//             RCLCPP_INFO(this->get_logger(), ss.str().c_str());

//             loop_rate.sleep();
//         }

//         RCLCPP_INFO(this->get_logger(), "about to check result");
//         // Check if goal is done and update result
//         if (rclcpp::ok())
//         {
//             result->final_count = counter;
//             goal_handle->succeed(result);
//             ss << "action completed. result = " << result->final_count << std::endl;
//             RCLCPP_INFO(this->get_logger(), ss.str().c_str());
//         }
//     }

//     // this runs if the handle goal function returns ACCEPT AND EXECUTE
//     void handle_accepted(const std::shared_ptr<GoalHandleCounterAction> goal_handle)
//     {
//         // this needs to return quickly to avoid blocking the executor, so spin up a new thread
//         std::thread{execute, goal_handle}.detach();
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto counter_action_server = std::make_shared<CounterActionServer>();
//     rclcpp::spin(counter_action_server);
//     rclcpp::shutdown();
//     return 0;
// }








using CounterAction = sample_actions::action::Counter;
using GoalHandleCounterAction = rclcpp_action::ServerGoalHandle<CounterAction>;


/*FUNCTION DECLARATION*/

// handles receiving goal request from the action client
rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CounterAction::Goal> goal);

// handles request to cancel action from action client
rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCounterAction> goal_handle);

void execute(const std::shared_ptr<GoalHandleCounterAction> goal_handle);

// this runs the execute function if the handle_goal function returns ACCEPT AND EXECUTE
void handle_accepted(const std::shared_ptr<GoalHandleCounterAction> goal_handle);



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("counter_action_server_node");

    // Create an action server with three callbacks
    auto action_server = rclcpp_action::create_server<CounterAction>(
        node,
        "counter_action_topic",
        handle_goal,
        handle_cancel,
        handle_accepted);

    std::cout<<"waiting for action client to send goal..." << std::endl;
    
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}



/*FUNCTION DEFINITION*/

// handles receiving goal request from the action client
rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CounterAction::Goal> goal)
{
    RCLCPP_INFO(rclcpp::get_logger("counter action server"), "Got goal request with order %d", goal->no_of_count);
    (void)uuid;
    // Let's reject conter goal of more than 50
    if (goal->no_of_count > 50)
    {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// handles request to cancel action from action client
rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCounterAction> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("counter action server"), "Got request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(const std::shared_ptr<GoalHandleCounterAction> goal_handle)
{
    using namespace std::chrono_literals;
    rclcpp::WallRate loop_rate(500ms);
    
    RCLCPP_INFO(rclcpp::get_logger("counter server action"), "Executing goal");

    // create and initalize goal feedback and result
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<CounterAction::Feedback>();
    auto result = std::make_shared<CounterAction::Result>();

    std::stringstream ss;

    // perform action process until goal is achieved or cancelled while publishing feedback
    int counter = 0;
    while ((counter < goal->no_of_count) && rclcpp::ok())
    {
        // Check if there is a cancel request
        if (goal_handle->is_canceling())
        {
            result->final_count = counter;
            goal_handle->canceled(result);

            RCLCPP_ERROR(rclcpp::get_logger("counter action server"), "Goal was canceled");
            RCLCPP_ERROR(rclcpp::get_logger("counter action server"), "reselt = %d", result->final_count);
            std::cout<<"waiting for action client to send goal..." << std::endl;
            return; //exit the execute function
        }
        // Update counter and feedback
        counter += 1;
        feedback->current_count = counter;
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        // std::cout << "feedback_count: " << feedback->current_count << std::endl;
        // RCLCPP_INFO(rclcpp::get_logger("counter action server"), "%d", feedback->current_count);

        loop_rate.sleep();
    }

    // RCLCPP_INFO(rclcpp::get_logger("counter action server"), "about to check result");
    // Check if goal is done and update result
    if (rclcpp::ok())
    {
        result->final_count = counter;
        goal_handle->succeed(result);
        std::cout<<"counter action completed. result = "<<result->final_count << std::endl;
        // RCLCPP_INFO(rclcpp::get_logger("counter action server"), ss.str().c_str());
    }

    std::cout<<"waiting for action client to send goal..." << std::endl;
}

// this runs if the handle goal function returns ACCEPT AND EXECUTE
void handle_accepted(const std::shared_ptr<GoalHandleCounterAction> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{execute, goal_handle}.detach();
}