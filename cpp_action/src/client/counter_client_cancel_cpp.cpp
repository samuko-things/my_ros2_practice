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

#include <chrono>
#include <cinttypes>
#include <iostream> // input and output on terminal
#include <string>   // String functions

#include "example_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sample_actions/action/counter.hpp"

using CounterAction = sample_actions::action::Counter;
using GoalHandleCounterAction = rclcpp_action::ClientGoalHandle<CounterAction>;

rclcpp::Node::SharedPtr _node = nullptr;





void feedback_callback(
  GoalHandleCounterAction::SharedPtr,
  const std::shared_ptr<const CounterAction::Feedback> feedback)
{
//   RCLCPP_INFO(
//     _node->get_logger(),
//     "feedback: %d",
//     feedback->current_count);
    std::cout << "feedback_count: " << feedback->current_count << std::endl;
}


int main(int argc, char **argv)
{
    using namespace std::chrono_literals;
    std::stringstream ss;

    rclcpp::init(argc, argv);
    _node = rclcpp::Node::make_shared("counter_action_client_node");
    auto _counter_action_client = rclcpp_action::create_client<CounterAction>(_node, "counter_action_topic");

    // check command line input
    if (argc != 2)
    {
        RCLCPP_INFO(_node->get_logger(), "usage: enter counter value");
        return 1;
    }

    // wait 20 seconds for the action server node by checking if the "counter_action_topic" is available
    while (!_counter_action_client->wait_for_action_server(2s))
    {
        // rclcpp::ok() returns true if shutdown has been called, false otherwise
        //  negating it means when shutdown is called it will exit the whole program
        if (!rclcpp::ok())
        {
            RCLCPP_INFO(_node->get_logger(), "interrupted while waiting for counter action service");
            return 0;
        }
        RCLCPP_INFO(_node->get_logger(), "counter action service not available, waiting again ...");
    }

    // populate goal with the command line input and send it
    auto goal_msg = CounterAction::Goal();
    goal_msg.no_of_count = std::stoi(argv[1]);

    RCLCPP_INFO(_node->get_logger(), "Sending goal...");

    // add feed back
    auto send_goal_options = rclcpp_action::Client<CounterAction>::SendGoalOptions();
    send_goal_options.feedback_callback = feedback_callback;
  

    // Ask server to achieve some goal and wait until it's accepted
    // auto goal_handle_future = _counter_action_client->async_send_goal(goal_msg);
    auto goal_handle_future = _counter_action_client->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(_node, goal_handle_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(_node->get_logger(), "send goal call failed :(");
        return 1;
    }

    // check if goal was rejected by the handle_goal() server function
    GoalHandleCounterAction::SharedPtr _goal_handle = goal_handle_future.get();
    if (!_goal_handle)
    {
        RCLCPP_ERROR(_node->get_logger(), "Goal was rejected by server");
        return 1;
    }

    // Wait for the server to be done with the goal
    // auto result_future = _counter_action_client->async_get_result(_goal_handle);

    // RCLCPP_INFO(_node->get_logger(), "Waiting for result");
    // if (rclcpp::spin_until_future_complete(_node, result_future) !=
    //     rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     //action fails due to server node being shutdown unexpectedly during execution
    //     RCLCPP_ERROR(_node->get_logger(), "get result call failed :(");
    //     return 1;
    // }



    // Wait for the server to be done with the goal
    RCLCPP_INFO(_node->get_logger(), "Waiting for result");
    auto result_future = _counter_action_client->async_get_result(_goal_handle);

    auto wait_result = rclcpp::spin_until_future_complete(_node, result_future, 10s); // wait for 10sec before cancelling

    if (rclcpp::FutureReturnCode::TIMEOUT == wait_result) {
        RCLCPP_INFO(_node->get_logger(), "canceling goal");
        // Cancel the goal since it is taking too long
        auto cancel_result_future = _counter_action_client->async_cancel_goal(_goal_handle);
        if (rclcpp::spin_until_future_complete(_node, cancel_result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
        {
        RCLCPP_ERROR(_node->get_logger(), "failed to cancel goal");
        rclcpp::shutdown();
        return 1;
        }
        RCLCPP_INFO(_node->get_logger(), "goal is being canceled");
    } else if (rclcpp::FutureReturnCode::SUCCESS != wait_result) {
        RCLCPP_ERROR(_node->get_logger(), "failed to get result");
        rclcpp::shutdown();
        return 1;
    }


        RCLCPP_INFO(_node->get_logger(), "Waiting for result");
    if (rclcpp::spin_until_future_complete(_node, result_future) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(_node->get_logger(), "get result call failed :(");
        return 1;
    }


    GoalHandleCounterAction::WrappedResult wrapped_result = result_future.get();

    switch (wrapped_result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        std::cout<<"counter action completed. result = "<<wrapped_result.result->final_count << std::endl;
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(_node->get_logger(), "Goal was aborted");
        return 1;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(_node->get_logger(), "Goal was canceled");
        RCLCPP_ERROR(_node->get_logger(), "result = %d", wrapped_result.result->final_count);
        return 1;
    default:
        RCLCPP_ERROR(_node->get_logger(), "Unknown result code");
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
