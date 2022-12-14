cmake_minimum_required(VERSION 3.8)
project(cpp_action)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sample_actions REQUIRED)


############# CPP FILES ##################################
# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(counter_server_cpp src/server/counter_server_cpp.cpp)
ament_target_dependencies(counter_server_cpp
                         rclcpp rclcpp_action rclcpp_components sample_actions)

add_executable(counter_client_cpp src/client/counter_client_cpp.cpp)
ament_target_dependencies(counter_client_cpp
                        rclcpp rclcpp_action rclcpp_components sample_actions)

add_executable(counter_client_cancel_cpp src/client/counter_client_cancel_cpp.cpp)
ament_target_dependencies(counter_client_cancel_cpp
                        rclcpp rclcpp_action rclcpp_components sample_actions)


# Install Cpp executables
install(TARGETS
  counter_server_cpp
  counter_client_cpp
  counter_client_cancel_cpp
  DESTINATION lib/${PROJECT_NAME}
)
###################################################################


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
