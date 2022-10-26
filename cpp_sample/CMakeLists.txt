cmake_minimum_required(VERSION 3.8)
project(cpp_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ ADD DEPENDENCIES #########################
# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sample_srvs REQUIRED)             
###########################################################



############# CPP FILES ##################################
# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(talker_cpp src/topics/talker_cpp.cpp)
ament_target_dependencies(talker_cpp rclcpp std_msgs)

add_executable(listener_cpp src/topics/listener_cpp.cpp)
ament_target_dependencies(listener_cpp rclcpp std_msgs)

add_executable(compute_volume_server_cpp src/service/compute_volume_server_cpp.cpp)
ament_target_dependencies(compute_volume_server_cpp rclcpp std_msgs sample_srvs)

add_executable(compute_volume_client_cpp src/service/compute_volume_client_cpp.cpp)
ament_target_dependencies(compute_volume_client_cpp rclcpp std_msgs sample_srvs)


# Install Cpp executables
install(TARGETS
  talker_cpp
  listener_cpp
  compute_volume_server_cpp
  compute_volume_client_cpp
  DESTINATION lib/${PROJECT_NAME}
)
###################################################################




# ################ PYTHON FILES for ament_cmake_python ##############
# # Install Python modules
# ament_python_install_package(${PROJECT_NAME})
 
# # Install Python executables
# install(PROGRAMS
#   scripts/listener.py       scripts/talker.py
#   DESTINATION lib/${PROJECT_NAME}
# )
# ###################################################################



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
