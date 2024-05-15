cmake_minimum_required(VERSION 3.8)
project(line_following)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)  

# Include cpp "include" directory
include_directories(include)

# Create cpp executables
add_executable(line_following_cpp src/line_follower.cpp)
ament_target_dependencies(line_following_cpp rclcpp)

# Install cpp executables
install(TARGETS
  line_following_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install python modules
ament_python_install_package(${PROJECT_NAME})

# Install python executables
install(PROGRAMS
  line_following/line_following.py
  line_following/lsa08_due_fake.py
  line_following/pid_tuning.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install (DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
