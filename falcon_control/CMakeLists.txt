cmake_minimum_required(VERSION 3.5)
project(falcon_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#find_package(libnifalcon REQUIRED)

include_directories(${LIBNIFALCON_INCLUDE_DIR})
#find_package(libnifalcon REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Boost REQUIRED)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
include_directories(${BOOST_INCLUDE_DIRECTORIES})
#...

find_package(falcon_interfaces REQUIRED) 

find_library(NIFALCON_LIBRARY nifalcon)


#ament_export_include_directories(${include_dirs})
#ament_export_libraries(${PROJECT_NAME})
#add_executable(talker src/publisher_member_function.cpp)
#ament_target_dependencies(talker rclcpp std_msgs)

#add_executable(listener src/subscriber_member_function.cpp)
#ament_target_dependencies((listener rclcpp std_msgs)




add_executable(falconpub src/falcon_pub.cpp)
ament_target_dependencies(falconpub rclcpp falcon_interfaces)
target_link_libraries(falconpub ${NIFALCON_LIBRARY})    


install(TARGETS
  falconpub
  DESTINATION lib/${PROJECT_NAME})

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
# install(PROGRAMS
#   src/falcon_sub.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
