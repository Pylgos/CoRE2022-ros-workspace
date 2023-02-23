cmake_minimum_required(VERSION 3.8)
project(robot_interface_proxy)

set(CMAKE_BUILD_TYPE Debug)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(socketcan_cpp REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME}
  SHARED
    src/proxy_base.cpp
)


ament_auto_add_executable(can_proxy
  src/can_proxy.cpp
)

target_link_libraries(can_proxy ::socketcan_cpp)


ament_auto_package()
