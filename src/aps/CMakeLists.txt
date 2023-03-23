cmake_minimum_required(VERSION 3.8)
project(aps)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(PkgConfig)
pkg_check_modules(LIBREALSENSE REQUIRED realsense2)

ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}
  src/aps.cpp
)

target_link_libraries(${PROJECT_NAME} ${LIBREALSENSE_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${LIBREALSENSE_INCLUDE_DIRS})

ament_auto_package()
