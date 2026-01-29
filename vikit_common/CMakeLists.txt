cmake_minimum_required(VERSION 3.8)
project(vikit_common)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
if(DEFINED ENV{ARM_ARCHITECTURE})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -march=armv7-a")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")
find_package(Sophus REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

# Set source files
set(SOURCEFILES
  src/atan_camera.cpp
  src/omni_camera.cpp
  src/math_utils.cpp
  src/vision.cpp
  src/performance_monitor.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/pinhole_camera.cpp
  src/homography.cpp
  src/img_align.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
)

# Add dependencies
ament_target_dependencies(${PROJECT_NAME} rclcpp)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

# Tests
add_executable(test_vk_common_camera test/test_camera.cpp)
target_link_libraries(test_vk_common_camera ${PROJECT_NAME})

add_executable(test_vk_common_triangulation test/test_triangulation.cpp)
target_link_libraries(test_vk_common_triangulation ${PROJECT_NAME})

add_executable(test_vk_common_patch_score test/test_patch_score.cpp)
target_link_libraries(test_vk_common_patch_score ${PROJECT_NAME})

ament_package()