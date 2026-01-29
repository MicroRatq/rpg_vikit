cmake_minimum_required(VERSION 3.8)
project(vikit_ros)

# Set build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Set build flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vikit_common REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
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
  src/output_helper.cpp
)

# Create library
add_library(${PROJECT_NAME} SHARED ${SOURCEFILES})

# Link libraries
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
)

# Add dependencies
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  visualization_msgs 
  tf2_ros
  vikit_common)

# Export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp visualization_msgs tf2_ros vikit_common)

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

ament_package()