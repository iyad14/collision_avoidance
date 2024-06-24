cmake_minimum_required(VERSION 3.8)
project(collision_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(CUDA REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Compiling CUDA headers
set(CUDA_NVCC_FLAGS "-arch=sm_75" CACHE STRING "nvcc flags" FORCE)
set (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
set(LIB_TYPE STATIC) 
cuda_add_library(cudaLib ${LIB_TYPE} lib/cuda_libs.cu)

# Importing libraries
add_library( clusterlib SHARED IMPORTED )
set_property( TARGET clusterlib PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86/libcudacluster-x86.so)

add_executable(collision_avoidance src/CollisionAvoidance.cpp)
ament_target_dependencies(collision_avoidance 
  rclcpp
  sensor_msgs
  tf2_ros
  tf2_eigen
  Eigen3
  cv_bridge
  PCL
  pcl_conversions
  pcl_ros
)

target_include_directories(collision_avoidance 
    PUBLIC 
    ${EIGEN3_INCLUDE_DIRS}
    )
target_link_libraries(collision_avoidance 
  Eigen3::Eigen 
  cudaLib 
  clusterlib)

# Install targets
install(TARGETS
  collision_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
