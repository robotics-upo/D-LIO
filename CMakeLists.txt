set(CMAKE_CXX_STANDARD 20)
cmake_minimum_required(VERSION 3.8)

project(dlio)

add_executable(dlo3d_node src/dlo3d_node.cpp)

target_compile_options(dlo3d_node PRIVATE 
  -O3 
  -march=native 
  -mtune=native
  -mavx
  -mavx2
  -mfma 
  -fopenmp-simd
  -lpthread
)
target_compile_definitions(dlo3d_node PRIVATE
  NDEBUG
  EIGEN_NO_DEBUG
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(Boost REQUIRED COMPONENTS thread chrono)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)
find_library(ANN_LIB ANN PATHS /usr/lib /usr/local/lib)

ament_target_dependencies(dlo3d_node
  rclcpp
  message_filters
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  std_srvs
  OpenMP
)

target_link_libraries(dlo3d_node
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${glog_LIBRARIES}
  ${CERES_LIBRARIES}
  ${ANN_LIB}
  OpenMP::OpenMP_CXX
)

target_include_directories(dlo3d_node PRIVATE
  include
  /usr/local/include
  ${PCL_INCLUDE_DIRS}
)


#########################################3

install(TARGETS
  dlo3d_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()



