cmake_minimum_required(VERSION 3.8)
project(db_tsdf)

cmake_policy(SET CMP0074 NEW)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS  OFF) 
add_compile_options(-O3 -march=native -mtune=native -mavx -mavx2 -mfma -fopenmp-simd)
add_compile_options(-Wall -Wextra -Wno-unused-parameter -Wno-sign-compare)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

find_package(VTK REQUIRED COMPONENTS 
  CommonCore
  CommonDataModel
  FiltersCore
  IOXML
  ImagingCore
  ImagingGeneral
  ImagingMorphological
)

add_executable(db_tsdf_node  src/db_tsdf_node.cpp)

ament_target_dependencies(db_tsdf_node
  rclcpp
  tf2            tf2_ros
  tf2_geometry_msgs
  sensor_msgs    nav_msgs
  pcl_ros        pcl_conversions
  std_srvs
  Boost
)

target_link_libraries(db_tsdf_node
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  OpenMP::OpenMP_CXX
  VTK::CommonCore
  VTK::CommonDataModel
  VTK::FiltersCore
  VTK::IOXML
  VTK::ImagingCore
  VTK::ImagingGeneral
)

target_include_directories(db_tsdf_node PRIVATE
  include
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

install(DIRECTORY 
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS
  db_tsdf_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()