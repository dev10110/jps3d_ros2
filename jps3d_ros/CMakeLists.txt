cmake_minimum_required(VERSION 3.8)
project(jps3d_ros)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to release mode
if (NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(jps3d REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nvblox_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dasc_msgs REQUIRED)
find_package(tf2 REQUIRED)

if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)




add_executable(jps_planner_2d src/jps_planner_2d.cpp)
ament_target_dependencies(jps_planner_2d 
	rclcpp
	nvblox_msgs
	geometry_msgs
  message_filters
  tf2
  tf2_geometry_msgs
  tf2_ros
  nav_msgs
  dasc_msgs
	)
if(EXISTS ${TF2_CPP_HEADERS})
  target_compile_definitions(jps_planner_2d PUBLIC -DTF2_CPP_HEADERS)
endif()

target_include_directories(jps_planner_2d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${JPS3D_INCLUDE_DIRS}
  ${EIGEN3D_INCLUDE_DIRS})
target_link_libraries(jps_planner_2d ${JPS3D_LIBRARIES} Eigen3::Eigen yaml-cpp ${Boost_LIBRARIES})

install(TARGETS 
	jps_planner_2d
  DESTINATION lib/${PROJECT_NAME})



## interpolant lib
add_executable(test_interpolate src/test_interpolate.cpp)
target_include_directories(test_interpolate PUBLIC include ${EIGEN3D_INCLUDE_DIRS})
target_link_libraries(test_interpolate Eigen3::Eigen)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
