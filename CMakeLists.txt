cmake_minimum_required(VERSION 3.10)
project(lane_change_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(LAPACK REQUIRED)

# ── CasADi (external project) ──
set(VENDOR_PATH "${CMAKE_BINARY_DIR}/vendor")
set(CASADI_URL https://github.com/casadi/casadi/releases/download/3.6.3/casadi-3.6.3.tar.gz)

include(ExternalProject)
ExternalProject_Add(
  lane_change_sim_casadi_external
  URL ${CASADI_URL}
  PREFIX ${VENDOR_PATH}
  UPDATE_COMMAND ""
  CMAKE_ARGS
  -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
  -DCMAKE_PREFIX_PATH=<INSTALL_DIR>
  -DWITH_EXAMPLES=OFF
  -DWITH_IPOPT=ON
  -DWITH_BUILD_REQUIRED=ON
  -DWITH_BUILD_HSL=ON
  -DWITH_HSL=ON
  -DBUILD_HSL_SRC=${PROJECT_SOURCE_DIR}/ThirdParty/coinhsl-archive-2021.05.05
)

file(MAKE_DIRECTORY "${VENDOR_PATH}/include")
add_library(lane_change_sim_casadi SHARED IMPORTED)
add_dependencies(lane_change_sim_casadi lane_change_sim_casadi_external)
set_target_properties(lane_change_sim_casadi PROPERTIES
  IMPORTED_LOCATION "${VENDOR_PATH}/lib/${CMAKE_SHARED_LIBRARY_PREFIX}casadi${CMAKE_SHARED_LIBRARY_SUFFIX}"
  IMPORTED_IMPLIB "${VENDOR_PATH}/lib/${CMAKE_IMPORT_LIBRARY_PREFIX}casadi${CMAKE_IMPORT_LIBRARY_SUFFIX}"
  INTERFACE_INCLUDE_DIRECTORIES "${VENDOR_PATH}/include"
)

# ── ROS2 dependencies ──
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── HSL (optional) ──
option(WITH_HSL "Compile solver with coinhsl" ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(HSL)
if(NOT ${HSL_FOUND})
  set(WITH_HSL OFF)
endif()

if(WITH_HSL)
  add_definitions(-DWITH_HSL)
endif()

# ── Interface generation (msg / srv / action) ──
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/VehicleParam.msg"
  "msg/VehicleState.msg"
  "msg/Vehicle.msg"
  "msg/Lane.msg"
  "msg/LaneArray.msg"
  "msg/LaneNet.msg"
  "msg/VehicleArray.msg"
  "msg/VehicleSet.msg"
  "msg/ArenaInfoStatic.msg"
  "msg/ArenaInfoDynamic.msg"
  "msg/BehaviorDecision.msg"
  "msg/TrajectoryPoint.msg"
  "msg/Trajectory.msg"
  "srv/ResetScenario.srv"
  "action/RunEpisode.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# ── Compiler definitions ──
add_compile_definitions(HAS_CASADI)

# ── Include directories ──
include_directories(
  include
  ${VENDOR_PATH}/include
)

# ── Core algorithm library (no ROS dependency) ──
add_library(mtcs_core
  src/planner_core.cpp
  src/behavior_planner.cpp
  src/forward_simulate.cc
  src/trajectory_planner.cpp
)
ament_target_dependencies(mtcs_core PUBLIC
  rclcpp
)
target_link_libraries(mtcs_core PUBLIC lane_change_sim_casadi)
# Link generated message types
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(mtcs_core PUBLIC "${cpp_typesupport_target}")

# ── Scenario support library ──
add_library(mtcs_support
  src/scenario/scenario_loader.cpp
)
ament_target_dependencies(mtcs_support PUBLIC
  rclcpp
)
target_link_libraries(mtcs_support PUBLIC "${cpp_typesupport_target}")

# ── Executables ──
add_executable(scene_node src/scene_node.cpp)
ament_target_dependencies(scene_node
  rclcpp visualization_msgs tf2_ros
  geometry_msgs std_msgs
)
target_link_libraries(scene_node mtcs_support "${cpp_typesupport_target}")

add_executable(planner_node src/planner_node.cpp)
ament_target_dependencies(planner_node
  rclcpp rclcpp_action
  visualization_msgs geometry_msgs
)
target_link_libraries(planner_node mtcs_core "${cpp_typesupport_target}")

add_executable(simulation_node src/simulation_node.cpp)
ament_target_dependencies(simulation_node
  rclcpp visualization_msgs nav_msgs
  geometry_msgs sensor_msgs tf2_ros std_msgs
)
target_link_libraries(simulation_node "${cpp_typesupport_target}")

# ── Install ──
install(TARGETS
  scene_node planner_node simulation_node mtcs_core mtcs_support
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config urdf rviz scripts
  DESTINATION share/${PROJECT_NAME}/
)

# ── Install vendored CasADi/IPOPT shared libs into the package lib dir so that
#    sourcing install/setup.bash puts them on LD_LIBRARY_PATH automatically ──
install(DIRECTORY "${VENDOR_PATH}/lib/"
  DESTINATION lib
  FILES_MATCHING PATTERN "*.so*"
)

ament_package()
