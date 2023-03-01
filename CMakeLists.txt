cmake_minimum_required(VERSION 3.8)
project(ur_gz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ros_gz REQUIRED)
if(NOT DEFINED ENV{GZ_VERSION})
  set(ENV{GZ_VERSION} "garden")
endif()
if("$ENV{GZ_VERSION}" STREQUAL "garden")
  find_package(gz-transport12 REQUIRED)
  find_package(gz-msgs9 REQUIRED)

  set(GZ_TARGET_PREFIX gz)
  set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})
  set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

  message(STATUS "Compiling against Gazebo Garden")
else()
  message(FATAL_ERROR "This branch is only compatible with Gazebo Garden forward.")
endif()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
# include
include_directories(include)

# generate universal_robot_gz lib
AUX_SOURCE_DIRECTORY(${PROJECT_SOURCE_DIR}/src DIR_SRCS)
add_library(${PROJECT_NAME} ${DIR_SRCS})

ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    std_msgs
    sensor_msgs
    trajectory_msgs
)

target_link_libraries(${PROJECT_NAME}
  gz-msgs${GZ_MSGS_VER}::core
  gz-transport${GZ_TRANSPORT_VER}::core
)

#executable node
add_executable(joint_state_publisher src/joint_state_publisher_main.cpp)
target_link_libraries(joint_state_publisher ${PROJECT_NAME})

add_executable(joint_controller src/joint_controller_main.cpp)
target_link_libraries(joint_controller ${PROJECT_NAME})

# for gz-sim plugin RobotiqController
# add_library(RobotiqController SHARED plugins/robotiq_controller/RobotiqController.cpp)

# target_link_libraries(RobotiqController
#     gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
# )

#install
install( DIRECTORY
    gz launch resource
  DESTINATION share/${PROJECT_NAME}
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install executables
install(TARGETS joint_state_publisher  joint_controller
  DESTINATION lib/${PROJECT_NAME} 
)

# install(TARGETS RobotiqController 
#    DESTINATION $ENV{HOME}/.gz/gazebo/plugins
# )

# Install executable scripts
install(PROGRAMS 
        scripts/moveit2.py
        scripts/test_pose_goal.py
        scripts/test_gripper.py
    DESTINATION lib/${PROJECT_NAME})

#environment
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/gz.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
