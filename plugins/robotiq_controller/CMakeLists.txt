if(NOT DEFINED ENV{GZ_VERSION})
  set(ENV{GZ_VERSION} "garden")
  find_package(gz-sim REQUIRED)
  set(GZ_SIM_VER ${gz-sim_VERSION_MAJOR})
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

# for ignition-gazebo plugin RobotiqController
add_library(RobotiqController SHARED RobotiqController.cpp)

target_link_libraries(RobotiqController
    gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})





