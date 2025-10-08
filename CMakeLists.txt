cmake_minimum_required(VERSION 3.14)
project(orbslam3_zmq_bridge LANGUAGES CXX)

option(ENABLE_KF_POLLING "Enable keyframe pose polling if supported" ON)
option(BUILD_ORBSLAM3_FROM_SOURCE "Add ORB-SLAM3 subdirectory if available" ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Dependencies
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(Eigen3 REQUIRED)

# ZeroMQ
find_package(PkgConfig REQUIRED)
pkg_check_modules(PC_ZMQ REQUIRED libzmq)
find_path(CPPZMQ_INCLUDE_DIRS zmq.hpp)
if(NOT CPPZMQ_INCLUDE_DIRS)
  message(STATUS "cppzmq headers not found in default include paths; assuming system provides zmq.hpp")
endif()

# ORB-SLAM3 (required)
set(ORB_SLAM3_DIR "" CACHE PATH "Path to ORB-SLAM3 install or build directory")
set(ORB_SLAM3_INCLUDE_DIR "" CACHE PATH "Path to ORB-SLAM3 include directory (containing System.h)")
set(ORB_SLAM3_LIBRARIES "" CACHE STRING "ORB-SLAM3 libraries to link (semicolon-separated)")

if(BUILD_ORBSLAM3_FROM_SOURCE)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3/CMakeLists.txt)
    message(STATUS "Adding ORB-SLAM3 subdirectory: orb_slam3")
    add_subdirectory(orb_slam3)
    # Expect upstream to define target ORB_SLAM3 or orb_slam3_lib; let user map below as needed
  endif()
endif()

if(ORB_SLAM3_INCLUDE_DIR AND ORB_SLAM3_LIBRARIES)
  message(STATUS "Using ORB-SLAM3 from explicit include/lib variables")
  set(HAVE_ORB_SLAM3 TRUE)
elseif(TARGET ORB_SLAM3)
  message(STATUS "Found target ORB_SLAM3 from subdirectory")
  set(HAVE_ORB_SLAM3 TRUE)
  set(ORB_SLAM3_LIBRARIES ORB_SLAM3)
elseif(TARGET orb_slam3_lib)
  message(STATUS "Found target orb_slam3_lib from subdirectory")
  set(HAVE_ORB_SLAM3 TRUE)
  set(ORB_SLAM3_LIBRARIES orb_slam3_lib)
elseif(ORB_SLAM3_DIR)
  find_path(ORB_SLAM3_INCLUDE_DIR_TMP System.h
    HINTS
      ${ORB_SLAM3_DIR}
      ${ORB_SLAM3_DIR}/include
      ${ORB_SLAM3_DIR}/include/ORB_SLAM3)
  if(ORB_SLAM3_INCLUDE_DIR_TMP)
    set(ORB_SLAM3_INCLUDE_DIR ${ORB_SLAM3_INCLUDE_DIR_TMP})
  endif()
  find_library(ORB_SLAM3_LIB_TMP NAMES ORB_SLAM3 orb_slam3
    HINTS ${ORB_SLAM3_DIR} ${ORB_SLAM3_DIR}/lib ${ORB_SLAM3_DIR}/build)
  if(ORB_SLAM3_LIB_TMP)
    set(ORB_SLAM3_LIBRARIES ${ORB_SLAM3_LIB_TMP})
  endif()
  if(ORB_SLAM3_INCLUDE_DIR AND ORB_SLAM3_LIBRARIES)
    set(HAVE_ORB_SLAM3 TRUE)
  endif()
endif()

if(NOT HAVE_ORB_SLAM3)
  message(FATAL_ERROR "ORB-SLAM3 not detected. Provide ORB_SLAM3_INCLUDE_DIR and ORB_SLAM3_LIBRARIES, or enable BUILD_ORBSLAM3_FROM_SOURCE with orb_slam3 subdir present.")
endif()

add_library(bridge_common
  src/zmq_util.cpp
  src/slam_runner.cpp
)
target_include_directories(bridge_common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CPPZMQ_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(bridge_common PUBLIC
  ${OpenCV_LIBS}
  ${PC_ZMQ_LIBRARIES}
  Eigen3::Eigen
)
target_compile_definitions(bridge_common PUBLIC
  $<$<BOOL:${ENABLE_KF_POLLING}>:ENABLE_KF_POLLING>
)

if(HAVE_ORB_SLAM3)
  if(ORB_SLAM3_INCLUDE_DIR)
    target_include_directories(bridge_common PUBLIC ${ORB_SLAM3_INCLUDE_DIR})
  endif()
  if(ORB_SLAM3_LIBRARIES)
    target_link_libraries(bridge_common PUBLIC ${ORB_SLAM3_LIBRARIES})
  endif()
endif()

add_executable(rgbd_zmq_bridge src/main.cpp)
target_link_libraries(rgbd_zmq_bridge PRIVATE bridge_common)

install(TARGETS rgbd_zmq_bridge RUNTIME DESTINATION bin)