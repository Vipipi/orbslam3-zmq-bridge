cmake_minimum_required(VERSION 3.14)

project(orb_slam3_internal LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
)

add_library(orb_slam3_lib SHARED
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Viewer.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/GeometricTools.cc
  src/TwoViewReconstruction.cc
  src/Config.cc
  src/Settings.cc
)

target_include_directories(orb_slam3_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}                # for includes like "Thirdparty/DBoW2/..."
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty
  ${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus
)

add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

target_link_libraries(orb_slam3_lib PUBLIC
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${Pangolin_LIBRARIES}
  Boost::serialization
  DBoW2
  g2o
)


