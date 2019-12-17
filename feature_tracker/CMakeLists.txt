cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    )
# from pl-slam
find_package(Boost REQUIRED COMPONENTS regex thread system filesystem)
find_package(OpenCV REQUIRED)
LINK_DIRECTORIES(${OpenCV_LIBS_DIR})#from pl-slam

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(${OpenCV2_INCLUDE_DIRS})#from pl-slam
include_directories(
  ${OpenCV_INCLUDE_DIRS}#from pl-slam
  ${Boost_INCLUDE_DIRS}#from pl-slam
  ${PROJECT_SOURCE_DIR}/3rdparty/line_descriptor/include/#from pl-slam
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    src/auxiliar.cpp
    src/stereoFeatures.cpp
    )

list(APPEND LINK_LIBS
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}#from pl-slam
    ${PROJECT_SOURCE_DIR}/3rdparty/line_descriptor/lib/liblinedesc.so #from pl-slam
)
target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS}
    ${PROJECT_SOURCE_DIR}/3rdparty/line_descriptor/lib/liblinedesc.so #from pl-slam
    ${${Boost_LIBRARIES}}#from pl-slam
)
