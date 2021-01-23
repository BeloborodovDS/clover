cmake_minimum_required(VERSION 2.8.3)
project(orb_slam3)

find_package(catkin REQUIRED COMPONENTS 
  image_transport
  cv_bridge
  roscpp 
  std_msgs
  std_srvs
  genmsg 
  message_generation
  roslib)
find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
    include
    /usr/local/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    submodules/ORB_SLAM3
    submodules/ORB_SLAM3/include
    submodules/ORB_SLAM3/include/CameraModels
    /usr/include/eigen3
)
link_directories(
    /usr/local/lib
)

add_subdirectory(submodules/ORB_SLAM3)

add_executable(slam_mono 
    src/mono.cpp
)
target_compile_options(slam_mono 
    PRIVATE
    -w
    -march=native
    -std=c++11 
    -O3
)

target_link_libraries(slam_mono 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ORB_SLAM3
)
