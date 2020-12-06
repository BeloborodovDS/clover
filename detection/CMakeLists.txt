SET(OPENVINO_PATH "/opt/intel/openvino")

SET(OPENVINO_LIBS "${OPENVINO_PATH}/deployment_tools/inference_engine/lib/armv7l")
SET(OPENVINO_INCLUDE "${OPENVINO_PATH}/deployment_tools/inference_engine/include")

cmake_minimum_required(VERSION 2.8.3)
project(human_detection)
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

add_message_files(
  FILES
  DetectionBox.msg
  DetectionList.msg)
  
generate_messages(DEPENDENCIES 
    std_msgs
    std_srvs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
    include
    /usr/local/include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OPENVINO_INCLUDE})
link_directories(
    /usr/local/lib
    ${OPENVINO_LIBS})

add_executable(detection_node 
    src/detection.cpp 
    src/ncs_wrapper/vino_wrapper.cpp
    submodules/sort-cpp/sort-c++/SORTtracker.cpp
    submodules/sort-cpp/sort-c++/Hungarian.cpp
    submodules/sort-cpp/sort-c++/KalmanTracker.cpp)
target_compile_options(detection_node 
    PRIVATE 
    -march=armv7-a 
    -std=c++11 
    -O3 
    -Wno-psabi)
target_link_libraries(detection_node 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    dl
    inference_engine
    inference_engine_legacy)
add_dependencies(detection_node 
    ${PROJECT_NAME}_generate_messages_cpp)
