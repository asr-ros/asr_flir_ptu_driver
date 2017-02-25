cmake_minimum_required(VERSION 2.4.6)
project(asr_flir_ptu_driver)
# wird zum debuggen gebraucht
#set(ROS_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Debug)
#set(wxWidgets_CONFIGURATION mswu)

#Required for ptu-cpi to build. doesnt work
#set($ENV{_LINUX} 1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  tf
  sensor_msgs
  std_msgs
  genmsg
  message_generation)
    
find_package(Boost REQUIRED COMPONENTS system)

find_package(wxWidgets REQUIRED)

add_message_files(
DIRECTORY
msg
FILES
State.msg
)

add_service_files(DIRECTORY srv FILES Validate.srv Predict.srv Range.srv)
generate_messages(DEPENDENCIES std_msgs sensor_msgs asr_flir_ptu_driver)


include(${wxWidgets_USE_FILE})
include_directories( ${wxWidgets_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_srvs tf sensor_msgs message_runtime
   # DEPENDS wxWidgets
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#include( "${wxWidgets_USE_FILE}") 

FILE(GLOB_RECURSE HEADERS "include/*.h")
add_executable(
asr_flir_ptu_driver 
${HEADERS}
src/driver/PTUNode.cpp 
src/driver/PTUDriver.cpp 
src/driver/PTUDriverMock.cpp
src/driver/PTUFree.cpp
#cpi-v1.09.15/code/ptu.c
#cpi-v1.09.15/code/linuxser.c
)
target_link_libraries(asr_flir_ptu_driver ${catkin_LIBRARIES})
add_dependencies(asr_flir_ptu_driver asr_flir_ptu_driver_gencpp asr_flir_ptu_driver_generated_messages_cpp)



add_executable(
ptu_gui 
src/gui/PTU_GUI_Node.cpp 
src/gui/PTU_GUI.cpp 
src/gui/GUI.cpp 
src/gui/wxImagePanel.cpp
)
target_link_libraries(ptu_gui ${catkin_LIBRARIES} ${wxWidgets_LIBRARIES})
add_dependencies(ptu_gui asr_flir_ptu_driver_gencpp asr_flir_ptu_driver_generated_messages_cpp)


