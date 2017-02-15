cmake_minimum_required(VERSION 2.4.6)
project(asr_flir_ptu_controller)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    genmsg
    actionlib_msgs
    actionlib
)

find_package(Boost REQUIRED COMPONENTS
  system
  thread
)
add_action_files(DIRECTORY action FILES PTUMovement.action)
generate_messages(DEPENDENCIES actionlib_msgs sensor_msgs)
include_directories(include deprecated/include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include deprecated/include
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS Boost
)
FILE(GLOB_RECURSE HEADERS "include/ptu_controller/*.h")
add_executable(PTUController
    ${HEADERS}
  src/PTUController.cpp
)
target_link_libraries(PTUController
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(PTUController
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_generate_messages_cpp
        asr_flir_ptu_driver_gencpp
)
add_executable(PTUControllerClient
  src/PTUControllerClient.cpp
)
target_link_libraries(PTUControllerClient
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(PTUControllerClient
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_generate_messages_cpp
)

add_library(PTUControllerCPP
  deprecated/src/PTUControllerCPP.cpp
)
target_link_libraries(PTUControllerCPP
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(PTUControllerCPP
	${catkin_EXPORTED_TARGETS}
	${PROJECT_NAME}_generate_messages_cpp
	asr_flir_ptu_driver_gencpp
)
