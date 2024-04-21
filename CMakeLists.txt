cmake_minimum_required(VERSION 3.0.2)
project(homework1)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs nav_msgs dynamic_reconfigure tf2 tf2_ros message_generation message_filters rosbag)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

add_message_files(
        FILES
        wheelSpeed.msg
)


add_service_files(
        FILES
        givePose.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
        nav_msgs
)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs std_msgs message_runtime message_filters)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(robotVelocity src/robotVelocity.cpp)
add_dependencies(robotVelocity ${catkin_EXPORTED_TARGETS})
add_dependencies(robotVelocity homework1_generate_messages_cpp)
target_link_libraries(robotVelocity ${catkin_LIBRARIES})

add_executable(robotOdometry src/robotOdometry.cpp) 
add_dependencies(robotOdometry ${PROJECT_NAME}_gencfg)
target_link_libraries(robotOdometry ${catkin_LIBRARIES})

add_executable(parametersCalibration src/parametersCalibration.cpp) 
target_link_libraries(parametersCalibration ${catkin_LIBRARIES})
target_link_libraries(parametersCalibration ${rosbag_LIBRARIES})
