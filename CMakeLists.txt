cmake_minimum_required(VERSION 2.8.3)
project(pr2_props)
find_package(catkin REQUIRED COMPONENTS roscpp actionlib pr2_gripper_sensor_action pr2_gripper_sensor_controller pr2_gripper_sensor_msgs)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(high_five src/high_five.cpp)
add_dependencies(high_five pr2_gripper_sensor_msgs_gencpp)
target_link_libraries(high_five ${catkin_LIBRARIES})

add_executable(repeat_high_five src/repeat_high_five.cpp)
add_dependencies(repeat_high_five pr2_gripper_sensor_msgs_gencpp)
target_link_libraries(repeat_high_five ${catkin_LIBRARIES})

add_executable(pound src/pound.cpp)
add_dependencies(pound pr2_gripper_sensor_msgs_gencpp)
target_link_libraries(pound ${catkin_LIBRARIES})

add_executable(low_five src/low_five.cpp)
add_dependencies(low_five pr2_gripper_sensor_msgs_gencpp)
target_link_libraries(low_five ${catkin_LIBRARIES})

add_executable(hug src/hug.cpp)
add_dependencies(hug pr2_gripper_sensor_msgs_gencpp)
target_link_libraries(hug ${catkin_LIBRARIES})
catkin_package(
    DEPENDS roscpp actionlib pr2_gripper_sensor_action pr2_gripper_sensor_controller pr2_gripper_sensor_msgs
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/run_all_props.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS hug low_five pound repeat_high_five high_five
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
