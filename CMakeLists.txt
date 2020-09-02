cmake_minimum_required(VERSION 2.8.3)
project(roboticarts_leds)


add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  serial
  message_generation
)


add_service_files(
    FILES
    SetLeds.srv
    SetLight.srv
 )


 generate_messages(
   DEPENDENCIES
   std_msgs
   std_srvs
)


catkin_package(
  CATKIN_DEPENDS roscpp rospy serial std_msgs std_srvs

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



# Add test.cpp script
# add_executable(teleop_leds scripts/test.cpp)
# target_link_libraries(test ${catkin_LIBRARIES} )




# Add leds_driver.cpp, leds_teleop, leds_navigation, leds_mux.cpp
add_library(${PROJECT_NAME} src/leds_driver.cpp src/leds_teleop.cpp src/leds_navigation.cpp src/leds_mux.cpp )

# Add leds_driver_node.cpp
add_dependencies(roboticarts_leds ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(leds_driver_node src/leds_driver_node.cpp)
add_dependencies(leds_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roboticarts_leds ${catkin_LIBRARIES})
target_link_libraries(leds_driver_node ${catkin_LIBRARIES} roboticarts_leds)


# Add leds_teleop_node.cpp
add_dependencies(roboticarts_leds ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(leds_teleop_node src/leds_teleop_node.cpp)
add_dependencies(leds_teleop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roboticarts_leds ${catkin_LIBRARIES})
target_link_libraries(leds_teleop_node ${catkin_LIBRARIES} roboticarts_leds)


# Add leds_navigation_node.cpp
#add_dependencies(roboticarts_leds ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_executable(leds_navigation_node src/leds_navigation_node.cpp)
#add_dependencies(leds_navigation_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(roboticarts_leds ${catkin_LIBRARIES})
#target_link_libraries(leds_navigation_node ${catkin_LIBRARIES} roboticarts_leds)


# Add leds_mux_node.cpp
add_dependencies(roboticarts_leds ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(leds_mux_node src/leds_mux_node.cpp)
add_dependencies(leds_mux_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roboticarts_leds ${catkin_LIBRARIES})
target_link_libraries(leds_mux_node ${catkin_LIBRARIES} roboticarts_leds)


