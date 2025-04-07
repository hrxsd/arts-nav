cmake_minimum_required(VERSION 2.8.3)
project(navigation)

ADD_COMPILE_OPTIONS(-std=c++14 )


find_package(catkin REQUIRED COMPONENTS
    actionlib
    geometry_msgs
    message_generation
    move_base_msgs
    nav_msgs
    roscpp
    rospy
    sensor_msgs
    std_msgs
    tf2_eigen
    tf2_geometry_msgs
    tf2_ros
    roslib
    std_srvs
    tf
)



include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

# add_message_files(
#   FILES
#   Pose6D.msg
# )

add_service_files(
  FILES
  nav_srv.srv
  ints.srv
  arx5_grasp.srv
  lcm_srv.srv
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
 std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime nav_msgs tf
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

catkin_install_python(PROGRAMS
  src/lcm_server.py
  src/exlcm/example_t.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_library(navcore src/Navcore.cc)
target_link_libraries(navcore ${catkin_LIBRARIES})

add_library(jsonlib src/lib_json/json_reader.cpp src/lib_json/json_value.cpp src/lib_json/json_writer.cpp)
target_link_libraries(jsonlib ${catkin_LIBRARIES})


# add_executable(navigation src/main.cc)
# target_link_libraries(navigation navcore jsonlib ${catkin_LIBRARIES})
# target_include_directories(main ${catkin_EXPORTED_TARGETS})
# target_include_directories(main ${catkin_LIBRARIES})

add_executable(navigation src/main.cc)
add_dependencies(navigation ${PROJECT_NAME}_gencpp)
target_link_libraries(navigation  navcore ${catkin_LIBRARIES})

add_executable(nav_status src/main_satus.cc)
add_dependencies(nav_status ${PROJECT_NAME}_gencpp)
target_link_libraries(nav_status  navcore ${catkin_LIBRARIES})

add_executable(nav_obs src/nav_obs.cc)
add_dependencies(nav_obs ${PROJECT_NAME}_gencpp)
target_link_libraries(nav_obs  navcore ${catkin_LIBRARIES})


add_executable(navi_service src/navi_server.cc)
add_dependencies(navi_service ${PROJECT_NAME}_gencpp)
target_link_libraries(navi_service jsonlib navcore ${catkin_LIBRARIES})



