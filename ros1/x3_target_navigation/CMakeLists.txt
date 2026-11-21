cmake_minimum_required(VERSION 3.0.2)
project(x3_target_navigation)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  actionlib geometry_msgs message_filters
  message_generation move_base_msgs nav_msgs roscpp sensor_msgs
  std_msgs std_srvs tf2_geometry_msgs tf2_ros
)
find_package(OpenCV 4 REQUIRED COMPONENTS core dnn imgproc)

add_message_files(FILES TargetEstimate.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES x3_target_core
  CATKIN_DEPENDS actionlib geometry_msgs message_runtime move_base_msgs
                 nav_msgs roscpp sensor_msgs std_msgs std_srvs tf2_geometry_msgs tf2_ros
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_library(x3_target_core
  src/depth_geometry.cpp
  src/parking_geometry.cpp
  src/target_fusion.cpp
)
add_dependencies(x3_target_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(x3_target_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

foreach(node rgbd_target_node target_fusion_node target_task_manager)
  add_executable(${node} src/${node}.cpp)
  add_dependencies(${node} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${node} x3_target_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endforeach()

install(TARGETS x3_target_core rgbd_target_node target_fusion_node target_task_manager
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
catkin_install_python(PROGRAMS
  scripts/x3_base_driver.py
  scripts/check_exclusive_devices.py
  scripts/odom_tf_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/test_obstacle_threshold.py
    test/test_bypass_behavior.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/obstacle_threshold.test)
  foreach(scenario right left full no_path side_blocked dynamic server_delayed wrong_direction brake_rebound)
    add_rostest(test/bypass_behavior.test ARGS scenario:=${scenario})
  endforeach()
  catkin_add_gtest(test_target_core test/test_target_core.cpp)
  if(TARGET test_target_core)
    add_dependencies(test_target_core ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(test_target_core x3_target_core ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  endif()
  add_rostest_gtest(test_preview_safety
    test/preview_safety.test
    test/test_preview_safety.cpp
  )
  if(TARGET test_preview_safety)
    add_dependencies(test_preview_safety ${${PROJECT_NAME}_EXPORTED_TARGETS})
    target_link_libraries(test_preview_safety ${catkin_LIBRARIES})
  endif()
endif()
