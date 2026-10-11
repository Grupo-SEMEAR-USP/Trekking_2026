cmake_minimum_required(VERSION 3.8)
project(robot_gazebo)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src/best_openvino_model DESTINATION share/${PROJECT_NAME})
install(FILES requirements-vision.txt DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  src/vision_node.py src/perception_node.py src/twist2ackermann.py
  src/odom_node.py src/uart_comm.py src/centraliza_cone.py src/detecta_cor_ros.py
  src/orange_mission_node.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES src/control_core.py src/orange_core.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts docs DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(control_core_tests test/test_control.py)
  ament_add_pytest_test(centering_tests test/test_centraliza_cone.py)
  ament_add_pytest_test(orange_tests test/test_orange.py)
endif()
ament_package()
