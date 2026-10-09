cmake_minimum_required(VERSION 3.8)
project(robot_control)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O3 -Wall)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------------------------------------------------------
# Mensagens customizadas (antes: add_message_files + generate_messages)
# ---------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/I2cData.msg"
  "msg/UARTData.msg"
  "msg/VelocityData.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

# ---------------------------------------------------------------
# movement  (JA CONVERTIDO)
# ---------------------------------------------------------------
add_executable(movement src/movement.cpp)
ament_target_dependencies(movement
  rclcpp geometry_msgs nav_msgs std_msgs tf2 angles)

# ---------------------------------------------------------------
# hw_interface e joycontrol  (TODO: descomentar apos converter os .cpp)
# ---------------------------------------------------------------
# rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#
# add_executable(hw_interface src/hw_interface.cpp)
# ament_target_dependencies(hw_interface rclcpp std_msgs)
# target_link_libraries(hw_interface "${cpp_typesupport_target}")
#
# add_executable(joycontrol src/joycontrol.cpp)
# ament_target_dependencies(joycontrol rclcpp std_msgs geometry_msgs)

# ---------------------------------------------------------------
# Instalacao
# ---------------------------------------------------------------
install(TARGETS movement
  DESTINATION lib/${PROJECT_NAME})
# install(TARGETS hw_interface joycontrol DESTINATION lib/${PROJECT_NAME})

# Scripts Python (so funcionam apos portar de rospy para rclpy)
install(PROGRAMS
  scripts/i2c_master.py
  scripts/uart_comm.py
  scripts/imu.py
  DESTINATION lib/${PROJECT_NAME})

# Em ROS1 o roslaunch achava esses arquivos direto na pasta do codigo-fonte.
# Em ROS2 eles precisam ser INSTALADOS em share/<pacote>, senao nao sao encontrados.
install(DIRECTORY launch config robot_description
  DESTINATION share/${PROJECT_NAME})

ament_package()