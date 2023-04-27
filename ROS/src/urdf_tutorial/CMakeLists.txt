cmake_minimum_required(VERSION 3.5)
project(urdf_tutorial)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(rclpy REQUIRED)

install(
  DIRECTORY images launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

#ament_python_install_package(${PROJECT_NAME})

#install(PROGRAMS
#	scripts/crane_controller.py
#	scripts/crane_HMI.py
#	DESTINATION lib/${PROJECT_NAME}
#)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
