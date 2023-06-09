cmake_minimum_required(VERSION 3.8)
project(oakd_lite_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
# find_package(joint_state_publisher REQUIRED)
find_package(urdf REQUIRED)
# find_package(xacro REQUIRED)



install (DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install (DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install (DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install (DIRECTORY config DESTINATION share/${PROJECT_NAME})
install (DIRECTORY worlds DESTINATION share/${PROJECT_NAME})

# set( IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR} )
# set( IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR} )

# target_link_libraries( ignition-gazebo-sensors-system
#   PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
#   PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
