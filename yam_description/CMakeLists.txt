cmake_minimum_required(VERSION 3.10)
project(yam_description)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install directories
install(DIRECTORY
  config
  launch
  meshes
  rviz
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
