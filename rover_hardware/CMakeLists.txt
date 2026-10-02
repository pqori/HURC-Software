cmake_minimum_required(VERSION 3.8)
project(rover_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(rover_hardware SHARED
  src/TopicDriveSystem.cpp
  src/StepperSystem.cpp
  src/HybridArmSystem.cpp
  src/EPOS.cpp
)

target_include_directories(rover_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rover_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

link_directories(EPOS_Linux_Library/lib/intel/x86_64)

target_link_libraries(rover_hardware
  pluginlib::pluginlib
  EposCmd
)

pluginlib_export_plugin_description_file(
  hardware_interface
  rover_hardware_plugin_description.xml
)

install(TARGETS rover_hardware
  EXPORT export_rover_hardware
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES rover_hardware_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_rover_hardware HAS_LIBRARY_TARGET)
ament_package()







# cmake_minimum_required(VERSION 3.8)
# project(rover_hardware)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# # uncomment the following section in order to fill in
# # further dependencies manually.
# # find_package(<dependency> REQUIRED)
# find_package(hardware_interface REQUIRED)
# find_package(pluginlib REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(rclcpp_lifecycle REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# add_library(rover_hardware SHARED
#   src/TopicDriveSystem.cpp
# )

# target_include_directories(rover_hardware PUBLIC
#   $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# ament_target_dependencies(rover_hardware PUBLIC
#   hardware_interface
#   pluginlib
#   rclcpp
#   rclcpp_lifecycle
# )

# pluginlib_export_plugin_description_file(hardware_interface rover_hardware_plugin_description.xml)

# install(TARGETS rover_hardware
#   EXPORT export_rover_hardware
#   LIBRARY DESTINATION lib
# )

# install(
#   DIRECTORY include/
#   DESTINATION include
# )

# install(
#   FILES rover_hardware_plugin_description.xml
#   DESTINATION share/${PROJECT_NAME}
# )


# ament_export_targets(export_rover_hardware HAS_LIBRARY_TARGET)

# ament_package()
