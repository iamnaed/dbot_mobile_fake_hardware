cmake_minimum_required(VERSION 3.8)
project(dbot_mobile_fake_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# compile
add_library(
  dbot_mobile_fake_hardware
  SHARED
  src/dbot_mobile_fake_hardware.cpp
)
target_compile_features(
  dbot_mobile_fake_hardware 
  PUBLIC 
  cxx_std_17
)
target_include_directories(
  dbot_mobile_fake_hardware 
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/dbot_mobile_fake_hardware>
)
ament_target_dependencies(
  dbot_mobile_fake_hardware 
  PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface dbot_mobile_fake_hardware.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/dbot_mobile_fake_hardware
)
install(TARGETS dbot_mobile_fake_hardware
  EXPORT export_dbot_mobile_fake_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_dbot_mobile_fake_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
