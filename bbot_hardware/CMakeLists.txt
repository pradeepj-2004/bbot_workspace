cmake_minimum_required(VERSION 3.16)
project(bbot_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
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


## COMPILE
add_library(
  bbot_hardware
  SHARED
  src/bbot_hardware.cpp
)
target_compile_features(bbot_hardware PUBLIC cxx_std_17)
target_include_directories(bbot_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/bbot_hardware>
)
ament_target_dependencies(
  bbot_hardware PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(
  DIRECTORY launch 
  DESTINATION share/bbot_hardware/launch
)

install(
  DIRECTORY
    scripts
  DESTINATION
    share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/odom.py
  DESTINATION lib/${PROJECT_NAME})



target_link_libraries(bbot_hardware PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "bbot_hardware_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface bbot_hardware.xml)
 
# INSTALL
install(
  DIRECTORY include/bbot_hardware
  DESTINATION include/bbot_hardware
)

install(TARGETS bbot_hardware
  EXPORT export_bbot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_bbot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
