cmake_minimum_required(VERSION 3.16)
project(loki_hw_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(MY_DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${MY_DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

# ------------------------------
# Loki hardware interface library
# ------------------------------

add_library(
  loki_hw_interface
  SHARED
  hardware/loki_system.cpp
  hardware/local_nucleo_interface.cpp
)
target_compile_features(loki_hw_interface PUBLIC cxx_std_17)
target_include_directories(loki_hw_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/loki_hw_interface>
)
ament_target_dependencies(
  loki_hw_interface PUBLIC
  ${MY_DEPENDENCIES}
)

target_link_libraries(loki_hw_interface PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "LOKI_HW_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface loki_hw_interface.xml)

# ------------------------------
# Local nucleo interface library
# ------------------------------

add_library(local_nucleo_interface SHARED
  hardware/local_nucleo_interface.cpp
)
target_compile_features(local_nucleo_interface PUBLIC cxx_std_17)
target_include_directories(local_nucleo_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/loki_hw_interface>
)
ament_target_dependencies(local_nucleo_interface PUBLIC ${MY_DEPENDENCIES})
target_link_libraries(local_nucleo_interface PUBLIC serial)
target_compile_definitions(local_nucleo_interface PRIVATE "LOKI_HW_INTERFACE_BUILDING_DLL")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# ------------
# Installation
# ------------

install(
  DIRECTORY hardware/include/
  DESTINATION include/loki_hw_interface
)
install(TARGETS loki_hw_interface local_nucleo_interface
  EXPORT export_loki_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# -------
# Exports
# -------

ament_export_targets(export_loki_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${MY_DEPENDENCIES})

ament_package()
