cmake_minimum_required(VERSION 3.16)
project(kvasir_hw_interface LANGUAGES CXX)

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
# Kvasir hardware interface library
# ------------------------------

add_library(
  kvasir_hw_interface
  SHARED
  hardware/kvasir_system.cpp
  hardware/local_nucleo_interface.cpp
)
target_compile_features(kvasir_hw_interface PUBLIC cxx_std_20)
target_include_directories(kvasir_hw_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/kvasir_hw_interface>
)
ament_target_dependencies(
  kvasir_hw_interface PUBLIC
  ${MY_DEPENDENCIES}
)

target_link_libraries(kvasir_hw_interface PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KVASIR_HW_INTERFACE_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface kvasir_hw_interface.xml)

# ------------------------------
# Local nucleo interface library
# ------------------------------

add_library(local_nucleo_interface SHARED
  hardware/local_nucleo_interface.cpp
)
target_compile_features(local_nucleo_interface PUBLIC cxx_std_20)
target_include_directories(local_nucleo_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/kvasir_hw_interface>
)
ament_target_dependencies(local_nucleo_interface PUBLIC ${MY_DEPENDENCIES})
target_link_libraries(local_nucleo_interface PUBLIC serial)
target_compile_definitions(local_nucleo_interface PRIVATE "KVASIR_HW_INTERFACE_BUILDING_DLL")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

# ------------
# Installation
# ------------

install(
  DIRECTORY hardware/include/
  DESTINATION include/kvasir_hw_interface
)
install(TARGETS kvasir_hw_interface local_nucleo_interface
  EXPORT export_kvasir_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# -------
# Exports
# -------

ament_export_targets(export_kvasir_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${MY_DEPENDENCIES})

ament_package()
