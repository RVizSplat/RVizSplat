cmake_minimum_required(VERSION 3.10)

project(gsplat_rviz_trials)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wnon-virtual-dtor -Woverloaded-virtual)
endif()

# We specifically don't turn on CMAKE_AUTOMOC, since it generates one huge
# mocs_compilation.cpp file that takes a lot of memory to compile.  Instead
# we create individual moc files that can be compiled separately.

find_package(ament_cmake_ros REQUIRED)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(GLEW REQUIRED)

# Avoid find_package(QT NAMES Qt6 Qt5 ...) due to CMake's default ascending path resolution
find_package(Qt6 QUIET COMPONENTS Widgets)
if(Qt6_FOUND)
  set(QT_VERSION_MAJOR 6)
else()
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(QT_VERSION_MAJOR 5)
endif()
set(QT_VERSION "${Qt${QT_VERSION_MAJOR}_VERSION}")
if(${QT_VERSION} VERSION_LESS 5.15.0)
  function(qt_wrap_cpp out)
    qt5_wrap_cpp(_sources ${ARGN})
    set("${out}" ${_sources} PARENT_SCOPE)
  endfunction()
endif()

set(gsplat_rviz_trials_headers_to_moc
  include/gsplat_rviz_trials/displays/gsplat_display.hpp
)

foreach(header "${gsplat_rviz_trials_headers_to_moc}")
  qt_wrap_cpp(gsplat_rviz_trials_moc_files "${header}")
endforeach()

set(gsplat_rviz_trials_source_files
  src/gsplat_rviz_trials/displays/gsplat_display.cpp
  src/gsplat_rviz_trials/ply_loader.cpp
  src/gsplat_rviz_trials/splat_cloud.cpp
)

add_library(gsplat_rviz_trials SHARED
  ${gsplat_rviz_trials_moc_files}
  ${gsplat_rviz_trials_source_files}
)

target_include_directories(gsplat_rviz_trials PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(gsplat_rviz_trials PUBLIC
  Qt${QT_VERSION_MAJOR}::Widgets
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_ogre_vendor::OgreMain
  rviz_rendering::rviz_rendering
  GLEW::GLEW
)

target_compile_definitions(gsplat_rviz_trials PRIVATE "GSPLAT_RVIZ_TRIALS_BUILDING_LIBRARY")

register_rviz_ogre_media_exports(DIRECTORIES 
  "ogre_media/materials/scripts"
  "ogre_media/materials/glsl120"
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(gsplat_rviz_trials HAS_LIBRARY_TARGET)
ament_export_dependencies(
  pluginlib
  rclcpp
  rviz_common
  rviz_ogre_vendor
  rviz_rendering
  Qt${QT_VERSION_MAJOR}
)

install(
  TARGETS gsplat_rviz_trials
  EXPORT gsplat_rviz_trials
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
