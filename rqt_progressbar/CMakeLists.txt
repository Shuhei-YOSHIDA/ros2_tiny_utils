cmake_minimum_required(VERSION 3.8)
project(rqt_progressbar)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(rqt_gui REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(rqt_progressbar_SRCS
  src/rqt_progressbar/rqt_progressbar.cpp
)

set(rqt_progressbar_HDRS
  include/rqt_progressbar/rqt_progressbar.hpp
)

set(rqt_progressbar_UIS
  src/rqt_progressbar/rqt_progressbar.ui
)

qt5_wrap_cpp(rqt_progressbar_MOCS ${rqt_progressbar_HDRS})

qt5_wrap_ui(rqt_progressbar_UIS_H ${rqt_progressbar_UIS})

add_library(${PROJECT_NAME} SHARED
  ${rqt_progressbar_SRCS}
  ${rqt_progressbar_MOCS}
  ${rqt_progressbar_UIS_H}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)
ament_target_dependencies(${PROJECT_NAME}
  rqt_gui_cpp
  qt_gui_cpp
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME})

#install(PROGRAMS scripts/rqt_image_view
  #DESTINATION lib/${PROJECT_NAME}
#)

#install(PROGRAMS scripts/image_publisher
  #DESTINATION lib/${PROJECT_NAME}
#)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

#install(DIRECTORY resource
  #DESTINATION share/${PROJECT_NAME}
#)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export new-style CMake variables
ament_export_targets(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
