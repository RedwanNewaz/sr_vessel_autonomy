cmake_minimum_required(VERSION 3.8)
project(sr_vessel_autonomy)
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(SRC_FILES
        src/ThreadedSocketClient.h
        src/ThreadedSocketClient.cpp
        )

find_package(fmt 7.1.3 )

if(NOT fmt_FOUND)
  message("fmt not found. Performing alternative action.")
  include(cmake/CPM.cmake)
  CPMAddPackage("gh:fmtlib/fmt#7.1.3")
else()
  message("fmt found. Proceeding with normal flow.")

  # Normal flow of the CMake configuration can be defined here.
  # Set compile flags, include directories, link libraries, etc.

endif()




include_directories(
        ${rclcpp_INCLUDE_DIRS}
        ${geometry_msgs_INCLUDE_DIRS}
        ${fmt_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/sr_vessel_autonomy_node.cpp ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}
        ${rclcpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        -pthread
        fmt
        )

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

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
       DESTINATION share/${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
