cmake_minimum_required(VERSION 3.5)
project(yost_3_space_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yost_3_space_api REQUIRED)

# TEST MAIN

add_executable(yost_test src/main.cpp ${API_Sources})
ament_target_dependencies(yost_test sensor_msgs yost_3_space_api)
target_include_directories(yost_test PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
        ${yost_3_space_api_INCLUDE_DIRS})
target_link_libraries(yost_test ${yost_3_space_api_LIBRARIES} udev pthread)

# DRIVER NODE
add_executable(yost_3_space_driver_node src/yost_3_space_driver.cpp ${API_Sources})
ament_target_dependencies(yost_3_space_driver_node rclcpp sensor_msgs yost_3_space_api)
target_include_directories(yost_3_space_driver_node PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
        ${yost_3_space_api_INCLUDE_DIRS})
target_link_libraries(yost_3_space_driver_node ${yost_3_space_api_LIBRARIES} udev pthread)


# install
install(TARGETS
        yost_test
        yost_3_space_driver_node
        DESTINATION lib/${PROJECT_NAME})


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
