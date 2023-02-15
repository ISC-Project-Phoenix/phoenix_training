cmake_minimum_required(VERSION 3.8)
project(run_mgr)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)

add_executable(run_mgr src/run_mgr.cpp src/run_mgr_node.cpp)
target_include_directories(run_mgr PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(run_mgr PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

# Make deps a variable so they get linked to tests as well
set(dependencies
        rclcpp
        sensor_msgs
        std_msgs
        cv_bridge
        OpenCV
        )

ament_target_dependencies(
        run_mgr
        ${dependencies}
)

install(TARGETS run_mgr
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    # set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()
