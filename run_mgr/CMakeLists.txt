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
    # Manually invoke clang format so it actually uses our file
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

    find_package(ament_cmake_gtest REQUIRED)

    # Add unit tests
    ament_add_gtest(${PROJECT_NAME}-test
            tests/unit.cpp
            src/run_mgr_node.cpp
            )
    ament_target_dependencies(${PROJECT_NAME}-test ${dependencies})
    target_include_directories(${PROJECT_NAME}-test PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
            )
    # Copy testing images over
    file(COPY tests/test_assets DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif ()

ament_package()
