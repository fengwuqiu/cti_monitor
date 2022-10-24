cmake_minimum_required(VERSION 3.5)
project(cti_monitor2)

SET(CMAKE_BUILD_TYPE "Release")
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -fdiagnostics-color=always)
endif ()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(cti_msgs REQUIRED)
find_package(Eigen3)
find_package(yaml-cpp REQUIRED)
find_package(jsoncpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(diagnostic_msgs REQUIRED)
set(nvml_lib_dir /usr/src/gdk/nvml/lib/libnvidia-ml.so)

include_directories(
        include
        ../../../install/cti_msgs/include
        ${EIGEN3_INCLUDE_DIR}
        ${YAML_CPP_INCLUDE_DIR}
        ${JSONCPP_INCLUDE_DIRS}
)

file(GLOB_RECURSE ALL_SRCS "src/[a-zA-Z]*.c" "src/[a-zA-Z]*.cc" "src/[a-zA-Z]*.cpp" "include/[a-zA-Z]*.cpp" "include/[a-zA-Z]*.c" "include/[a-zA-Z]*.cc")

add_executable(${PROJECT_NAME}_node ${ALL_SRCS})

ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        std_msgs
        sensor_msgs
        rcl_interfaces
        cti_msgs
        )
target_link_libraries(${PROJECT_NAME}_node 
        ${catkin_LIBRARIES} 
        ${nvml_lib_dir}
        pthread 
        yaml-cpp
        jsoncpp
        )

install(TARGETS ${PROJECT_NAME}_node
        DESTINATION lib/${PROJECT_NAME}
        )

install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

ament_package()
