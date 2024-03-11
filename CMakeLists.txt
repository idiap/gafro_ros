cmake_minimum_required(VERSION 3.5)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

project(gafro_ros)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_compile_options(
    "-Wall"
    "-Wextra"
    "-Werror=vla"
    "-Wno-unused-function"
    "-Wno-missing-braces"
    "-Wno-unknown-pragmas"
    "-Wno-parentheses"
    "-pedantic"
    "-Wconversion"
    "-Werror=pedantic"
    "-O2"
)

find_package(catkin REQUIRED COMPONENTS 
	roscpp
    sackmesser_ros
    tf
    urdf
)

find_package(gafro REQUIRED)
find_package(gafro_robot_descriptions REQUIRED)
find_package(sackmesser REQUIRED)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src
	CATKIN_DEPENDS roscpp sackmesser_ros urdf
    DEPENDS gafro
)

add_subdirectory(src)

install(
	DIRECTORY "${CATKIN_DEVEL_PREFIX}/include/${PROJECT_NAME}/"
    DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
    FILES_MATCHING PATTERN "*.hpp"
)