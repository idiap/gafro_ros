find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED)

target_include_directories(${PROJECT_NAME} 
	PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
	SYSTEM PRIVATE 
		${catkin_INCLUDE_DIRS}
	SYSTEM INTERFACE 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)
target_link_libraries(${PROJECT_NAME}
	PUBLIC
		gafro::gafro
		gafro_robot_descriptions::gafro_robot_descriptions
		${catkin_LIBRARIES}
		sackmesser::sackmesser
		Eigen3::Eigen
        ${YAML_CPP_LIBRARIES}
)

add_subdirectory(${PROJECT_NAME})

add_executable(convert_urdf gafro_ros/convert_urdf.cpp)
target_link_libraries(convert_urdf
    PUBLIC
        ${catkin_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
		${PROJECT_NAME}        
)
target_include_directories(convert_urdf
	SYSTEM PRIVATE 
		${catkin_INCLUDE_DIRS}
)