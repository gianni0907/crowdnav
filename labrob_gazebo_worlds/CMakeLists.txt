cmake_minimum_required(VERSION 3.0.2)
project(labrob_gazebo_worlds)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
	${catkin_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(EnhancedActorPlugin SHARED
	src/EnhancedActorPlugin.cc
)

target_link_libraries(EnhancedActorPlugin
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
)


install(TARGETS EnhancedActorPlugin
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)