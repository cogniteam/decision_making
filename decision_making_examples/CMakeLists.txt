cmake_minimum_required(VERSION 2.8.3)
project(decision_making_examples)

find_package(catkin REQUIRED COMPONENTS
  decision_making_parser
  decision_making
  roscpp
  random_numbers
  sensor_msgs
  geometry_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example1 src/example1.cpp)
add_executable(roombaFSM src/Roomba.cpp)
add_executable(turnstileFSM src/Turnstile.cpp)
add_executable(ScxmlHierarchyExampleFSM src/ScxmlHierarchyExample.cpp)
add_executable(wandering src/Wandering.cpp)

target_link_libraries(example1 ${catkin_LIBRARIES} )
target_link_libraries(roombaFSM ${catkin_LIBRARIES} )
target_link_libraries(turnstileFSM ${catkin_LIBRARIES} )
target_link_libraries(ScxmlHierarchyExampleFSM ${catkin_LIBRARIES} )
target_link_libraries(wandering ${catkin_LIBRARIES})

decision_making_parsing(
	src/Roomba.cpp
	src/Turnstile.cpp
	src/ScxmlHierarchyExample.cpp
	src/Wandering.cpp
)

#############
## Install ##
#############

install(TARGETS example1 roombaFSM turnstileFSM ScxmlHierarchyExampleFSM wandering
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)