cmake_minimum_required(VERSION 2.8.3)
project(decision_making_examples)

find_package(catkin REQUIRED COMPONENTS
  decision_making_parser
  decision_making
  roscpp 
  random_numbers
  sensor_msgs
  std_msgs
  geometry_msgs
)       
 
catkin_package()

###########
## Build ## 
###########
    
include_directories(
  ${catkin_INCLUDE_DIRS} 
)
          
add_executable(fsm_roomba src/fsm/Roomba.cpp)
add_executable(fsm_roomba_events src/fsm/RoombaEvents.cpp)
add_executable(fsm_turnstile src/fsm/Turnstile.cpp)
add_executable(fsm_turnstile_events src/fsm/TurnstileEvents.cpp)
add_executable(fsm_wandering src/fsm/Wandering.cpp)
add_executable(fsm_wandering_events src/fsm/WanderingEvents.cpp)
add_executable(hsm_ex src/hsm/ScxmlHierarchyExample.cpp)
#add_executable(bt_ex src/bt/Example1.cpp)
add_executable(tao_ex src/tao/TaoExample.cpp)
add_executable(tao_increment src/tao/TaoIncrement.cpp)
add_executable(hsm_mission src/hsm/Mission.cpp)
 
target_link_libraries(fsm_roomba ${catkin_LIBRARIES})
target_link_libraries(fsm_roomba_events ${catkin_LIBRARIES})
target_link_libraries(fsm_turnstile ${catkin_LIBRARIES})
target_link_libraries(fsm_turnstile_events ${catkin_LIBRARIES})
target_link_libraries(fsm_wandering ${catkin_LIBRARIES})
target_link_libraries(fsm_wandering_events ${catkin_LIBRARIES})
target_link_libraries(hsm_ex ${catkin_LIBRARIES})
#target_link_libraries(bt_ex ${catkin_LIBRARIES})
target_link_libraries(tao_ex ${catkin_LIBRARIES})
target_link_libraries(tao_increment ${catkin_LIBRARIES})
target_link_libraries(hsm_mission ${catkin_LIBRARIES}) 


decision_making_parsing(
	src/fsm/Roomba.cpp
	src/fsm/Turnstile.cpp
	src/fsm/Wandering.cpp
	src/hsm/ScxmlHierarchyExample.cpp
#	src/bt/Example1.cpp
	src/tao/TaoExample.cpp
	src/tao/TaoIncrement.cpp
	src/hsm/Mission.cpp
)

#############
## Install ##
#############

install(TARGETS fsm_roomba fsm_roomba_events fsm_turnstile fsm_turnstile_events fsm_wandering fsm_wandering_events
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
