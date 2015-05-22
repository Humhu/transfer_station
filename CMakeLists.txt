cmake_minimum_required( VERSION 2.8.3 )
project( transfer_station )

find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)

find_package( Boost REQUIRED )

catkin_package(
	INCLUDE_DIRS	include 
	DEPENDS			Boost
)

include_directories( include )
include_directories( 
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_definitions( "-Wall -Wextra -std=c++0x -O2" )
