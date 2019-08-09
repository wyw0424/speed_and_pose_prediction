cmake_minimum_required(VERSION 2.8.3)
project(speed_and_pose_prediction)

find_package(Eigen3 REQUIRED)


# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS 
			roscpp 
			sensor_msgs
			cv_bridge
            geometry_msgs
)
catkin_package(CATKIN_DEPENDS roscpp)

find_package(OpenCV REQUIRED)
message(STATUS " version:${OpenCV_VERSION}")

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(speed_and_pose_prediction_ros_node src/speed_and_pose_prediction_ros_node.cpp)
target_link_libraries(speed_and_pose_prediction_ros_node ${catkin_LIBRARIES})


set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11")
