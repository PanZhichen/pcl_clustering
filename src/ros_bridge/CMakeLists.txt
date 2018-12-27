add_library(ros_bridge SHARED cloud_odom_ros_subscriber.cpp cloud_odom_ros_publisher.cpp)
find_package(PCL 1.7 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
            roscpp
            )
target_link_libraries(ros_bridge
                      identifiable
                      velodyne_utils
                      difference
                      ${catkin_LIBRARIES}
                      ${OpenCV_LIBS}
                      ${PCL_LIBRARIES})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
