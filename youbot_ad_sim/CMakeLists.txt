cmake_minimum_required(VERSION 3.0.2)
project(youbot_ad_sim)
find_package(catkin REQUIRED)

catkin_package(
)

include_directories(
)


install(FILES
    launch/slam/slam_control.launch
    launch/slam/slam_gmapping_youbot.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
