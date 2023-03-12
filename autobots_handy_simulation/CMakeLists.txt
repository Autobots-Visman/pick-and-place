cmake_minimum_required(VERSION 3.0.2)
project(autobots_handy_simulation)

find_package(catkin REQUIRED COMPONENTS)
catkin_package(CATKIN_DEPENDS)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    file(GLOB TEST_LAUNCH_FILES tests/*.test)
    foreach(TEST_LAUNCH_FILE ${TEST_LAUNCH_FILES})
        add_rostest(${TEST_LAUNCH_FILE})
    endforeach()
endif()
