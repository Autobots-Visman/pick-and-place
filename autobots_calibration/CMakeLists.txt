cmake_minimum_required(VERSION 3.0.2)
project(autobots_calibration)

find_package(catkin
    REQUIRED COMPONENTS
    message_generation
    std_msgs
)

# generate messages
file(GLOB MSG_FILES msg/*.msg)
foreach(MSG_FILE ${MSG_FILES})
    get_filename_component(FILENAME ${MSG_FILE} NAME)
    add_message_files(FILES ${FILENAME})
endforeach()
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

# add launch files
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# add scripts
file(GLOB SCRIPT_FILES scripts/*)
install(
  PROGRAMS ${SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add tests
if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    # for each test in the test directory add it as a test
    file(GLOB TEST_LAUNCH_FILES tests/*.test)
    foreach(TEST_LAUNCH_FILE ${TEST_LAUNCH_FILES})
        add_rostest(${TEST_LAUNCH_FILE})
    endforeach()
endif()
