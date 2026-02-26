cmake_minimum_required(VERSION 3.1)
project(camera_lucid VERSION 0.0)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.orogen/config")
include(camera_lucidBase)

# How to enable tests:
# To enable tests, copy the files from
# "online_test" folder to "test" (without quotes)
# uncomment the 'syskit_orogen_tests(test)' line below
# run "autoproj test enable ." (don't forget the dot) inside
# the drivers/orogen/camera_lucid folder and then amake --force
# finally, "run autoproj test . --tool" and remember to alter
# task_test.rb ip to match the cameras ip.

if (ROCK_TEST_ENABLED)
    enable_testing()
    find_package(Syskit REQUIRED)
    find_package(Rock REQUIRED)
    add_subdirectory(test)
#   syskit_orogen_tests(test)
endif()