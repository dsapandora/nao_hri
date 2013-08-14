cmake_minimum_required(VERSION 2.8.3)
project(nao_human)

# Tell CMake where to find "FindNAOqi.cmake"
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find all required modules for compilation
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    tf
    geometry_msgs
    std_msgs)

find_package(NAOqi REQUIRED COMPONENTS
    qi
    alcommon
    althread
    alvision
    alproxies
    alerror
    alvalue)

find_package(Boost REQUIRED COMPONENTS program_options signals)

# Information for other packages to import this module
catkin_package(CATKIN_DEPENDS roscpp)


include_directories(include 
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${NAOqi_INCLUDE_DIRS})

add_executable(tracker src/tracker.cpp src/interactions.cpp)
target_link_libraries(tracker
        ${catkin_LIBRARIES}
        ${NAOqi_LIBRARIES}
        ${Boost_LIBRARIES})

add_dependencies(tracker ${catkin_EXPORTED_TARGETS})

install(TARGETS tracker
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Instructions to install launch files
#install(DIRECTORY launch/
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )
