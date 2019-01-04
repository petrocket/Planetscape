cmake_minimum_required(VERSION 3.3.0)

project(Planetscape)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

find_package(OGRE 1.10 REQUIRED)
find_package(OIS REQUIRED)

add_executable(Planetscape src/BaseApplication.cpp src/PlanetScapeApplication.cpp)
target_include_directories(Planetscape PRIVATE src lib ${OIS_INCLUDE_DIRS})
target_link_libraries(Planetscape PRIVATE OgreMain OgreBites ${OIS_LIBRARIES})