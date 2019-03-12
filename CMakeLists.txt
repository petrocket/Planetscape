cmake_minimum_required(VERSION 3.3.0)

project(Planetscape)

find_package(OGRE 1.10 REQUIRED)

add_executable(Planetscape src/PlanetScapeApplication.cpp)
target_include_directories(Planetscape PRIVATE src rapidjson/include)
target_link_libraries(Planetscape PRIVATE OgreMain OgreBites)