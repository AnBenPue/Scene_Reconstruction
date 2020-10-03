cmake_minimum_required(VERSION 3.5)

project(scene_reconstruction LANGUAGES CXX)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories( src
                     headers)

file(GLOB SR_LIB_SOURCES	
	headers/reconstructor.h
    headers/pointsMatcher.h
    src/reconstructor.cpp
    src/pointsMatcher.cpp
)

add_library( scene_reconstruction_library ${SR_LIB_SOURCES} )

add_executable(scene_reconstruction src/main.cpp)
target_link_libraries(scene_reconstruction scene_reconstruction_library ${OpenCV_LIBS})
