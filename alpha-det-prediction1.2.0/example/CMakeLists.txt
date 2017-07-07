cmake_minimum_required(VERSION 2.8)

include(path-config.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

include_directories(
    ${ALPHA_DET_PATH}/include
    ${OPENCV_PATH}/include
)

link_directories(
    ${ALPHA_DET_PATH}/lib
    ${OPENCV_PATH}/lib
)

set(SOURCE_FILES
    AlphaDet.cpp
    main.cpp
)

add_executable(AlphaDet_test ${SOURCE_FILES})

target_link_libraries(AlphaDet_test opencv_world alpha-det-prediction)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
