cmake_minimum_required(VERSION 3.5.1)

project(mavsdk_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

file(READ MAVSDK_VERSION MAVSDK_VERSION)
string(STRIP ${MAVSDK_VERSION} MAVSDK_VERSION)

find_package(MAVSDK "${MAVSDK_VERSION}" REQUIRED)
find_package(Threads REQUIRED)

# find gazebo
find_package(gazebo REQUIRED)

# find protobuf
find_package(Protobuf REQUIRED)

# Groundtruth proto 파일 위치
set(GROUNDTRUTH_PROTO ${CMAKE_CURRENT_SOURCE_DIR}/msgs/Groundtruth.proto)
protobuf_generate_cpp(GROUNDTRUTH_PROTO_SRCS GROUNDTRUTH_PROTO_HDRS ${GROUNDTRUTH_PROTO})

if(MAVSDK_FOUND AND gazebo_FOUND)
    add_executable(mavsdk_tests
        autopilot_tester.cpp
        test_main.cpp
	test_multicopter_constspeed_x.cpp
	test_multicopter_constspeed_y.cpp
	test_multicopter_constspeed_z.cpp

        #test_multicopter_20m_normal.cpp
	#test_multicopter_20m.cpp
	#test_multicopter_40m.cpp
	#test_multicopter_80m.cpp
	#test_multicopter_120m.cpp
        ${GROUNDTRUTH_PROTO_SRCS}
        ${GROUNDTRUTH_PROTO_HDRS}
    )

    target_link_libraries(mavsdk_tests
        MAVSDK::mavsdk
        ${CMAKE_THREAD_LIBS_INIT}
        ${GAZEBO_LIBRARIES}                 # Gazebo libraries
        ${Protobuf_LIBRARIES}
    )

    target_include_directories(mavsdk_tests PUBLIC
        ${CMAKE_BINARY_DIR}/..
        ${GAZEBO_INCLUDE_DIRS}              # Gazebo headers
        ${Protobuf_INCLUDE_DIRS}
	${CMAKE_CURRENT_BINARY_DIR}
    )

    target_compile_options(mavsdk_tests
        PRIVATE
            -Wall
            -Wextra
            -Werror
            -Wno-error=deprecated-declarations
    )
else()
    message("MAVSDK C++ not found, skipping mavsdk_tests build..")
endif()
