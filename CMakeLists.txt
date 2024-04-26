cmake_minimum_required(VERSION 3.10)
project(zoomy-server)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

add_subdirectory(vendor)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(spdlog REQUIRED)
find_library(PIGPIO_LIBS pigpio REQUIRED)

add_executable(zoomy-server
        src/CZoomyServer.cpp
        include/CZoomyServer.hpp
        src/CCommonBase.cpp
        include/CCommonBase.hpp
)
target_link_libraries(zoomy-server ${OpenCV_LIBS} ${PIGPIO_LIBS} spdlog::spdlog vika-udp)