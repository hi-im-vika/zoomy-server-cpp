cmake_minimum_required(VERSION 3.10)
project(zoomy-cpp)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(vendor)

add_definitions(-DWINDOW_NAME="${CMAKE_PROJECT_NAME}")

add_executable(zoomy-client
        src/CZoomyClient.cpp
        include/CZoomyClient.hpp
        src/CCommonBase.cpp
        include/CCommonBase.hpp
        src/CWindow.cpp
        include/CWindow.hpp
)
target_link_libraries(zoomy-client imgui vika-udp)

add_executable(zoomy-server
        src/CZoomyServer.cpp
        include/CZoomyServer.hpp
        src/CCommonBase.cpp
        include/CCommonBase.hpp
)
target_link_libraries(zoomy-server vika-udp)