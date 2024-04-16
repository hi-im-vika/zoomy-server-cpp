# vendor/vika-udp/CMakeLists.txt
message(STATUS "Fetching vika-udp ...")

FetchContent_GetProperties(vika-udp)
if (NOT vika-udp_POPULATED)
    FetchContent_Populate(vika-udp)
endif ()

add_library(vika-udp
        ${vika-udp_SOURCE_DIR}/src/CUDPClient.cpp
        ${vika-udp_SOURCE_DIR}/src/CUDPServer.cpp
        ${vika-udp_SOURCE_DIR}/include/CUDPClient.hpp
        ${vika-udp_SOURCE_DIR}/include/CUDPServer.hpp
)

find_package(spdlog REQUIRED)

set_target_properties(vika-udp PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(vika-udp PUBLIC ${vika-udp_SOURCE_DIR}/src ${vika-udp_SOURCE_DIR}/include)
target_link_libraries(vika-udp PUBLIC spdlog::spdlog)

FetchContent_MakeAvailable(vika-udp)