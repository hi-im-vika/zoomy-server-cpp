# vendor/CMakeLists.txt

include(FetchContent)

FetchContent_Declare(
        vika-udp
        GIT_REPOSITORY "https://github.com/hi-im-vika/vika-udp.git"
        GIT_TAG         origin/main
)

add_subdirectory(vika-udp)