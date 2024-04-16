/**
 * CZoomyServer.cpp - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CZoomyServer.hpp"

CZoomyServer::CZoomyServer() = default;

CZoomyServer::~CZoomyServer() = default;

void CZoomyServer::update() {
    spdlog::info("Server update");
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(10));
}

void CZoomyServer::draw() {
    spdlog::info("Server draw");
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(10));
}

int main() {
//    std::cout << "Hello, World!" << std::endl;
    CZoomyServer c = CZoomyServer();
    c.run();
    return 0;
}