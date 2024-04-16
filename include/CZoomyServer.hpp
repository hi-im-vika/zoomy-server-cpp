/**
 * CZoomyServer.h - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#pragma once

#include <iostream>
#include <CUDPServer.hpp>
#include "CCommonBase.hpp"

class CZoomyServer : public CCommonBase {
private:
public:
    CZoomyServer();
    ~CZoomyServer();

    void update() override;
    void draw() override;
};
