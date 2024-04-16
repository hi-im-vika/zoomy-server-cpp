/**
 * CZoomyClient.h - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#pragma once

#include <iostream>

#include <iostream>
#include <imgui.h>
#include <backends/imgui_impl_sdl2.h>
#include <backends/imgui_impl_opengl3.h>
#include <spdlog/spdlog.h>
#include <SDL.h>
#include <sstream>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <SDL_opengles2.h>
#else
#include <SDL_opengl.h>
#endif

#include <CUDPClient.hpp>

#include "CWindow.hpp"
#include "CCommonBase.hpp"
#include "CDPIHandler.hpp"

class CZoomyClient : public CCommonBase {
private:
    // imgui
    std::unique_ptr<CWindow> _window;
    GLuint _tex;
    GLuint _another_tex;
    cv::Mat _opencv_area;
    cv::Mat _img, _raw_img;
    SDL_Event _evt;

    std::mutex _lockout;

    void mat_to_tex(cv::Mat &input, GLuint &output);
public:
    CZoomyClient(cv::Size s);
    ~CZoomyClient();

    void update() override;
    void draw() override;
};
