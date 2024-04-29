/**
 * CZoomyServer.h - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#pragma once

#include <iostream>
#include <CUDPServer.hpp>
#include "CCommonBase.hpp"
#include "CControlPi.hpp"

enum value_type {
    GC_LEFTX,
    GC_LEFTY,
    GC_RIGHTX,
    GC_RIGHTY,
    GC_A,
    GC_B,
    GC_X,
    GC_Y,
};

enum pins {
    MOTOR_NW = 5,
    MOTOR_NE = 6,
    MOTOR_SW = 12,
    MOTOR_SE = 13,
};

class CZoomyServer : public CCommonBase {
private:
    // net
    std::string _port;
    sockaddr_in _client{};
    CUDPServer _server;
    std::thread _thread_tx, _thread_rx;
    std::chrono::steady_clock::time_point _timeout_count;
    int _time_since_start;
    std::queue<std::vector<uint8_t>> _tx_queue, _rx_queue;
    std::vector<uint8_t> _rx_buf;
    long _rx_bytes;

    // pigpio
    std::vector<int> _input_pins, _output_pins;
    std::vector<int> _raw_values, _values;
    CControlPi _control;

    // OpenCV
    cv::Mat _frame;
    cv::Mat _camera_frame;
    cv::VideoCapture _video_capture;

    void process_rx(std::string &rx);
    void rx();
    void tx();

public:
    CZoomyServer(std::string port, std::string gstreamer_string);
    ~CZoomyServer();

    void update() override;
    void draw() override;

    static void thread_rx(CZoomyServer *who_called);
    static void thread_tx(CZoomyServer *who_called);
};
