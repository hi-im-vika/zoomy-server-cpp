/**
 * CZoomyServer.cpp - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CZoomyServer.hpp"

#define PING_TIMEOUT 1000
#define NET_DELAY 1

CZoomyServer::CZoomyServer(std::string port, std::string gstreamer_string) {
    _port = port;

    // pigpio init
    _raw_values = _values = std::vector<int>(8,0);

    _output_pins = {
            pins::MOTOR_NW,
            pins::MOTOR_NE,
            pins::MOTOR_SW,
            pins::MOTOR_SE,
    };

    if (!_control.init_gpio(_input_pins, _output_pins)) {
        spdlog::error("Error during GPIO init.");
        exit(-1);
    }

    if (!_control.init_i2c(CControlPi::i2c_ch::CH1, devices::PCA9685)) {
        spdlog::error("Error during I2C init.");
        exit(-1);
    }

    // net init
    _timeout_count = std::chrono::steady_clock::now();
    _time_since_start = 0;

    _server.setup(_port);

    // start listen thread
    _thread_rx = std::thread(thread_rx, this);
    _thread_rx.detach();

    // start send thread
    _thread_tx = std::thread(thread_tx, this);
    _thread_tx.detach();

    // OpenCV init

    spdlog::info("Launching GStreamer with the below options:");
    spdlog::info("\"" + gstreamer_string + "\"");
    _video_capture = cv::VideoCapture(std::string(gstreamer_string), cv::CAP_GSTREAMER);
    if (!_video_capture.isOpened()) {
        spdlog::error("Could not open camera.");
        exit(-1);
    } else {
        spdlog::info("Camera opened with backend " + _video_capture.getBackendName());
        _video_capture.read(_frame);
    }
}

CZoomyServer::~CZoomyServer() {
    _control.zap_com();
};

// TODO: split up image capture and rx data processing into threads
void CZoomyServer::update() {
    _video_capture.read(_frame);
    cv::Mat smaller;
    cv::resize(_frame,smaller,cv::Size(480,360));
    for (; !_rx_queue.empty(); _rx_queue.pop()) {

        // process received control values
        std::string as_string(std::string(_rx_queue.front().begin(),_rx_queue.front().end()));
        spdlog::info("Received: " + as_string);
        if (as_string != "\005") {
            process_rx(as_string);
        }

        std::vector<uint8_t> encoded;
        cv::imencode(".jpg", smaller, encoded);

        std::string payload("test");
        // echo client data back to client
//        _tx_queue.emplace(std::string(encoded.begin, encoded);
        std::vector<uint8_t> tx_assembled(payload.begin(),payload.end());
        tx_assembled.insert(tx_assembled.end(),encoded.begin(),encoded.end());
        _tx_queue.emplace(tx_assembled);
    }

    _time_since_start = (int) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _timeout_count).count();
//    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::draw() {

//    // DEBUG: cycle between 0% and 100% duty cycle for PWM motor control
//    for (int duty = 0; duty < 255; duty++) {
//        gpioPWM(gpio_pins::MOTOR_NW, 0 + duty);
//        gpioPWM(gpio_pins::MOTOR_NE, 0 + duty);
//        gpioPWM(gpio_pins::MOTOR_SW, 0 + duty);
//        gpioPWM(gpio_pins::MOTOR_SE, 0 + duty);
//        std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(10));
//    }
//    for (int duty = 0; duty < 255; duty++) {
//        gpioPWM(gpio_pins::MOTOR_NW, 255 - duty);
//        gpioPWM(gpio_pins::MOTOR_NE, 255 - duty);
//        gpioPWM(gpio_pins::MOTOR_SW, 255 - duty);
//        gpioPWM(gpio_pins::MOTOR_SE, 255 - duty);
//        std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(10));
//    }

    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(10));
}

void CZoomyServer::rx() {
    _rx_bytes = 0;
    _rx_buf.clear();
    _server.do_rx(_rx_buf, _client, _rx_bytes);
    std::vector<uint8_t> temp(_rx_buf.begin(),_rx_buf.begin() + _rx_bytes);
    // only add to rx queue if data is not empty
    if(!temp.empty()) _rx_queue.emplace(temp);
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::tx() {
    for (; !_tx_queue.empty(); _tx_queue.pop()) {
//            spdlog::info("Sending");
        _server.do_tx(_tx_queue.front(),_client);
    }
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::thread_rx(CZoomyServer *who_called) {
    while (!who_called->_do_exit) {
        who_called->rx();
    }
}

void CZoomyServer::thread_tx(CZoomyServer *who_called) {
    while (!who_called->_do_exit) {
        who_called->tx();
    }
}

void CZoomyServer::process_rx(std::string &rx) {
    std::stringstream ss;
    std::string temp;
    ss.str(rx);
    int idx = 0;
    while(ss >> temp) {
        _values.at(idx) = std::stoi(temp);
        idx++;
    }
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: server <port> <gstreamer string>" << std::endl;
        return 1;
    }
    CZoomyServer c = CZoomyServer(argv[1], argv[2]);
    c.run();
    return 0;
}