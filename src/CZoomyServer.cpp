/**
 * CZoomyServer.cpp - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CZoomyServer.hpp"

#define PING_TIMEOUT 1000
#define NET_DELAY 1

CZoomyServer::CZoomyServer(std::string port) {
    _port = port;
    _output_pins.push_back(pins::LAUNCHER);

    // pigpio init

    if (!_control.init_gpio(_input_pins, _output_pins)) {
        spdlog::error("Error during GPIO init.");
        exit(-1);
    }

    if (!_control.init_pca9685(CControlPi::i2c_ch::CH1)) {
        spdlog::error("Error during PCA9685 init.");
        exit(-1);
    }

    if (!_control.init_mpu6050(CControlPi::i2c_ch::CH0)) {
        spdlog::error("Error during MPU6050 init.");
        exit(-1);
    }

    if (!_mecanum.init(&_control, 0.35, true)) {
        spdlog::error("Error during CMecanumMove init.");
        exit(-1);
    }

    // net init
    _timeout_count = std::chrono::steady_clock::now();
    _server.setup(_port);

    // start listen thread
    _thread_rx = std::thread(thread_rx, this);
    _thread_rx.detach();

    // start send thread
    _thread_tx = std::thread(thread_tx, this);
    _thread_tx.detach();
}

CZoomyServer::~CZoomyServer() {
    deinit();
}

void CZoomyServer::deinit() {
    _do_exit = true;
    if (_thread_rx.joinable()) _thread_rx.join();
    if (_thread_tx.joinable()) _thread_tx.join();
    _control.zap_com();
}

void CZoomyServer::update() {
    for (; !_rx_queue.empty(); _rx_queue.pop()) {

        // process received control values
        std::string as_string(std::string(_rx_queue.front().begin(), _rx_queue.front().end()));

        // only process data if not ping
        if (as_string != "\005") {
            _control.queue_new_gc_data(as_string);
        }

        std::string payload("test");
        _tx_queue.emplace(payload.begin(), payload.end());
    }

}

void CZoomyServer::draw() {
    _mecanum.moveOmni(-_control.get_gc_values()[0], _control.get_gc_values()[1], _control.get_gc_values()[2], _control.get_gc_values()[3]);
    //_mecanum.setRotation(_control.get_gc_values()[4] - 180.0);

    // conform to OOP standards later!!
    if(_control.get_gc_values().at(6)) {
        gpioWrite(pins::LAUNCHER, PI_ON);
    } else {
        gpioWrite(pins::LAUNCHER, PI_OFF);
    }

    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(1));
}

void CZoomyServer::rx() {
    _rx_bytes = 0;
    _rx_buf.clear();
    _server.do_rx(_rx_buf, _client, _rx_bytes);
    std::vector<uint8_t> temp(_rx_buf.begin(), _rx_buf.begin() + _rx_bytes);
    // only add to rx queue if data is not empty
    if (!temp.empty()) _rx_queue.emplace(temp);
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::tx() {
    for (; !_tx_queue.empty(); _tx_queue.pop()) {
        _server.do_tx(_tx_queue.front(), _client);
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

int main(int argc, char *argv[]) {

    if (argc != 2) {
        std::cerr << "Usage: server <port>" << std::endl;
        return 1;
    }
    CZoomyServer c = CZoomyServer(argv[1]);
    c.run();
    return 0;
}