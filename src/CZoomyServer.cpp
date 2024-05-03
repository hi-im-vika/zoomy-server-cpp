/**
 * CZoomyServer.cpp - new file
 * 2024-04-15
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CZoomyServer.hpp"

#define PING_TIMEOUT 1000
#define NET_DELAY 1
#define DEADZONE 2048

CZoomyServer::CZoomyServer(std::string port, std::string gstreamer_string) {
    _port = port;

    // pigpio init

    if (!_control.init_gpio(_input_pins, _output_pins)) {
        spdlog::error("Error during GPIO init.");
        exit(-1);
    }

    if (!_control.init_pca9685(CControlPi::i2c_ch::CH1)) {
        spdlog::error("Error during PCA9685 init.");
        exit(-1);
    }

    if (!_mecanum.init(&_control)) {
        spdlog::error("Error during CMecanumMove init.");
        exit(-1);
    }

    _joystickA = _joystickB = {0, 0};

    // net init
    _timeout_count = std::chrono::steady_clock::now();

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
    deinit();
}

void CZoomyServer::deinit() {
    _do_exit = true;
    if (_thread_rx.joinable()) _thread_rx.join();
    if (_thread_tx.joinable()) _thread_tx.join();
    _control.zap_com();
}

// TODO: split up image capture and rx data processing into threads
void CZoomyServer::update() {
    _video_capture.read(_frame);
    cv::Mat smaller;
    cv::resize(_frame,smaller,cv::Size(480,360));
    for (; !_rx_queue.empty(); _rx_queue.pop()) {

        // process received control values
        std::string as_string(std::string(_rx_queue.front().begin(),_rx_queue.front().end()));

        // only process data if not ping
        if (as_string != "\005") {
            _control.queue_new_gc_data(as_string);
        }

        std::vector<uint8_t> encoded;
        cv::imencode(".jpg", smaller, encoded);

        std::string payload("test");
        std::vector<uint8_t> tx_assembled(payload.begin(),payload.end());
        tx_assembled.insert(tx_assembled.end(),encoded.begin(),encoded.end());
        _tx_queue.emplace(tx_assembled);
    }

}

void CZoomyServer::draw() {
    auto local_values = _control.get_gc_values();
    if (hypot(local_values[0], local_values[1]) > DEADZONE) {
        _joystickA = {local_values[0], local_values[1]};
    }
    else {
        _joystickA = {0, 0};
    }
    if (hypot(local_values[2], local_values[3]) > DEADZONE) {
        _joystickB = {local_values[2], local_values[3]};
    }
    else {
        _joystickB = {0, 0};
    }
    _mecanum.moveOmni(_joystickA[0], _joystickA[1], _joystickB[0]);
    //_mecanum.moveTank(_joystickB[1], _joystickA[1]);

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

int main(int argc, char *argv[]) {

    if (argc != 3) {
        std::cerr << "Usage: server <port> <gstreamer string>" << std::endl;
        return 1;
    }
    CZoomyServer c = CZoomyServer(argv[1], argv[2]);
    c.run();
    return 0;
}