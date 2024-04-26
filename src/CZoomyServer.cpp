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

    // pigpio init
    _raw_values = _values = std::vector<int>(8,0);
    _output_pins.push_back(CControlPi::gpio_pins::STEERING);
    _output_pins.push_back(CControlPi::gpio_pins::THROTTLE);
    _output_pins.push_back(CControlPi::gpio_pins::HEARTBEAT);
    _control.init_gpio(_input_pins, _output_pins);

    gpioHardwarePWM(CControlPi::gpio_pins::STEERING, pwm::FREQ, pwm::CENTRE);
    gpioHardwarePWM(CControlPi::gpio_pins::THROTTLE, pwm::FREQ, pwm::CENTRE);

    // OpenCV init

    /**
     * camera selection
     * facetime hd camera:
     * _video_capture.open(0);
     * continuity:
     * _video_capture.open(2);
     * raspberry pi:
     * _video_capture = cv::VideoCapture("libcamerasrc ! video/x-raw, width=128, height=96 ! appsink", cv::CAP_GSTREAMER);
     */

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

CZoomyServer::~CZoomyServer() = default;

void CZoomyServer::update() {
    _video_capture.read(_frame);
    cv::Mat smaller;
    cv::resize(_frame,smaller,cv::Size(480,360));
    for (; !_rx_queue.empty(); _rx_queue.pop()) {

        // process received control values
//        process_rx(_rx_queue.front());

        std::vector<uint8_t> encoded;
        cv::imencode(".jpg", smaller, encoded);

        // echo client data back to client
//        _tx_queue.emplace(std::string(encoded.begin, encoded);
        std::vector<uint8_t> tx_assembled(_rx_queue.front());
        tx_assembled.insert(tx_assembled.end(),encoded.begin(),encoded.end());
        _tx_queue.emplace(tx_assembled);
    }

    _time_since_start = (int) std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - _timeout_count).count();
//    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
}

void CZoomyServer::draw() {
    auto steering = (float) (_values.at(0) / 32768.0);
    auto throttle = (float) (_values.at(3) / 32768.0);
    gpioHardwarePWM(CControlPi::gpio_pins::STEERING,pwm::FREQ,CZoomyServer::pwm::CENTRE + (int) (steering * CZoomyServer::pwm::MAX_RANGE));
    gpioHardwarePWM(CControlPi::gpio_pins::THROTTLE,pwm::FREQ,CZoomyServer::pwm::CENTRE + (int) (throttle * -1 * CZoomyServer::pwm::MAX_RANGE));

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
        if (idx >= 2) {
            _values.at(idx - 2) = std::stoi(temp);
        }
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