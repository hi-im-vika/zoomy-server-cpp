/**
 * CControlPi.cpp - GPIO control on the Raspberry Pi
 * 2024-02-29
 * vika <https://github.com/hi-im-vika>
 */

#include <spdlog/spdlog.h>
#include "../include/CControlPi.hpp"

// TODO: initialize compass/orientation sensor
CControlPi::CControlPi() {
    _ready_gpio = false;
    _ready_i2c = false;
    _ready_pca9685 = false;
    _i2c_handle = 0;
    _gc_values = std::vector<int>(10, 0);

    // start send thread
    _thread_process_gc = std::thread(thread_process_gc, this);
    _thread_process_gc.detach();
}

CControlPi::~CControlPi() {
    _do_exit = true;
    if (_thread_process_gc.joinable()) _thread_process_gc.join();
    zap_com();
}

void CControlPi::zap_com() {
    if(_ready_gpio) gpioTerminate();
    if(_ready_i2c) {
        i2cClose(_i2c_handle);
        _ready_i2c = false;
    }
}

bool CControlPi::init_gpio(const std::vector<int> &input_pins, std::vector<int> &output_pins) {
    if (gpioInitialise() < 0) {
        std::cout << "Error during GPIO init" << std::endl;
        _ready_gpio = false;
        return _ready_gpio;
    }
    if (!input_pins.empty()) {
        for (auto i: input_pins) {
            if (gpioSetMode(i, PI_INPUT) != 0) {
                std::cout << "Error during GPIO input pin setup" << std::endl;
                _ready_gpio = false;
                return _ready_gpio;
            }
        }
    }
    if (!output_pins.empty()) {
        for (auto i: output_pins) {
            if (gpioSetMode(i, PI_INPUT) != 0) {
                std::cout << "Error during GPIO output pin setup" << std::endl;
                _ready_gpio = false;
                return _ready_gpio;
            }
        }
    }
    _ready_gpio = true;
    return _ready_gpio;
}

bool CControlPi::init_i2c(i2c_ch ch, uint address) {
    _i2c_handle = i2cOpen(ch, address, 0);
    if (_i2c_handle < 0) {
        std::cout << "Error during I2C setup" << std::endl;
        _ready_i2c = false;
        return _ready_i2c;
    }
    std::cout << "I2C setup with peripheral address 0x" << std::hex << address << std::endl;
    _ready_i2c = true;
    return _ready_i2c;
}

bool CControlPi::init_pca9685(i2c_ch ch, uint address) {
    if (!_ready_i2c) {
        if (!init_i2c(ch, address)) {
            return false;
        }
    }

    i2c_write_byte(0x00, 0x30);
    i2c_write_byte(0xFE, 0x1E);
    i2c_write_byte(0x00, 0x20);
    i2c_write_byte(0x00, 0xA0);

    uint8_t data = 0;
    i2c_read_byte(0x00, data);

    _ready_pca9685 = (data == 0x20);
    return _ready_pca9685;
}

void CControlPi::pca9685_motor_control(motor m, int value) {
    if (!_ready_pca9685) return;
    int absolute = abs(value);
    if (absolute > 0x0FFF) return;
    if (!value) {
        switch (m) {
            case M_NE:
                i2c_write_word(CControlPi::motor_regs::MREG_NE_F, 0x0000);
                i2c_write_word(CControlPi::motor_regs::MREG_NE_B, 0x0000);
                break;
            case M_NW:
                i2c_write_word(CControlPi::motor_regs::MREG_NW_F, 0x0000);
                i2c_write_word(CControlPi::motor_regs::MREG_NW_B, 0x0000);
                break;
            case M_SE:
                i2c_write_word(CControlPi::motor_regs::MREG_SE_F, 0x0000);
                i2c_write_word(CControlPi::motor_regs::MREG_SE_B, 0x0000);
                break;
            case M_SW:
                i2c_write_word(CControlPi::motor_regs::MREG_SW_F, 0x0000);
                i2c_write_word(CControlPi::motor_regs::MREG_SW_B, 0x0000);
                break;
            default:
                break;
        }
        return;
    }
    if (value > 0) {
        switch (m) {
            case M_NE:
                i2c_write_word(CControlPi::motor_regs::MREG_NE_F, absolute);
                i2c_write_word(CControlPi::motor_regs::MREG_NE_B, 0x0000);
                break;
            case M_NW:
                i2c_write_word(CControlPi::motor_regs::MREG_NW_F, absolute);
                i2c_write_word(CControlPi::motor_regs::MREG_NW_B, 0x0000);
                break;
            case M_SE:
                i2c_write_word(CControlPi::motor_regs::MREG_SE_F, absolute);
                i2c_write_word(CControlPi::motor_regs::MREG_SE_B, 0x0000);
                break;
            case M_SW:
                i2c_write_word(CControlPi::motor_regs::MREG_SW_F, absolute);
                i2c_write_word(CControlPi::motor_regs::MREG_SW_B, 0x0000);
                break;
            default:
                break;
        }
        return;
    } else {
        switch (m) {
            case M_NE:
                i2c_write_word(CControlPi::motor_regs::MREG_NE_B, absolute);
                i2c_write_word(CControlPi::motor_regs::MREG_NE_F, 0x0000);
                break;
            case M_NW:
                i2c_write_word(CControlPi::motor_regs::MREG_NW_B, absolute);
                i2c_write_word(CControlPi::motor_regs::MREG_NW_F, 0x0000);
                break;
            case M_SE:
                i2c_write_word(CControlPi::motor_regs::MREG_SE_B, absolute);
                i2c_write_word(CControlPi::motor_regs::MREG_SE_F, 0x0000);
                break;
            case M_SW:
                i2c_write_word(CControlPi::motor_regs::MREG_SW_B, absolute);
                i2c_write_word(CControlPi::motor_regs::MREG_SW_F, 0x0000);
                break;
            default:
                break;
        }
        return;
    }
}

void CControlPi::queue_new_gc_data(std::string &data) {
    _gc_queue.emplace(data);
}

std::vector<int> CControlPi::get_gc_values() {
    return _gc_values;
}

void CControlPi::process_gc(std::string &gc) {
    std::stringstream ss;
    std::string temp;
    ss.str(gc);
    int idx = 0;
    while(ss >> temp) {
        _gc_values.at(idx) = std::stoi(temp);
        idx++;
    }
}

void CControlPi::thread_process_gc(CControlPi *who_called) {
    while (!who_called->_do_exit) {
        for (; !who_called->_gc_queue.empty(); who_called->_gc_queue.pop()) {
            who_called->process_gc(who_called->_gc_queue.front());
        }
//    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(NET_DELAY));
    }
}

bool CControlPi::i2c_write_byte(uint reg, uint val) {
    if (!_ready_i2c) return false;
    if(i2cWriteByteData(_i2c_handle,reg,val) != 0) {
        return false;
    }
    return true;
}

bool CControlPi::i2c_read_byte(uint reg, uint8_t &data) {
    if (!_ready_i2c) return false;
    uint8_t raw_data = i2cReadByteData(_i2c_handle,reg);
    if(raw_data < 0) {
        return false;
    }
    data = raw_data;
    return true;
}

bool CControlPi::i2c_write_block(uint reg, std::vector<char> &buf) {
    if (!_ready_i2c) return false;
    if(i2cWriteI2CBlockData(_i2c_handle,reg,buf.data(), buf.size()) != 0) {
        return false;
    }
    return true;
}

bool CControlPi::i2c_write_word(uint reg, uint word) {
    if (!_ready_i2c) return false;
    if(i2cWriteWordData(_i2c_handle,reg,word) != 0) {
        return false;
    }
    return true;
}