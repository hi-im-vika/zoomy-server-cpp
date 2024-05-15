/**
 * CControlPi.cpp - GPIO control on the Raspberry Pi
 * 2024-02-29
 * vika <https://github.com/hi-im-vika>
 */

#include <spdlog/spdlog.h>
#include "../include/CControlPi.hpp"

// TODO: initialize compass/orientation sensor
CControlPi::CControlPi() {

    // gpio
    _ready_gpio = false;

    // i2c
    _ready_i2c_ch0 = false;
    _ready_i2c_ch1 = false;
    _i2c_handle_ch0 = 0;
    _i2c_handle_ch1 = 0;

    // device-specific
    _ready_pca9685 = false;
    _handle_pca9685 = nullptr;

    _ready_hmc5883l = false;
    _handle_hmc5883l = nullptr;

    _ready_mpu6050 = false;
    _handle_mpu6050 = nullptr;

    // game controller
    _gc_values = std::vector<int>(10, 0);

    // start send thread
    _thread_process_gc = std::thread(thread_process_gc, this);
    _thread_process_gc.detach();
}

CControlPi::~CControlPi() {
    zap_com();
}

void CControlPi::zap_com() {
    _do_exit = true;
    if (_thread_process_gc.joinable()) _thread_process_gc.join();
    if (_ready_gpio) gpioTerminate();
    if (get_i2c_status(i2c_ch::CH0) || get_i2c_status(i2c_ch::CH1)) {
        i2cClose(_i2c_handle_ch0);
        i2cClose(_i2c_handle_ch1);
        _ready_i2c_ch0 = false;
        _ready_i2c_ch1 = false;
        _ready_hmc5883l = false;
        _ready_pca9685 = false;
        _ready_mpu6050 = false;
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
    int *temp_handle;
    bool *temp_status;
    switch(ch) {
        case i2c_ch::CH0:
            temp_handle = &_i2c_handle_ch0;
            temp_status = &_ready_i2c_ch0;
            break;
        case i2c_ch::CH1:
            temp_handle = &_i2c_handle_ch1;
            temp_status = &_ready_i2c_ch1;
            break;
        default:
            return false;
    }
    *temp_handle = i2cOpen(ch, address, 0);
    if (*temp_handle < 0) {
        std::cout << "Error during I2C setup" << std::endl;
        *temp_status = false;
        return *temp_status;
    }
    std::cout << "I2C setup with peripheral address 0x" << std::hex << address << std::endl;
    *temp_status = true;
    return *temp_status;
}

bool CControlPi::init_pca9685(i2c_ch ch, uint address) {
    if (!init_i2c(ch, address)) {
        return false;
    }

    i2c_write_byte(ch, 0x00, 0x30);
    i2c_write_byte(ch, 0xFE, 0x1E);
    i2c_write_byte(ch, 0x00, 0x20);
    i2c_write_byte(ch, 0x00, 0xA0);

    uint8_t data = 0;
    i2c_read_byte(ch, 0x00, data);

    _ready_pca9685 = (data == 0x20);
    _handle_pca9685 = get_i2c_handle(ch);
    _ch_pca9685 = ch;
    return _ready_pca9685;
}

bool CControlPi::init_hmc5883l(i2c_ch ch, uint address) {
    if (!init_i2c(ch, address)) {
        return false;
    }

    std::vector<char> buffer(3,'\1');

    if(!i2c_read_block(ch, 0x0A, buffer, (int) buffer.size())) {
        spdlog::error("Error during ID read");
        return false;
    }

    if (buffer.at(0) != 0x48 || buffer.at(1) != 0x34 || buffer.at(2) != 0x33) {
        spdlog::error("Device ID mismatch.");
        return false;
    }

    if(!i2c_write_byte(ch, 0x02,0) != 0) {
        spdlog::error("Mode set error.");
        return false;
    }

    _ready_hmc5883l = !(buffer.at(0) != 0x48 || buffer.at(1) != 0x34 || buffer.at(2) != 0x33);
    _handle_hmc5883l = get_i2c_handle(ch);
    _ch_hmc5883l = ch;
    return _ready_hmc5883l;
}

bool CControlPi::init_mpu6050(i2c_ch ch, uint address) {
    return false;
}

void CControlPi::pca9685_motor_control(motor m, int value) {
    if (!_ready_pca9685) {
        spdlog::error("Device not ready.");
        return;
    }
    int absolute = abs(value);
    if (absolute > 0x0FFF) return;
    if (!value) {
        switch (m) {
            case M_NE:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NE_F, 0x0000);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NE_B, 0x0000);
                break;
            case M_NW:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NW_F, 0x0000);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NW_B, 0x0000);
                break;
            case M_SE:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SE_F, 0x0000);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SE_B, 0x0000);
                break;
            case M_SW:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SW_F, 0x0000);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SW_B, 0x0000);
                break;
            default:
                break;
        }
        return;
    }
    if (value > 0) {
        switch (m) {
            case M_NE:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NE_F, absolute);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NE_B, 0x0000);
                break;
            case M_NW:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NW_F, absolute);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NW_B, 0x0000);
                break;
            case M_SE:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SE_F, absolute);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SE_B, 0x0000);
                break;
            case M_SW:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SW_F, absolute);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SW_B, 0x0000);
                break;
            default:
                break;
        }
        return;
    } else {
        switch (m) {
            case M_NE:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NE_B, absolute);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NE_F, 0x0000);
                break;
            case M_NW:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NW_B, absolute);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_NW_F, 0x0000);
                break;
            case M_SE:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SE_B, absolute);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SE_F, 0x0000);
                break;
            case M_SW:
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SW_B, absolute);
                i2c_write_word(_ch_pca9685, CControlPi::motor_regs::MREG_SW_F, 0x0000);
                break;
            default:
                break;
        }
        return;
    }
}

bool CControlPi::hmc5883l_raw_data(std::vector<char> &data) {
    if (!_ready_hmc5883l) {
        spdlog::error("Device not ready.");
        return false;
    }
    if (!i2c_read_block(_ch_hmc5883l,0x03,data,6)) {
        return false;
    }
    return true;
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
    std::this_thread::sleep_until(std::chrono::system_clock::now() + std::chrono::milliseconds(1));
    }
}


bool CControlPi::get_i2c_status(i2c_ch ch) {
    switch (ch) {
        case i2c_ch::CH0:
            return _ready_i2c_ch0 && (_i2c_handle_ch0 >= 0);
        case i2c_ch::CH1:
            return _ready_i2c_ch1 && (_i2c_handle_ch1 >= 0);
        default:
            return false;
    }
}

int* CControlPi::get_i2c_handle(i2c_ch ch) {
    switch (ch) {
        case i2c_ch::CH0:
            return &_i2c_handle_ch0;
        case i2c_ch::CH1:
            return &_i2c_handle_ch1;
        default:
            return nullptr;
    }
}

bool CControlPi::i2c_write_byte(i2c_ch ch, uint reg, uint val) {
    if (!get_i2c_status(ch)) return false;
    if(i2cWriteByteData(*get_i2c_handle(ch),reg,val) != 0) {
        return false;
    }
    return true;
}

bool CControlPi::i2c_read_byte(i2c_ch ch, uint reg, uint8_t &data) {
    if (!get_i2c_status(ch)) return false;
    uint8_t raw_data = i2cReadByteData(*get_i2c_handle(ch),reg);
    if(raw_data < 0) {
        return false;
    }
    data = raw_data;
    return true;
}

bool CControlPi::i2c_read_block(i2c_ch ch, uint reg, std::vector<char> &buf, int bytes) {
    if (!get_i2c_status(ch)) return false;
    std::vector<char> temp_buf(bytes,'\1');
    if(i2cReadI2CBlockData(*get_i2c_handle(ch), reg, temp_buf.data(), bytes) <= 0) {
        return false;
    }
    buf = temp_buf;
    return true;
}

bool CControlPi::i2c_write_block(i2c_ch ch, uint reg, std::vector<char> &buf) {
    if (!get_i2c_status(ch)) return false;
    if(i2cWriteI2CBlockData(*get_i2c_handle(ch),reg,buf.data(), buf.size()) != 0) {
        return false;
    }
    return true;
}

bool CControlPi::i2c_write_word(i2c_ch ch, uint reg, uint word) {
    if (!get_i2c_status(ch)) return false;
    if(i2cWriteWordData(*get_i2c_handle(ch),reg,word) != 0) {
        return false;
    }
    return true;
}