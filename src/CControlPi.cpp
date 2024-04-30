/**
 * CControlPi.cpp - GPIO control on the Raspberry Pi
 * 2024-02-29
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CControlPi.hpp"

// TODO: initialize compass/orientation sensor
CControlPi::CControlPi() {
    _ready_gpio = false;
    _ready_i2c = false;
    _ready_pca9685 = false;
}

CControlPi::~CControlPi() {
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


//bool CControlPi::get_data(data_type type, int channel, int &result) {
//    switch (type) {
//        case data_type::ANALOG:
//            return code_talker((adc_channel) channel, result);
//        case data_type::DIGITAL:
//            result = gpioRead(channel);
//            return true;
//        case data_type::SERVO:
//            result = _servo_pos;
//            return true;
//        default:
//            return false;
//    }
//}
//
//bool CControlPi::set_data(data_type type, int channel, int val) {
//    switch (type) {
//        case data_type::ANALOG:
//            std::cout << "Analog write not supported." << std::endl;
//            return false;
//        case data_type::DIGITAL:
//            return !gpioWrite(channel, val);
//        case data_type::SERVO:
//            return servo_talker(data_op::WRITE, channel, val, _servo_result);
//        default:
//            return false;
//    }
//}
//
//// TODO: add/change functions to produce four hardware timer backed pwm outputs
//bool CControlPi::servo_talker(CControlPi::data_op op, int channel, int set_val, int &result) {
//    switch (op) {
//        case data_op::WRITE:
//            _servo_pos = set_val;
//            _servo_pos_converted = (int) ((_servo_pos / 180.0) * 200000) + 50000;
//            return !gpioHardwarePWM(channel, CControlPi::servo::PWM_FREQ, _servo_pos_converted);
//        case data_op::READ:
//            result = _servo_pos;
//            return true;
//        default:
//            return false;
//    }
//}
//
//bool CControlPi::code_talker(CControlPi::adc_channel ch, int &result) {
//    char cmd[] = { 1, (uint8_t) ch, 0};
//    int handle = spiOpen(0, 200000, 3);
//    bool got_data = spiXfer(handle, cmd, (char*) input_buffer, 3);
//    result = ((input_buffer[1] & 3) << 8) | input_buffer[2];
//    spiClose(handle);
//    return got_data;
//}