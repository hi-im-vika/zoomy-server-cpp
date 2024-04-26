/**
 * CControlPi.cpp - GPIO control on the Raspberry Pi
 * 2024-02-29
 * vika <https://github.com/hi-im-vika>
 */

#include "../include/CControlPi.h"

// TODO: initialize compass/orientation sensor
void CControlPi::init() {
    gpioInitialise();
}

void CControlPi::zap_com() {
    gpioTerminate();
}

void CControlPi::init_gpio(const std::vector<int> &input_pins, std::vector<int> &output_pins) {
    if (gpioInitialise() < 0) {
        std::cout << "Error during GPIO init" << std::endl;
        exit(-1);
    }
    if (!input_pins.empty()) {
        for (auto i: input_pins) {
            gpioSetMode(i, PI_INPUT);
        }
    }
    if (!output_pins.empty()) {
        for (auto i: output_pins) {
            gpioSetMode(i, PI_OUTPUT);
        }
    }
}

bool CControlPi::get_data(data_type type, int channel, int &result) {
    switch (type) {
        case data_type::ANALOG:
            return code_talker((adc_channel) channel, result);
        case data_type::DIGITAL:
            result = gpioRead(channel);
            return true;
        case data_type::SERVO:
            result = _servo_pos;
            return true;
        default:
            return false;
    }
}

bool CControlPi::set_data(data_type type, int channel, int val) {
    switch (type) {
        case data_type::ANALOG:
            std::cout << "Analog write not supported." << std::endl;
            return false;
        case data_type::DIGITAL:
            return !gpioWrite(channel, val);
        case data_type::SERVO:
            return servo_talker(data_op::WRITE, channel, val, _servo_result);
        default:
            return false;
    }
}

// TODO: add/change functions to produce four hardware timer backed pwm outputs
bool CControlPi::servo_talker(CControlPi::data_op op, int channel, int set_val, int &result) {
    switch (op) {
        case data_op::WRITE:
            _servo_pos = set_val;
            _servo_pos_converted = (int) ((_servo_pos / 180.0) * 200000) + 50000;
            return !gpioHardwarePWM(channel, CControlPi::servo::PWM_FREQ, _servo_pos_converted);
        case data_op::READ:
            result = _servo_pos;
            return true;
        default:
            return false;
    }
}

bool CControlPi::code_talker(CControlPi::adc_channel ch, int &result) {
    char cmd[] = { 1, (uint8_t) ch, 0};
    int handle = spiOpen(0, 200000, 3);
    bool got_data = spiXfer(handle, cmd, (char*) input_buffer, 3);
    result = ((input_buffer[1] & 3) << 8) | input_buffer[2];
    spiClose(handle);
    return got_data;
}