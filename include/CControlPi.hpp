/**
 * CControlPi.h - GPIO control on the Raspberry Pi
 * 2024-02-29
 * vika <https://github.com/hi-im-vika>
 */

#pragma once


#include <iostream>
#include <vector>
#include <filesystem>
#include <pigpio.h>
#include <thread>

// Linux specific includes
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <sys/types.h>

/**
 * @brief A class to provide physical input for the Raspberry Pi.
 * @author vika
 */
class CControlPi {
private:
    int _servo_result;
    int _servo_pos;
    int _servo_pos_converted;
    uint8_t input_buffer[3];
public:
    enum gpio_pins {
        BUTTON1 = 17,
        BUTTON2 = 27,
        LED1 = 22,
        SERVO1 = 12,
        HEARTBEAT = 17,
        STEERING = 12,
        THROTTLE = 13
    };
    enum data_type {
        ANALOG,
        DIGITAL,
        SERVO
    };
    enum adc_channel {
        CH0 = 0x80,
        CH1 = 0x90,
    };
    enum data_op {
        READ,
        WRITE
    };
    enum servo {
        PWM_FREQ = 100,
    };
    CControlPi() = default;
    ~CControlPi() = default;
    void init();
    void init_gpio(const std::vector<int> &input_pins, std::vector<int> &output_pins);
    bool get_data(data_type type, int channel, int &result);
    bool set_data(data_type type, int channel, int val);
    bool servo_talker(CControlPi::data_op op, int channel, int set_val, int &result);
    bool code_talker(adc_channel ch, int &result);
    void zap_com();
};
