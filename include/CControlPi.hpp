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
    // game controller
    std::vector<int> _values;

    // i2c setup
    bool _ready_i2c;

    // gpio setup
    bool _ready_gpio;
public:
    enum i2c_ch {
        CH0 = 0,
        CH1 = 1
    };
    CControlPi() = default;
    ~CControlPi() = default;
    bool init_gpio(const std::vector<int> &input_pins, std::vector<int> &output_pins);
    bool init_i2c(i2c_ch ch, uint address);
//    bool get_data(data_type type, int channel, int &result);
//    bool set_data(data_type type, int channel, int val);
//    bool servo_talker(CControlPi::data_op op, int channel, int set_val, int &result);
//    bool code_talker(adc_channel ch, int &result);
    void zap_com();
};
