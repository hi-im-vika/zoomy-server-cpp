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
    enum motor_regs {
        MREG_NE_F = 0x08,
        MREG_NE_B = 0x0C,
        MREG_NW_B = 0x10,
        MREG_NW_F = 0x14,
        MREG_SE_F = 0x18,
        MREG_SE_B = 0x1C,
        MREG_SW_B = 0x20,
        MREG_SW_F = 0x24,
    };

    // game controller
    std::vector<int> _values;

    // i2c setup
    bool _ready_i2c;

    // gpio setup
    bool _ready_gpio;
public:
    enum motor {
        M_NE,
        M_NW,
        M_SE,
        M_SW
    };
    enum i2c_ch {
        CH0 = 0,
        CH1 = 1
    };
    CControlPi() = default;
    ~CControlPi() = default;
    enum i2c_devices {
        ADDR_DEFAULT_PCA9685 = 0x40
    };

    bool init_gpio(const std::vector<int> &input_pins, std::vector<int> &output_pins);
    bool init_i2c(i2c_ch ch, uint address);
    void zap_com();

    bool i2c_write_byte(uint reg, uint val);
    bool i2c_read_byte(uint reg, uint8_t &data);
    bool i2c_write_block(uint reg, std::vector<char> &buf);
    bool i2c_write_word(uint reg, uint word);

    void pca9685_motor_control(motor m, int value);
};
