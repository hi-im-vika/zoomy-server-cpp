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
#include <queue>

/**
 * @brief A class to provide physical input for the Raspberry Pi.
 * @author vika
 */
class CControlPi {
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
    enum i2c_devices {
        ADDR_DEFAULT_PCA9685 = 0x40,
        ADDR_DEFAULT_HMC5883L = 0x1E
    };

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

    bool _do_exit = false;

    // game controller
    std::thread _thread_process_gc;
    std::queue<std::string> _gc_queue;
    std::vector<int> _gc_values;

    void process_gc(std::string &gc);
    static void thread_process_gc(CControlPi *who_called);

    // i2c setup
    bool _ready_i2c_ch0;
    bool _ready_i2c_ch1;
    int _i2c_handle_ch0;
    int _i2c_handle_ch1;

    int* get_i2c_handle(i2c_ch ch);
    bool get_i2c_status(i2c_ch ch);
    bool i2c_write_byte(i2c_ch ch, uint reg, uint val);
    bool i2c_write_block(i2c_ch ch, uint reg, std::vector<char> &buf);
    bool i2c_write_word(i2c_ch ch, uint reg, uint word);

    bool i2c_read_byte(i2c_ch ch, uint reg, uint8_t &data);
    bool i2c_read_block(i2c_ch ch, uint reg, std::vector<char> &buf, int bytes);

    // i2c device specific
    // TODO: bundle into struct later
    bool _ready_pca9685;        ///< PCA9685 initialization status.
    int* _handle_pca9685;       ///< PCA9685 I2C handle pointer.
    i2c_ch _ch_pca9685;         ///< PCA9685 I2C channel on the Raspberry Pi.

    bool _ready_hmc5883l;       ///< HMC5883L initialization status.
    int* _handle_hmc5883l;      ///< HMC5883L I2C handle pointer.
    i2c_ch _ch_hmc5883l;        ///< HMC5883L I2C channel on the Raspberry Pi.

    // gpio setup
    bool _ready_gpio;           ///< GPIO initialization status.

public:

    CControlPi();
    ~CControlPi();

    bool init_gpio(const std::vector<int> &input_pins, std::vector<int> &output_pins);
    bool init_i2c(i2c_ch ch, uint address);

    bool init_pca9685(i2c_ch ch, uint address = ADDR_DEFAULT_PCA9685);
    bool init_hmc5883l(i2c_ch ch, uint address = ADDR_DEFAULT_HMC5883L);

    void zap_com();

    void pca9685_motor_control(motor m, int value);
    bool hmc5883l_raw_data(std::vector<char> &data);

    void queue_new_gc_data(std::string &data);

    std::vector<int> get_gc_values();
};
