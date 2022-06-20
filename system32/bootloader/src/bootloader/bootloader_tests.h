#ifndef __TESTS_H__

    #define __TESTS_H__

    #include <stdint.h>
    #include <stdbool.h>

    #define FLASH_SIZE_REG 0x1FFFF7E0

    bool dfu_boot_token(void);
    bool test_if_user_app_is_missing(uint32_t test_address);
    bool test_button_state(void);
    int8_t check_set_device_unique(void);
    void check_set_rdp(void);
    void get_device_serial_number(char *s);
    uint64_t read_device_serial_number();

#endif