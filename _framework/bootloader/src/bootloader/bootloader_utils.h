#ifndef __UTILS_H__
#define __UTILS_H__

    #include <stddef.h>

    #include "bootloader_config.h"
    
    void system_hard_reset(void);
    void jump_to_application(uint32_t app_entry);
    void *memcpy(void * dst, const void * src, size_t count);
    void atoh(char *ascii_ptr, char *hex_ptr,int len);
    size_t strlen(const char *s);
    uint32_t getFlashSectorSize(uint8_t sector);
    void usb_hard_reset(void);
    void print_device_serial_number(char * s);
#endif
