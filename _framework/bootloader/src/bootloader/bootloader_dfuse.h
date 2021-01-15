#ifndef __DFUSE_H__
    #define __DFUSE_H__ 

    #include <stdint.h>
    #include <stdbool.h>
    
    #include <libopencm3/usb/usbd.h>

    #include "bootloader_config.h"
    #include "bootloader_usb.h"

    /* Commands sent with wBlockNum == 0 as per ST implementation. */
    #define CMD_SETADDR	0x21
    #define CMD_ERASE	0x41

    uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout);
    
    void usbdfu_getstatus_complete(struct usb_setup_data *req);

    enum usbd_request_return_codes usbdfu_control_request(
        usbd_device *usbd_devicezz,
        struct usb_setup_data *req, 
        uint8_t **buf, 
        uint16_t *len,
        usbd_control_complete_callback *complete
    );

    void erase_remaining_sectors(void);

#endif