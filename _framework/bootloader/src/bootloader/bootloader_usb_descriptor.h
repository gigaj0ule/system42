#ifndef __USB_DESCRIPTOR_H__
    #define __USB_DESCRIPTOR_H__

    #include <stdint.h>
    #include <stdbool.h>
    #include <libopencm3/usb/dfu.h>

    #include "bootloader_config.h"
    #include "bootloader_usb.h"

	/*enum usbd_request_return_codes {
		USBD_REQ_NOTSUPP					= 0,
		USBD_REQ_HANDLED					= 1,
		USBD_REQ_NEXT_CALLBACK				= 2,
	};*/

    #define STR_HELPER(x) #x
    #define STR(x) STR_HELPER(x)

    struct iFlashMemoryPartitions_t {
        const char header[28];
        
        /*
        #ifndef BOOT_KEY_FILE
        const char  bootloader_size_kb[2];
        const char  bootloader_size_multiplier[1];
        const char  bootloader_transfer_size[1];    // B, K, M (bytes)
        const char  bootloader_transfer_unit[1];
        const char  bootloader_permissions[1];

        const char  partition_1[1];
        #endif
        */

        char        payload_size_kb[5];
        const char  payload_size_multiplier[1];
        char        payload_transfer_size[1];       // B, K, M (bytes)
        const char  payload_transfer_unit[1];
        const char  payload_permissions[1];

        const char  nullTerminator[1];
    } __attribute__((packed));

    /* Notes about the dfuse struct above:
    *  Format: /<start_address>/<number>*<page_size><multiplier><memtype>
    * 
    *  <number>: how many pages
    *  <page_size>: self explanatory
    *  <multiplier>: 'B'(ytes), 'K'(ilobytes), 'M'(egabytes)
    *  <memtype>: the bottom three bits are significant:
    *             writeable|erasable|readable
    * subsequent blocks separated by commas
    *
    * Using the internal page size: "@Internal Flash /0x08000000/64*128Ba,448*128Bg"
    * Using 1K blocks: "@Internal Flash   /0x08000000/8*001Ka,56*001Kg"
    */

    typedef struct {
        struct usb_config_descriptor config;
        struct usb_interface_descriptor iface;
        struct usb_dfu_descriptor dfu_function;
    } config_desc_t;

#endif