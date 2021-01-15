
#include <stdint.h>

#include <libopencm3/usb/usbstd.h>

#include "bootloader_config.h"
#include "bootloader_usb.h"
#include "bootloader_usb_descriptor.h"
#include "bootloader_tests.h"

// =========================================================================
// DFUse Memory Partitions
struct iFlashMemoryPartitions_t iFlashMemoryPartitions = {
    .header                      = "@Internal Flash /" STR(FLASH_BASE_ADDR) "/",

    #ifndef ENABLE_AES_DECRYPTION
    .bootloader_size_kb          = STR(FLASH_BOOTLDR_SIZE_KB),
    .bootloader_size_multiplier  = {'*'},
    .bootloader_transfer_size    = {'1'},
    .bootloader_transfer_unit    = {'K'},
    //.bootloader_permissions  = {'g'},
    //#else
    .bootloader_permissions      = {'a'},
    .partition_1                 = {','},
    #endif
    .payload_size_kb             = "00000",
    .payload_size_multiplier     = {'*'},
    .payload_transfer_size       = {'1'},
    .payload_transfer_unit       = {'K'},
    .payload_permissions         = {'g'},
    .nullTerminator              = {0x00}
};

// =========================================================================
// String Descriptors
volatile const char iSerial[]    = "000000000000";
volatile const char appSize[]    = "000";
const char iManufacturer[]       = I_MANUFACTURER;
const char iProduct[]            = I_PRODUCT;

const char * const _usb_strings[4] = {
    iManufacturer,                                   // iManufacturer
    iProduct,                                        // iProduct
    (const char *)iSerial,                           // iSerialNumber
    (const char *)&iFlashMemoryPartitions,           // This string is used by ST Microelectronics' DfuSe utility.
};

const uint8_t _num_strings = 4;


// =========================================================================
// USB Device Descriptor
const struct usb_device_descriptor dev_desc = {
    .bLength                = USB_DT_DEVICE_SIZE,    // bLength
    .bDescriptorType        = USB_DT_DEVICE,         // bDescriptorType
    .bcdUSB                 = 0x0200,                // bcdUSB, version 2.00
    .bDeviceClass           = 0,                     // bDeviceClass : See interface
    .bDeviceSubClass        = 0,                     // bDeviceSubClass : See interface
    .bDeviceProtocol        = 0,                     // bDeviceProtocol : See interface
    .bMaxPacketSize0        = 64,                    // bMaxPacketSize0 0x40 = 64
    .idVendor               = ID_VENDOR,             // idVendor
    .idProduct              = ID_PRODUCT,            // idProduct
    .bcdDevice              = 0x0200,                // bcdDevice
    .iManufacturer          = 1,                     // iManufacturer : index of string Manufacturer 
    .iProduct               = 2,                     // iProduct      : index of string descriptor of product
    .iSerialNumber          = 3,                     // iSerialNumber : index of string serial number
    .bNumConfigurations     = 1,                     // bNumConfigurations
};


// =========================================================================
// DFU Functional Descriptor
#define dummyTransferSize 0x1000

struct usb_dfu_descriptor dfu_function = {
    .bLength                = sizeof(struct usb_dfu_descriptor),            // blength = 7 Bytes
    .bDescriptorType        = DFU_FUNCTIONAL,                               // DFU Functional Descriptor
    .bmAttributes           = USB_DFU_CAN_DOWNLOAD | USB_DFU_WILL_DETACH,   // bmAttributes, bitCanDnload | bitCanUpload
    .wDetachTimeout         = 255,                                          // DetachTimeOut= 255 ms
    .wTransferSize          = dummyTransferSize,                            // TransferSize = 1024 Byte
    .bcdDFUVersion          = 0x011A,                                       // bcdDFUVersion = 1.1
};


// =========================================================================
// DFU Interface Descriptor
const struct usb_interface_descriptor dfu_iface = {
    .bLength                = USB_DT_INTERFACE_SIZE, // bLength: Interface Descriptor size
    .bDescriptorType        = USB_DT_INTERFACE,      // bDescriptorType:
    .bInterfaceNumber       = 0,                     // bInterfaceNumber: Number of Interface
    .bAlternateSetting      = 0,                     // bAlternateSetting: Alternate setting 0
    .bNumEndpoints          = 0,                     // bNumEndpoints
    .bInterfaceClass        = 0xFE,                  // Device Firmware Upgrade
    .bInterfaceSubClass     = 1,                     // bInterfaceSubClass
    .bInterfaceProtocol     = 2,                     // nInterfaceProtocol, switched to 0x02 while in dfu_mode
    .iInterface             = 4,                     // iInterface: string #4

    .endpoint = NULL,
    .extra = &dfu_function,
    .extralen = sizeof(dfu_function)
};


// =========================================================================
// USB Interfaces
const struct usb_interface ifaces[] = {
    {
        .num_altsetting = 1,
        .altsetting = &dfu_iface,
    }
};


// =========================================================================
// USB Configuation Descriptor
const struct usb_config_descriptor usb_configuration_descriptor = {

	// USB Device descriptor
    .bLength                = USB_DT_CONFIGURATION_SIZE,             // bLength: Configuation Descriptor size
    .bDescriptorType        = USB_DT_CONFIGURATION,                  // bDescriptorType: Configuration
    .wTotalLength           = sizeof(usb_configuration_descriptor),  // wTotalLength: Bytes returned
    .bNumInterfaces         = 1,                                     // bNumInterfaces: 1 interface
    .bConfigurationValue    = 1,                                     // bConfigurationValue:
    .iConfiguration         = 4,                                     // iConfiguration:
    .bmAttributes           = 0xC0,                                  // bmAttributes:
    .bMaxPower              = 0x32,                                  // bMaxPower: 100 mA
    // 09

    .interface = ifaces
};