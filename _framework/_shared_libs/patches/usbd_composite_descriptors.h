#if defined(USBD_USE_HID_COMPOSITE)
#include "usbd_hid_composite.h"
#endif

#if defined(USBD_USE_CDC)
#include "usbd_cdc.h"
#endif

#if defined(USBD_USE_AUDIO)
#include "usbd_audio.h"
#endif

#include "usbd_ep_conf.h"

// USB HID device FS Configuration Descriptor

// --------------------------------------------------------------------------
// Device Descriptor!
typedef struct {

    /* Configuration Descriptor */
    uint8_t  cfgDesc_bLength;
    uint8_t  cfgDesc_bDescriptorType;
    uint16_t cfgDesc_wTotalLength;
    uint8_t  cfgDesc_bNumInterfaces;
    uint8_t  cfgDesc_bConfigurationValue;
    uint8_t  cfgDesc_iConfiguration;
    uint8_t  cfgDesc_bmAttributes;
    uint8_t  cfgDesc_bMaxPower;

    #if defined(USBD_USE_CDC)

    // CDC Interface Descriptor (9)
    uint8_t CDC_IF_DESC_bLength;
    uint8_t CDC_IF_DESC_bDescriptorType;
    uint8_t CDC_IF_DESC_bInterfaceNumber;
    uint8_t CDC_IF_DESC_bAlternateSetting;
    uint8_t CDC_IF_DESC_bNumEndpoints;
    uint8_t CDC_IF_DESC_bInterfaceClass;
    uint8_t CDC_IF_DESC_bInterfaceSubClass;
    uint8_t CDC_IF_DESC_bInterfaceProtocol;
    uint8_t CDC_IF_DESC_iInterface;

    // Header Functional Descriptor (5)
    uint8_t  CDC_IF_HDR_F_DESC_bLength;
    uint8_t  CDC_IF_HDR_F_DESC_bDescriptorType;
    uint8_t  CDC_IF_HDR_F_DESC_bDescriptorSubtype;
    uint16_t CDC_IF_HDR_F_DESC_bcdCDC;

    // Call Management Functional Descriptor (5)
    uint8_t CDC_IF_CM_F_DESC_bFunctionLength;
    uint8_t CDC_IF_CM_F_DESC_bDescriptorType;
    uint8_t CDC_IF_CM_F_DESC_bDescriptorSubtype;
    uint8_t CDC_IF_CM_F_DESC_bmCapabilities;
    uint8_t CDC_IF_CM_F_DESC_bDataInterface;

    // ACM Functional Descriptor (4)
    uint8_t CDC_IF_ACM_F_DESC_bFunctionLength;
    uint8_t CDC_IF_ACM_F_DESC_bDescriptorType;
    uint8_t CDC_IF_ACM_F_DESC_bDescriptorSubtype;
    uint8_t CDC_IF_ACM_F_DESC_bmCapabilities;

    // Union Functional Descriptor (5)
    uint8_t CDC_IF_UFD_DESC_bFunctionLength;
    uint8_t CDC_IF_UFD_DESC_bDescriptorType;
    uint8_t CDC_IF_UFD_DESC_bDescriptorSubtype;
    uint8_t CDC_IF_UFD_DESC_bMasterInterface;
    uint8_t CDC_IF_UFD_DESC_bSlaveInterface0;

    // CDC CMD EP Descriptor (7)
    uint8_t  CDC_IF_CMD_EP_DESC_bLength;
    uint8_t  CDC_IF_CMD_EP_DESC_bDescriptorType;
    uint8_t  CDC_IF_CMD_EP_DESC_bEndpointAddress;
    uint8_t  CDC_IF_CMD_EP_DESC_bmAttributes;
    uint16_t CDC_IF_CMD_EP_DESC_wMaxPacketSize;
    uint8_t  CDC_IF_CMD_EP_DESC_bInterval;

    // CDC Data class interface descriptor (9)
    uint8_t CDC_IF_DCIF_DESC_bLength;
    uint8_t CDC_IF_DCIF_DESC_bDescriptorType;
    uint8_t CDC_IF_DCIF_DESC_bInterfaceNumber;
    uint8_t CDC_IF_DCIF_DESC_bAlternateSetting;
    uint8_t CDC_IF_DCIF_DESC_bNumEndpoints;
    uint8_t CDC_IF_DCIF_DESC_bInterfaceClass;
    uint8_t CDC_IF_DCIF_DESC_bInterfaceSubClass;
    uint8_t CDC_IF_DCIF_DESC_bInterfaceProtocol;
    uint8_t CDC_IF_DCIF_DESC_iInterface;

    // CDC Endpoint OUT Descriptor (7)
    uint8_t  CDC_IF_OUT_EP_DESC_bLength;
    uint8_t  CDC_IF_OUT_EP_DESC_bDescriptorType;
    uint8_t  CDC_IF_OUT_EP_DESC_bEndpointAddress;
    uint8_t  CDC_IF_OUT_EP_DESC_bmAttributes;
    uint16_t CDC_IF_OUT_EP_DESC_wMaxPacketSize;
    uint8_t  CDC_IF_OUT_EP_DESC_bInterval;

    // CDC Endpoint IN Descriptor (7)
    uint8_t  CDC_IF_IN_EP_DESC_bLength;
    uint8_t  CDC_IF_IN_EP_DESC_bDescriptorType;
    uint8_t  CDC_IF_IN_EP_DESC_bEndpointAddress;
    uint8_t  CDC_IF_IN_EP_DESC_bmAttributes;
    uint16_t CDC_IF_IN_EP_DESC_wMaxPacketSize;
    uint8_t  CDC_IF_IN_EP_DESC_bInterval;
    #endif

    // -----------------------------------
    // -----------------------------------
    #if defined(USBD_USE_HID_COMPOSITE)

    // Joystick / Mouse Interface Descriptor (9)
    uint8_t HID_MOUSE_IF_DESC_bLength;
    uint8_t HID_MOUSE_IF_DESC_bDescriptorType;
    uint8_t HID_MOUSE_IF_DESC_bInterfaceNumber;
    uint8_t HID_MOUSE_IF_DESC_bAlternateSetting;
    uint8_t HID_MOUSE_IF_DESC_bNumEndpoints;
    uint8_t HID_MOUSE_IF_DESC_bInterfaceClass;
    uint8_t HID_MOUSE_IF_DESC_bInterfaceSubClass;
    uint8_t HID_MOUSE_IF_DESC_bInterfaceProtocol;
    uint8_t HID_MOUSE_IF_DESC_iInterface;

    // Joystick / Mouse HID Report Descriptor (9)
    uint8_t HID_MOUSE_IF_REP_DESC_bLength;
    uint8_t HID_MOUSE_IF_REP_DESC_bDescriptorType;
    uint16_t HID_MOUSE_IF_REP_DESC_bcdHID;
    uint8_t HID_MOUSE_IF_REP_DESC_bCountryCode;
    uint8_t HID_MOUSE_IF_REP_DESC_bNumDescriptors;
    uint8_t HID_MOUSE_IF_REP_DESC_bDescriptorType2;
    uint16_t HID_MOUSE_IF_REP_DESC_wItemLength;

    // Joystick / Mouse HID Endpoint IN Descriptor (7)
    uint8_t HID_MOUSE_IF_IN_EP_DESC_bLength;
    uint8_t HID_MOUSE_IF_IN_EP_DESC_bDescriptorType;
    uint8_t HID_MOUSE_IF_IN_EP_DESC_bEndpointAddress;
    uint8_t HID_MOUSE_IF_IN_EP_DESC_bmAttributes;
    uint16_t HID_MOUSE_IF_IN_EP_DESC_wMaxPacketSize;
    uint8_t HID_MOUSE_IF_IN_EP_DESC_bInterval;

    // Keyboard Interface Descriptor (9)
    uint8_t HID_KB_IF_DESC_bLength;
    uint8_t HID_KB_IF_DESC_bDescriptorType;
    uint8_t HID_KB_IF_DESC_bInterfaceNumber;
    uint8_t HID_KB_IF_DESC_bAlternateSetting;
    uint8_t HID_KB_IF_DESC_bNumEndpoints;
    uint8_t HID_KB_IF_DESC_bInterfaceClass;
    uint8_t HID_KB_IF_DESC_bInterfaceSubClass;
    uint8_t HID_KB_IF_DESC_bInterfaceProtocol;
    uint8_t HID_KB_IF_DESC_iInterface;

    // Keyboard HID Report Descriptor (9)
    uint8_t HID_KB_IF_REPT_DESC_bLength;
    uint8_t HID_KB_IF_REPT_DESC_bDescriptorType;
    uint16_t HID_KB_IF_REPT_DESC_bcdHID;
    uint8_t HID_KB_IF_REPT_DESC_bCountryCode;
    uint8_t HID_KB_IF_REPT_DESC_bNumDescriptors;
    uint8_t HID_KB_IF_REPT_DESC_bDescriptorType2;
    uint16_t HID_KB_IF_REPT_DESC_wItemLength;

    // Keyboard Endpoint IN Descriptor (7)
    uint8_t HID_KB_IF_IN_EP_DESC_bLength;
    uint8_t HID_KB_IF_IN_EP_DESC_bDescriptorType;
    uint8_t HID_KB_IF_IN_EP_DESC_bEndpointAddress;
    uint8_t HID_KB_IF_IN_EP_DESC_bmAttributes;
    uint16_t HID_KB_IF_IN_EP_DESC_wMaxPacketSize;
    uint8_t HID_KB_IF_IN_EP_DESC_bInterval;

    #endif

    // --------------------------------------------------------------
    // --------------------------------------------------------------
    #ifdef USBD_USE_AUDIO

    // USB Speaker Standard interface descriptor (9)
    uint8_t AUDIO_IF_DESC_bLength;
    uint8_t AUDIO_IF_DESC_bDescriptorType;
    uint8_t AUDIO_IF_DESC_bInterfaceNumber;
    uint8_t AUDIO_IF_DESC_bAlternateSetting;
    uint8_t AUDIO_IF_DESC_bNumEndpoints;
    uint8_t AUDIO_IF_DESC_bInterfaceClass;
    uint8_t AUDIO_IF_DESC_bInterfaceSubClass;
    uint8_t AUDIO_IF_DESC_bInterfaceProtocol;
    uint8_t AUDIO_IF_DESC_iInterface;

    // USB Speaker Class-specific AC Interface Descriptor (9)
    uint8_t AUDIO_SPKR_IF_DESC_bLength;
    uint8_t AUDIO_SPKR_IF_DESC_bDescriptorType;
    uint8_t AUDIO_SPKR_IF_DESC_bDescriptorSubtype;
    uint16_t AUDIO_SPKR_IF_DESC_bcdADC;
    uint16_t AUDIO_SPKR_IF_DESC_wTotalLength;
    uint8_t AUDIO_SPKR_IF_DESC_bInCollection;
    uint8_t AUDIO_SPKR_IF_DESC_baInterfaceNr;

    // USB Speaker Input Terminal Descriptor (12)
    uint8_t AUDIO_SPKR_IN_TM_DESC_bLength;
    uint8_t AUDIO_SPKR_IN_TM_DESC_bDescriptorType;
    uint8_t AUDIO_SPKR_IN_TM_DESC_bDescriptorSubtype;
    uint8_t AUDIO_SPKR_IN_TM_DESC_bTerminalID;
    uint16_t AUDIO_SPKR_IN_TM_DESC_wTerminalType;
    uint8_t AUDIO_SPKR_IN_TM_DESC_bAssocTerminal;
    uint8_t AUDIO_SPKR_IN_TM_DESC_bNrChannels;
    uint16_t AUDIO_SPKR_IN_TM_DESC_wChannelConfig;
    uint8_t AUDIO_SPKR_IN_TM_DESC_iChannelNames;
    uint8_t AUDIO_SPKR_IN_TM_DESC_iTerminal;

    // USB Speaker Audio Feature Unit Descriptor (9)
    uint8_t AUDIO_SPKR_AFU_DESC_bLength;
    uint8_t AUDIO_SPKR_AFU_DESC_bDescriptorType;
    uint8_t AUDIO_SPKR_AFU_DESC_bDescriptorSubtype;
    uint8_t AUDIO_SPKR_AFU_DESC_bUnitID;
    uint8_t AUDIO_SPKR_AFU_DESC_bSourceID;
    uint8_t AUDIO_SPKR_AFU_DESC_bControlSize;
    uint8_t AUDIO_SPKR_AFU_DESC_bmaControls0;
    uint8_t AUDIO_SPKR_AFU_DESC_bmaControls1;
    uint8_t AUDIO_SPKR_AFU_DESC_iTerminal;

    // USB Speaker Output Terminal Descriptor (9)
    uint8_t AUDIO_SPKR_OUT_TM_DESC_bLength;
    uint8_t AUDIO_SPKR_OUT_TM_DESC_bDescriptorType;
    uint8_t AUDIO_SPKR_OUT_TM_DESC_bDescriptorSubtype;
    uint8_t AUDIO_SPKR_OUT_TM_DESC_bTerminalID;
    uint16_t AUDIO_SPKR_OUT_TM_DESC_wTerminalType;
    uint8_t AUDIO_SPKR_OUT_TM_DESC_bAssocTerminal;
    uint8_t AUDIO_SPKR_OUT_TM_DESC_bSourceID;
    uint8_t AUDIO_SPKR_OUT_TM_DESC_iTerminal;
    
    // USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith (9)
    uint8_t AUDIO_SPKR_AS_IF0_DESC_bLength;
    uint8_t AUDIO_SPKR_AS_IF0_DESC_bDescriptorType;
    uint8_t AUDIO_SPKR_AS_IF0_DESC_bInterfaceNumber;
    uint8_t AUDIO_SPKR_AS_IF0_DESC_bAlternateSetting;
    uint8_t AUDIO_SPKR_AS_IF0_DESC_bNumEndpoints;
    uint8_t AUDIO_SPKR_AS_IF0_DESC_bInterfaceClass;
    uint8_t AUDIO_SPKR_AS_IF0_DESC_bInterfaceSubClass;
    uint8_t AUDIO_SPKR_AS_IF0_DESC_bInterfaceProtocol;
    uint8_t AUDIO_SPKR_AS_IF0_DESC_iInterface;

    // USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith (9)
    uint8_t AUDIO_SPKR_AS_IF1_DESC_bLength;
    uint8_t AUDIO_SPKR_AS_IF1_DESC_bDescriptorType;
    uint8_t AUDIO_SPKR_AS_IF1_DESC_bInterfaceNumber;
    uint8_t AUDIO_SPKR_AS_IF1_DESC_bAlternateSetting;
    uint8_t AUDIO_SPKR_AS_IF1_DESC_bNumEndpoints;
    uint8_t AUDIO_SPKR_AS_IF1_DESC_bInterfaceClass;
    uint8_t AUDIO_SPKR_AS_IF1_DESC_bInterfaceSubClass;
    uint8_t AUDIO_SPKR_AS_IF1_DESC_bInterfaceProtocol;
    uint8_t AUDIO_SPKR_AS_IF1_DESC_iInterface;

    // USB Speaker Audio Streaming Interface Descriptor (7)
    uint8_t AUDIO_SPKR_AUD_ST_IF_DESC_bLength;
    uint8_t AUDIO_SPKR_AUD_ST_IF_DESC_bDescriptorType;
    uint8_t AUDIO_SPKR_AUD_ST_IF_DESC_bDescriptorSubtype;
    uint8_t AUDIO_SPKR_AUD_ST_IF_DESC_bTerminalLink;
    uint8_t AUDIO_SPKR_AUD_ST_IF_DESC_bDelay;
    uint16_t AUDIO_SPKR_AUD_ST_IF_DESC_wFormatTag;

    // USB Speaker Audio Type III Format Interface Descriptor (11)
    uint8_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_bLength;
    uint8_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_bDescriptorType;
    uint8_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_bDescriptorSubtype;
    uint8_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_bFormatType;
    uint8_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_bNrChannels;
    uint8_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_bSubFrameSize;
    uint8_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_bBitResolution;
    uint8_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_bSamFreqType;
    uint16_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_tSamFreqLowestHalfWord;
    uint8_t AUDIO_SPKR_AUD_T3FMT_IF_DESC_tSamFreqHighestByte;

    // Audio Endpoint 1 - Standard Descriptor (9)
    uint8_t AUD_SPKR_OUT_EP_DESC_bLength;
    uint8_t AUD_SPKR_OUT_EP_DESC_bDescriptorType;
    uint8_t AUD_SPKR_OUT_EP_DESC_bEndpointAddress;
    uint8_t AUD_SPKR_OUT_EP_DESC_bmAttributes;
    uint16_t AUD_SPKR_OUT_EP_DESC_wMaxPacketSize;
    uint8_t AUD_SPKR_OUT_EP_DESC_bInterval;
    uint8_t AUD_SPKR_OUT_EP_DESC_bRefresh;
    uint8_t AUD_SPKR_OUT_EP_DESC_bSynchAddress;

    // Endpoint - Audio Streaming Descriptor (7)
    uint8_t AUD_STRM_GEN_EP_DESC_bLength;
    uint8_t AUD_STRM_GEN_EP_DESC_bDescriptorType;
    uint8_t AUD_STRM_GEN_EP_DESC_bDescriptor;
    uint8_t AUD_STRM_GEN_EP_DESC_bmAttributes;
    uint8_t AUD_STRM_GEN_EP_DESC_bLockDelayUnits;
    uint16_t AUD_STRM_GEN_EP_DESC_wLockDelay;
    #endif

} __attribute__((packed)) COMPOSITE_DESCRIPTOR_t;


/* USB CDC device Configuration Descriptor */
COMPOSITE_DESCRIPTOR_t COMPOSITE_DESCRIPTOR = {

    // --------------------------------------------------------------
    // Configuration Descriptor

    // bLength: Configuration Descriptor size
    .cfgDesc_bLength = 0x09,
    // bDescriptorType: Configuration
    .cfgDesc_bDescriptorType = USB_DESC_TYPE_CONFIGURATION,
    // wTotalLength:no of returned bytes
    .cfgDesc_wTotalLength = sizeof(COMPOSITE_DESCRIPTOR_t),
    // bNumInterfaces:
    .cfgDesc_bNumInterfaces = DEV_NUM_IFACES,
    // bConfigurationValue: Configuration value
    .cfgDesc_bConfigurationValue = 0x01,
    // iConfiguration: Index of string descriptor describing the configuration
    .cfgDesc_iConfiguration = 0x00,
    // bmAttributes: self powered
    .cfgDesc_bmAttributes = 0xC0,
    // bMaxPower 0 mA
    .cfgDesc_bMaxPower = 0x32,


    #if defined(USBD_USE_CDC)
    // --------------------------------------------------------------
    // CDC Interface Descriptor

    // bLength: Interface Descriptor size (9)
    .CDC_IF_DESC_bLength = 0x09,
    // bDescriptorType: Interface
    .CDC_IF_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    // bInterfaceNumber: Number of Interface
    .CDC_IF_DESC_bInterfaceNumber = USBD_CDC_CMD_INTERFACE,
    // bAlternateSetting: Alternate setting
    .CDC_IF_DESC_bAlternateSetting = 0x00,
    // bNumEndpoints: One endpoints used
    .CDC_IF_DESC_bNumEndpoints = 0x01,
    // bInterfaceClass: Communication Interface Class
    .CDC_IF_DESC_bInterfaceClass = 0x02,
    // bInterfaceSubClass: Abstract Control Model
    .CDC_IF_DESC_bInterfaceSubClass = 0x02,
    // bInterfaceProtocol: Common AT commands
    .CDC_IF_DESC_bInterfaceProtocol = 0x00,
    // iInterface: (string ID)
    .CDC_IF_DESC_iInterface = 0x00,

    // --------------------------------------------------------------
    // Header Functional Descriptor

    // bLength: Endpoint Descriptor size (5)
    .CDC_IF_HDR_F_DESC_bLength = 0x05,
    // bDescriptorType: CS_INTERFACE
    .CDC_IF_HDR_F_DESC_bDescriptorType = 0x24,
    // bDescriptorSubtype: Header Func Desc
    .CDC_IF_HDR_F_DESC_bDescriptorSubtype = 0x00,
    // bcdCDC: spec release number
    .CDC_IF_HDR_F_DESC_bcdCDC = 0x0110,

    // --------------------------------------------------------------
    // Call Management Functional Descriptor

    // bFunctionLength (5)
    .CDC_IF_CM_F_DESC_bFunctionLength = 0x05,
    // bDescriptorType: CS_INTERFACE
    .CDC_IF_CM_F_DESC_bDescriptorType = 0x24,
    // bDescriptorSubtype: Call Management Func Desc
    .CDC_IF_CM_F_DESC_bDescriptorSubtype = 0x01,
    // bmCapabilities:
    .CDC_IF_CM_F_DESC_bmCapabilities = 0b00,
    // bDataInterface: 1
    .CDC_IF_CM_F_DESC_bDataInterface = 0x00,

    // --------------------------------------------------------------
    // ACM Functional Descriptor

    // bFunctionLength (4)
    .CDC_IF_ACM_F_DESC_bFunctionLength = 0x04,
    // bDescriptorType: CS_INTERFACE
    .CDC_IF_ACM_F_DESC_bDescriptorType = 0x24,
    // bDescriptorSubtype: Abstract Control Management desc
    .CDC_IF_ACM_F_DESC_bDescriptorSubtype = 0x02,
    // bmCapabilities
    .CDC_IF_ACM_F_DESC_bmCapabilities = 0b0000,//

    // --------------------------------------------------------------
    // Union Functional Descriptor

    // bFunctionLength (5)
    .CDC_IF_UFD_DESC_bFunctionLength = 0x05,
    // bDescriptorType: CS_INTERFACE
    .CDC_IF_UFD_DESC_bDescriptorType = 0x24,
    // bDescriptorSubtype: Union func desc
    .CDC_IF_UFD_DESC_bDescriptorSubtype = 0x06,
    // bMasterInterface: Communication class interface
    .CDC_IF_UFD_DESC_bMasterInterface = 0x00,
    // bSlaveInterface0: Data Class Interface
    .CDC_IF_UFD_DESC_bSlaveInterface0 = 0x01,

    // --------------------------------------------------------------
    // CDC CMD EP Descriptor

    // bLength: (7) Endpoint Descriptor size
    .CDC_IF_CMD_EP_DESC_bLength = 0x07,
    // bDescriptorType: Endpoint
    .CDC_IF_CMD_EP_DESC_bDescriptorType = USB_DESC_TYPE_ENDPOINT,
    // bEndpointAddress
    .CDC_IF_CMD_EP_DESC_bEndpointAddress = CDC_CMD_EP,
    // bmAttributes: Interrupt
    .CDC_IF_CMD_EP_DESC_bmAttributes = 0x03,
    // wMaxPacketSize:
    .CDC_IF_CMD_EP_DESC_wMaxPacketSize = CDC_CMD_PACKET_SIZE,
    // bInterval
    .CDC_IF_CMD_EP_DESC_bInterval = CDC_FS_BINTERVAL,

    // --------------------------------------------------------------
    // CDC Data class interface descriptor

    // bLength: (9) Endpoint Descriptor size
    .CDC_IF_DCIF_DESC_bLength = 0x09,
    // bDescriptorType:
    .CDC_IF_DCIF_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    // bInterfaceNumber: Number of Interface
    .CDC_IF_DCIF_DESC_bInterfaceNumber = USBD_CDC_DATA_INTERFACE,
    // bAlternateSetting: Alternate setting
    .CDC_IF_DCIF_DESC_bAlternateSetting = 0x00,
    // bNumEndpoints: Two endpoints used
    .CDC_IF_DCIF_DESC_bNumEndpoints = 0x02,
    // bInterfaceClass: CDC
    .CDC_IF_DCIF_DESC_bInterfaceClass = 0x0A,
    // bInterfaceSubClass:
    .CDC_IF_DCIF_DESC_bInterfaceSubClass = 0x00,
    // bInterfaceProtocol:
    .CDC_IF_DCIF_DESC_bInterfaceProtocol = 0x00,
    // iInterface:
    .CDC_IF_DCIF_DESC_iInterface = 0x00,

    // --------------------------------------------------------------
    // Endpoint OUT Descriptor

    // bLength: (7) Endpoint Descriptor size
    .CDC_IF_OUT_EP_DESC_bLength = 0x07,
    // bDescriptorType: Endpoint
    .CDC_IF_OUT_EP_DESC_bDescriptorType = USB_DESC_TYPE_ENDPOINT,
    // bEndpointAddress
    .CDC_IF_OUT_EP_DESC_bEndpointAddress = CDC_OUT_EP,
    // bmAttributes: Bulk
    .CDC_IF_OUT_EP_DESC_bmAttributes = 0x02,
    // wMaxPacketSize:
    .CDC_IF_OUT_EP_DESC_wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE,
    // bInterval: ignore for Bulk transfer
    .CDC_IF_OUT_EP_DESC_bInterval = 0x00,

    // --------------------------------------------------------------
    // Endpoint IN Descriptor

    // bLength: (7) Endpoint Descriptor size
    .CDC_IF_IN_EP_DESC_bLength = 0x07,
    // bDescriptorType: Endpoint
    .CDC_IF_IN_EP_DESC_bDescriptorType = USB_DESC_TYPE_ENDPOINT,
    // bEndpointAddress
    .CDC_IF_IN_EP_DESC_bEndpointAddress = CDC_IN_EP,
    // bmAttributes: Bulk
    .CDC_IF_IN_EP_DESC_bmAttributes = 0x02,
    // wMaxPacketSize:
    .CDC_IF_IN_EP_DESC_wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE,
    // bInterval: ignore for Bulk transfer
    .CDC_IF_IN_EP_DESC_bInterval = 0x00
    #endif

    // --------------------------------------------------------------
    // --------------------------------------------------------------
    #if defined(USBD_USE_HID_COMPOSITE)

    #if defined(USBD_USE_CDC)
    ,
    #endif

    // --------------------------------------------------------------
    // Joystick / Mouse Interface Descriptor

    // bLength: (9) Interface Descriptor size
    .HID_MOUSE_IF_DESC_bLength = 0x09,
    // bDescriptorType: Interface descriptor type
    .HID_MOUSE_IF_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    // bInterfaceNumber: Number of Interface
    .HID_MOUSE_IF_DESC_bInterfaceNumber = HID_MOUSE_INTERFACE,
    // bAlternateSetting: Alternate setting
    .HID_MOUSE_IF_DESC_bAlternateSetting = 0x00,
    // bNumEndpoints
    .HID_MOUSE_IF_DESC_bNumEndpoints = 0x01,
    // bInterfaceClass: HID
    .HID_MOUSE_IF_DESC_bInterfaceClass = 0x03,
    // bInterfaceSubClass : 1=BOOT, 0=no boot
    .HID_MOUSE_IF_DESC_bInterfaceSubClass = 0x01,
    // nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
    .HID_MOUSE_IF_DESC_bInterfaceProtocol = 0x02,
    // iInterface: Index of string descriptor
    .HID_MOUSE_IF_DESC_iInterface = 0x00,

    // --------------------------------------------------------------
    // Joystick / Mouse HID Report Descriptor

    // bLength: (9) HID Descriptor size
    .HID_MOUSE_IF_REP_DESC_bLength = 0x09,
    // bDescriptorType: HID
    .HID_MOUSE_IF_REP_DESC_bDescriptorType = 0x21, //(HID_DESCRIPTOR_TYPE);
    // bcdHID: HID Class Spec release number
    .HID_MOUSE_IF_REP_DESC_bcdHID = 0x0111,
    // bCountryCode: Hardware target country
    .HID_MOUSE_IF_REP_DESC_bCountryCode = 0x00,
    // bNumDescriptors: Number of HID class descriptors to follow
    .HID_MOUSE_IF_REP_DESC_bNumDescriptors = 0x01,
    // bDescriptorType
    .HID_MOUSE_IF_REP_DESC_bDescriptorType2 = 0x22,
    // wItemLength: Total length of Report descriptor
    .HID_MOUSE_IF_REP_DESC_wItemLength = HID_MOUSE_REPORT_DESC_SIZE,

    // --------------------------------------------------------------
    // Joystick / Mouse HID Endpoint IN Descriptor

    // bLength: (7) Endpoint Descriptor size
    .HID_MOUSE_IF_IN_EP_DESC_bLength = 0x07,
    // bDescriptorType:
    .HID_MOUSE_IF_IN_EP_DESC_bDescriptorType = USB_DESC_TYPE_ENDPOINT,
    // bEndpointAddress: Endpoint Address (IN)
    .HID_MOUSE_IF_IN_EP_DESC_bEndpointAddress = HID_MOUSE_EPIN_ADDR,
    // bmAttributes: Interrupt endpoint
    .HID_MOUSE_IF_IN_EP_DESC_bmAttributes = 0x03,
    // wMaxPacketSize: 4 Byte max
    .HID_MOUSE_IF_IN_EP_DESC_wMaxPacketSize = HID_MOUSE_EPIN_SIZE,
    // bInterval: Polling Interval
    .HID_MOUSE_IF_IN_EP_DESC_bInterval = HID_FS_BINTERVAL,

    // --------------------------------------------------------------
    // Keyboard Interface Descriptor

    // bLength: (9) Interface Descriptor size
    .HID_KB_IF_DESC_bLength = 0x09,
    // bDescriptorType: Interface descriptor type
    .HID_KB_IF_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    // bInterfaceNumber: Number of Interface
    .HID_KB_IF_DESC_bInterfaceNumber = HID_KEYBOARD_INTERFACE,
    // bAlternateSetting: Alternate setting
    .HID_KB_IF_DESC_bAlternateSetting = 0x00,
    // bNumEndpoints
    .HID_KB_IF_DESC_bNumEndpoints = 0x01,
    // bInterfaceClass: HID
    .HID_KB_IF_DESC_bInterfaceClass = 0x03,
    // bInterfaceSubClass : 1=BOOT, 0=no boot
    .HID_KB_IF_DESC_bInterfaceSubClass = 0x01,
    // nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse
    .HID_KB_IF_DESC_bInterfaceProtocol = 0x01,
    // iInterface: Index of string descriptor
    .HID_KB_IF_DESC_iInterface = 0x00,

    // --------------------------------------------------------------
    // Keyboard HID Report Descriptor

    // bLength: (9) HID Descriptor size
    .HID_KB_IF_REPT_DESC_bLength = 0x09,
    // bDescriptorType: HID
    .HID_KB_IF_REPT_DESC_bDescriptorType = 0x21, //(HID_DESCRIPTOR_TYPE);
    // bcdHID: HID Class Spec release number
    .HID_KB_IF_REPT_DESC_bcdHID = 0x0111,
    // bCountryCode: Hardware target country
    .HID_KB_IF_REPT_DESC_bCountryCode = 0x00,
    // bNumDescriptors: Number of HID class descriptors to follow
    .HID_KB_IF_REPT_DESC_bNumDescriptors = 0x01,
    // bDescriptorType
    .HID_KB_IF_REPT_DESC_bDescriptorType2 = 0x22,
    // wItemLength: Total length of Report descriptor
    .HID_KB_IF_REPT_DESC_wItemLength = HID_KEYBOARD_REPORT_DESC_SIZE,

    // --------------------------------------------------------------
    // Keyboard Endpoint IN Descriptor

    // bLength: (7) Endpoint Descriptor size
    .HID_KB_IF_IN_EP_DESC_bLength = 0x07,
    // bDescriptorType:
    .HID_KB_IF_IN_EP_DESC_bDescriptorType = USB_DESC_TYPE_ENDPOINT,
    // bEndpointAddress: Endpoint Address (IN)
    .HID_KB_IF_IN_EP_DESC_bEndpointAddress = HID_KEYBOARD_EPIN_ADDR,
    // bmAttributes: Interrupt endpoint
    .HID_KB_IF_IN_EP_DESC_bmAttributes = 0x03,
    // wMaxPacketSize: 4 Byte max
    .HID_KB_IF_IN_EP_DESC_wMaxPacketSize = HID_KEYBOARD_EPIN_SIZE,
    // bInterval: Polling Interval
    .HID_KB_IF_IN_EP_DESC_bInterval = HID_FS_BINTERVAL

    #endif

    // --------------------------------------------------------------
    // --------------------------------------------------------------
    #ifdef USBD_USE_AUDIO

    #if defined(USBD_USE_CDC)
    ,
    #endif

    // --------------------------------------------------------------
    // USB Speaker Standard interface descriptor

    // bLength (9)
    .AUDIO_IF_DESC_bLength = 0x09,
    // bDescriptorType
    .AUDIO_IF_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    // bInterfaceNumber
    .AUDIO_IF_DESC_bInterfaceNumber = USBD_AUDIO_INTERFACE_NUMBER,
    // bAlternateSetting
    .AUDIO_IF_DESC_bAlternateSetting = 0x00,
    // bNumEndpoints
    .AUDIO_IF_DESC_bNumEndpoints = 0x00,
    // bInterfaceClass
    .AUDIO_IF_DESC_bInterfaceClass = USB_DEVICE_CLASS_AUDIO,
    // bInterfaceSubClass
    .AUDIO_IF_DESC_bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOCONTROL,
    // bInterfaceProtocol
    .AUDIO_IF_DESC_bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED,
    // iInterface
    .AUDIO_IF_DESC_iInterface = 0x00,

    // --------------------------------------------------------------
    // USB Speaker Class-specific AC Interface Descriptor

    // bLength (9)
    .AUDIO_SPKR_IF_DESC_bLength = 0x09,
    // bDescriptorType
    .AUDIO_SPKR_IF_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    // bDescriptorSubtype
    .AUDIO_SPKR_IF_DESC_bDescriptorSubtype = AUDIO_CONTROL_HEADER,
    // bcdADC
    .AUDIO_SPKR_IF_DESC_bcdADC = 0x0100, // 1.00
    // wTotalLength
    .AUDIO_SPKR_IF_DESC_wTotalLength = 0x0027,
    // bInCollection
    .AUDIO_SPKR_IF_DESC_bInCollection = 0x01,
    // baInterfaceNr
    .AUDIO_SPKR_IF_DESC_baInterfaceNr = 0x01,

    // --------------------------------------------------------------
    // USB Speaker Input Terminal Descriptor

    // bLength (12)
    .AUDIO_SPKR_IN_TM_DESC_bLength = AUDIO_INPUT_TERMINAL_DESC_SIZE,
    // bDescriptorType
    .AUDIO_SPKR_IN_TM_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    // bDescriptorSubtype
    .AUDIO_SPKR_IN_TM_DESC_bDescriptorSubtype = AUDIO_CONTROL_INPUT_TERMINAL,
    // bTerminalID
    .AUDIO_SPKR_IN_TM_DESC_bTerminalID = 0x01,
    // wTerminalType: AUDIO_TERMINAL_USB_STREAMING
    .AUDIO_SPKR_IN_TM_DESC_wTerminalType = 0x0101,
    // bAssocTerminal
    .AUDIO_SPKR_IN_TM_DESC_bAssocTerminal = 0x00,
    // bNrChannels
    .AUDIO_SPKR_IN_TM_DESC_bNrChannels = 0x01,
    // wChannelConfig:  0x0000  Mono
    .AUDIO_SPKR_IN_TM_DESC_wChannelConfig = 0x0000,
    // iChannelNames
    .AUDIO_SPKR_IN_TM_DESC_iChannelNames = 0x00,
    // iTerminal
    .AUDIO_SPKR_IN_TM_DESC_iTerminal = 0x00,

    // --------------------------------------------------------------
    // USB Speaker Audio Feature Unit Descriptor

    // bLength (9)
    .AUDIO_SPKR_AFU_DESC_bLength = 0x09,
    // bDescriptorType
    .AUDIO_SPKR_AFU_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    // bDescriptorSubtype
    .AUDIO_SPKR_AFU_DESC_bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
    // bUnitID
    .AUDIO_SPKR_AFU_DESC_bUnitID = AUDIO_OUT_STREAMING_CTRL,
    // bSourceID
    .AUDIO_SPKR_AFU_DESC_bSourceID = 0x01,
    // bControlSize
    .AUDIO_SPKR_AFU_DESC_bControlSize = 0x01,
    // bmaControls(0)
    .AUDIO_SPKR_AFU_DESC_bmaControls0 = AUDIO_CONTROL_MUTE,
    // bmaControls(1)
    .AUDIO_SPKR_AFU_DESC_bmaControls1 = 0,
    // iTerminal
    .AUDIO_SPKR_AFU_DESC_iTerminal = 0x00,

    // --------------------------------------------------------------
    // USB Speaker Output Terminal Descriptor

    // bLength (9)
    .AUDIO_SPKR_OUT_TM_DESC_bLength = 0x09,
    // bDescriptorType
    .AUDIO_SPKR_OUT_TM_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    // bDescriptorSubtype
    .AUDIO_SPKR_OUT_TM_DESC_bDescriptorSubtype = AUDIO_CONTROL_OUTPUT_TERMINAL,
    // bTerminalID
    .AUDIO_SPKR_OUT_TM_DESC_bTerminalID = 0x03,
    // wTerminalType
    .AUDIO_SPKR_OUT_TM_DESC_wTerminalType = 0x0301,
    // bAssocTerminal
    .AUDIO_SPKR_OUT_TM_DESC_bAssocTerminal = 0x00,
    // bSourceID
    .AUDIO_SPKR_OUT_TM_DESC_bSourceID = 0x02,
    // iTerminal
    .AUDIO_SPKR_OUT_TM_DESC_iTerminal = 0x00,

    // --------------------------------------------------------------
    // USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith

    // bLength (9)
    .AUDIO_SPKR_AS_IF0_DESC_bLength = AUDIO_INTERFACE_DESC_SIZE,
    // bDescriptorType
    .AUDIO_SPKR_AS_IF0_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    // bInterfaceNumber
    .AUDIO_SPKR_AS_IF0_DESC_bInterfaceNumber = USBD_AUDIO_STREAM_IF_NUMBER,
    // bAlternateSetting
    .AUDIO_SPKR_AS_IF0_DESC_bAlternateSetting = 0x00,
    // bNumEndpoints
    .AUDIO_SPKR_AS_IF0_DESC_bNumEndpoints = 0x00,
    // bInterfaceClass
    .AUDIO_SPKR_AS_IF0_DESC_bInterfaceClass = USB_DEVICE_CLASS_AUDIO,
    // bInterfaceSubClass
    .AUDIO_SPKR_AS_IF0_DESC_bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOSTREAMING,
    // bInterfaceProtocol
    .AUDIO_SPKR_AS_IF0_DESC_bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED,
    // iInterface
    .AUDIO_SPKR_AS_IF0_DESC_iInterface = 0x00,

    // --------------------------------------------------------------
    // USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith

    // bLength (9)
    .AUDIO_SPKR_AS_IF1_DESC_bLength = AUDIO_INTERFACE_DESC_SIZE,
    // bDescriptorType
    .AUDIO_SPKR_AS_IF1_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    // bInterfaceNumber
    .AUDIO_SPKR_AS_IF1_DESC_bInterfaceNumber = USBD_AUDIO_STREAM_IF_NUMBER,
    // bAlternateSetting
    .AUDIO_SPKR_AS_IF1_DESC_bAlternateSetting = 0x01,
    // bNumEndpoints
    .AUDIO_SPKR_AS_IF1_DESC_bNumEndpoints = 0x01,
    // bInterfaceClass
    .AUDIO_SPKR_AS_IF1_DESC_bInterfaceClass = USB_DEVICE_CLASS_AUDIO,
    // bInterfaceSubClass
    .AUDIO_SPKR_AS_IF1_DESC_bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOSTREAMING,
    // bInterfaceProtocol
    .AUDIO_SPKR_AS_IF1_DESC_bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED,
    // iInterface
    .AUDIO_SPKR_AS_IF1_DESC_iInterface = 0x00,

    // --------------------------------------------------------------
    // USB Speaker Audio Streaming Interface Descriptor

    // bLength (7)
    .AUDIO_SPKR_AUD_ST_IF_DESC_bLength = AUDIO_STREAMING_INTERFACE_DESC_SIZE,
    // bDescriptorType
    .AUDIO_SPKR_AUD_ST_IF_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    // bDescriptorSubtype
    .AUDIO_SPKR_AUD_ST_IF_DESC_bDescriptorSubtype = AUDIO_STREAMING_GENERAL,
    // bTerminalLink
    .AUDIO_SPKR_AUD_ST_IF_DESC_bTerminalLink = 0x01,
    // bDelay
    .AUDIO_SPKR_AUD_ST_IF_DESC_bDelay = 0x01,
    // wFormatTag AUDIO_FORMAT_PCM 0x0001
    .AUDIO_SPKR_AUD_ST_IF_DESC_wFormatTag = 0x0001,

    // --------------------------------------------------------------
    // USB Speaker Audio Type III Format Interface Descriptor

    // bLength (11)
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bLength = 0x0B,
    // bDescriptorType
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    // bDescriptorSubtype
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bDescriptorSubtype = AUDIO_STREAMING_FORMAT_TYPE,
    // bFormatType
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bFormatType = AUDIO_FORMAT_TYPE_I,
    // bNrChannels: 2
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bNrChannels = 0x02,
    // bSubFrameSize : 2 Bytes per frame (16bits)
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bSubFrameSize = 0x02,
    // bBitResolution (16-bits per sample)
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bBitResolution = 16,
    // bSamFreqType only one frequency supported
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bSamFreqType = 0x01,
    // tSamFreq[0:1] Audio sampling frequency coded on 3 bytes 
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_tSamFreqLowestHalfWord = (uint16_t)USBD_AUDIO_FREQ,
    // tSamFreq[2]
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_tSamFreqHighestByte = (uint8_t)((uint32_t)USBD_AUDIO_FREQ >> 16), // Highest Byte

    // --------------------------------------------------------------
    // Audio Endpoint 1 - Standard Descriptor

    // bLength (9)
    .AUD_SPKR_OUT_EP_DESC_bLength = AUDIO_STANDARD_ENDPOINT_DESC_SIZE,
    // bDescriptorType
    .AUD_SPKR_OUT_EP_DESC_bDescriptorType = USB_DESC_TYPE_ENDPOINT,
    // bEndpointAddress 1 out endpoint
    .AUD_SPKR_OUT_EP_DESC_bEndpointAddress = AUDIO_OUT_EP,
    // bmAttributes
    .AUD_SPKR_OUT_EP_DESC_bmAttributes = USBD_EP_TYPE_ISOC,
    // wMaxPacketSize in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord))
    .AUD_SPKR_OUT_EP_DESC_wMaxPacketSize = AUDIO_PACKET_SIZE(USBD_AUDIO_FREQ),
    // bInterval
    .AUD_SPKR_OUT_EP_DESC_bInterval = 0x01,
    // bRefresh
    .AUD_SPKR_OUT_EP_DESC_bRefresh = 0x00,
    // bSynchAddress
    .AUD_SPKR_OUT_EP_DESC_bSynchAddress = 0x00,

    // --------------------------------------------------------------
    // Endpoint - Audio Streaming Descriptor

    // bLength (7)
    .AUD_STRM_GEN_EP_DESC_bLength = AUDIO_STREAMING_ENDPOINT_DESC_SIZE,
    // bDescriptorType
    .AUD_STRM_GEN_EP_DESC_bDescriptorType = AUDIO_ENDPOINT_DESCRIPTOR_TYPE,
    // bDescriptor
    .AUD_STRM_GEN_EP_DESC_bDescriptor = AUDIO_ENDPOINT_GENERAL,
    // bmAttributes
    .AUD_STRM_GEN_EP_DESC_bmAttributes = 0x00,
    // bLockDelayUnits
    .AUD_STRM_GEN_EP_DESC_bLockDelayUnits = 0x00,
    // wLockDelay
    .AUD_STRM_GEN_EP_DESC_wLockDelay = 0x0000

    #endif
};


/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPOSITE_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};


// --------------------------------------------------------------------------
/**
  * @brief  calculate_descriptor_length
  *         Return descriptor wTotalLength
  * @retval wTotalLength
  */
static uint16_t calculate_descriptor_length() {
    return sizeof(COMPOSITE_DESCRIPTOR_t);
}

// --------------------------------------------------------------------------
/**
  * @brief  USBD_COMPOSITE_GetFSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  *
static uint8_t  *USBD_COMPOSITE_GetFSCfgDesc(uint16_t *length)
{
    *length = calculate_descriptor_length();

    // If CDC port is enabled then we must over-write the
    // values of bInterval and wMaxPacketSize with the
    // appropriate speed values.
    #if defined(USBD_USE_CDC)
        COMPOSITE_DESCRIPTOR.CDC_IF_CMD_EP_DESC_bInterval   = CDC_FS_BINTERVAL;
        COMPOSITE_DESCRIPTOR.CDC_IF_OUT_EP_DESC_wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
        COMPOSITE_DESCRIPTOR.CDC_IF_IN_EP_DESC_wMaxPacketSize = CDC_DATA_FS_MAX_PACKET_SIZE;
    #endif

    #if defined(USBD_USE_HID_COMPOSITE)
        COMPOSITE_DESCRIPTOR.HID_MOUSE_IF_IN_EP_DESC_bInterval = HID_FS_BINTERVAL;
    #endif

    return &COMPOSITE_DESCRIPTOR;
}

**
  * @brief  USBD_COMPOSITE_GetHSCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  *
static uint8_t  *USBD_COMPOSITE_GetHSCfgDesc(uint16_t *length)
{
    *length = calculate_descriptor_length();

    // If CDC port is enabled then we must over-write the
    // values of bInterval and wMaxPacketSize with the
    // appropriate speed values.
    #if defined(USBD_USE_CDC)
        COMPOSITE_DESCRIPTOR.CDC_IF_CMD_EP_DESC_bInterval   = CDC_HS_BINTERVAL;
        COMPOSITE_DESCRIPTOR.CDC_IF_OUT_EP_DESC_wMaxPacketSize = CDC_DATA_HS_MAX_PACKET_SIZE;
        COMPOSITE_DESCRIPTOR.CDC_IF_IN_EP_DESC_wMaxPacketSize = CDC_DATA_HS_MAX_PACKET_SIZE;
    #endif

    #if defined(USBD_USE_HID_COMPOSITE)
        COMPOSITE_DESCRIPTOR.HID_MOUSE_IF_IN_EP_DESC_bInterval = HID_HS_BINTERVAL;
    #endif

    return &COMPOSITE_DESCRIPTOR;
}

**
  * @brief  USBD_CDC_GetCfgDesc
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  *
static uint8_t  *USBD_COMPOSITE_GetOtherSpeedCfgDesc(uint16_t *length)
{
    *length = calculate_descriptor_length();

    COMPOSITE_DESCRIPTOR.cfgDesc_bDescriptorType = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;

    // If CDC port is enabled then we must over-write the
    // values of bInterval and wMaxPacketSize with the
    // appropriate speed values.
    #if defined(USBD_USE_CDC)
        COMPOSITE_DESCRIPTOR.CDC_IF_CMD_EP_DESC_bInterval   = CDC_FS_BINTERVAL;
        COMPOSITE_DESCRIPTOR.CDC_IF_OUT_EP_DESC_wMaxPacketSize = 64;
        COMPOSITE_DESCRIPTOR.CDC_IF_IN_EP_DESC_wMaxPacketSize = 64;
    #endif

    #if defined(USBD_USE_HID_COMPOSITE)
        COMPOSITE_DESCRIPTOR.HID_MOUSE_IF_IN_EP_DESC_bInterval = HID_FS_BINTERVAL;
    #endif

    return &COMPOSITE_DESCRIPTOR;
}

**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*
uint8_t  *USBD_COMPOSITE_GetDeviceQualifierDescriptor(uint16_t *length)
{
  *length = sizeof(USBD_COMPOSITE_DeviceQualifierDesc);
  return USBD_COMPOSITE_DeviceQualifierDesc;
}
*/