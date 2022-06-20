    .AUDIO_IF_DESC_bLength = 0x09,
    .AUDIO_IF_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    .AUDIO_IF_DESC_bInterfaceNumber = USBD_AUDIO_INTERFACE_NUMBER,
    .AUDIO_IF_DESC_bAlternateSetting = 0x00,
    .AUDIO_IF_DESC_bNumEndpoints = 0x00,
    .AUDIO_IF_DESC_bInterfaceClass = USB_DEVICE_CLASS_AUDIO,
    .AUDIO_IF_DESC_bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOCONTROL,
    .AUDIO_IF_DESC_bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED,
    .AUDIO_IF_DESC_iInterface = 0x00,
  /* 09 byte*/

    .AUDIO_SPKR_IF_DESC_bLength = 0x09,
    .AUDIO_SPKR_IF_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    .AUDIO_SPKR_IF_DESC_bDescriptorSubtype = AUDIO_CONTROL_HEADER,
    .AUDIO_SPKR_IF_DESC_bcdADC = 0x0100, // 1.00

    .AUDIO_SPKR_IF_DESC_wTotalLength = 0x0027,

    .AUDIO_SPKR_IF_DESC_bInCollection = 0x01,
    .AUDIO_SPKR_IF_DESC_baInterfaceNr = 0x01,
  /* 09 byte*/

  /* USB Speaker Input Terminal Descriptor */
    .AUDIO_SPKR_IN_TM_DESC_bLength = AUDIO_INPUT_TERMINAL_DESC_SIZE,
    .AUDIO_SPKR_IN_TM_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    .AUDIO_SPKR_IN_TM_DESC_bDescriptorSubtype = AUDIO_CONTROL_INPUT_TERMINAL,
    .AUDIO_SPKR_IN_TM_DESC_bTerminalID = 0x01,
    .AUDIO_SPKR_IN_TM_DESC_wTerminalType = 0x0101,

    .AUDIO_SPKR_IN_TM_DESC_bAssocTerminal = 0x00,
    .AUDIO_SPKR_IN_TM_DESC_bNrChannels = 0x01,
    .AUDIO_SPKR_IN_TM_DESC_wChannelConfig = 0x0000,

    .AUDIO_SPKR_IN_TM_DESC_iChannelNames = 0x00,
    .AUDIO_SPKR_IN_TM_DESC_iTerminal = 0x00,
      /* 12 byte*/

  /* USB Speaker Audio Feature Unit Descriptor */
    .AUDIO_SPKR_AFU_DESC_bLength = 0x09,
    .AUDIO_SPKR_AFU_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    .AUDIO_SPKR_AFU_DESC_bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
    .AUDIO_SPKR_AFU_DESC_bUnitID = AUDIO_OUT_STREAMING_CTRL,
    .AUDIO_SPKR_AFU_DESC_bSourceID = 0x01,
    .AUDIO_SPKR_AFU_DESC_bControlSize = 0x01,
    .AUDIO_SPKR_AFU_DESC_bmaControls0 = AUDIO_CONTROL_MUTE,
    .AUDIO_SPKR_AFU_DESC_bmaControls1 = 0,
    .AUDIO_SPKR_AFU_DESC_iTerminal = 0x00,
      /* 09 byte*/

  /*USB Speaker Output Terminal Descriptor */
    .AUDIO_SPKR_OUT_TM_DESC_bLength = 0x09,
    .AUDIO_SPKR_OUT_TM_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    .AUDIO_SPKR_OUT_TM_DESC_bDescriptorSubtype = AUDIO_CONTROL_OUTPUT_TERMINAL,
    .AUDIO_SPKR_OUT_TM_DESC_bTerminalID = 0x03,
    .AUDIO_SPKR_OUT_TM_DESC_wTerminalType = 0x0301,

    .AUDIO_SPKR_OUT_TM_DESC_bAssocTerminal = 0x00,
    .AUDIO_SPKR_OUT_TM_DESC_bSourceID = 0x02,
    .AUDIO_SPKR_OUT_TM_DESC_iTerminal = 0x00,
      /* 09 byte*/
  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwith */
  /* Interface 1, Alternate Setting 0                                             */
    .AUDIO_SPKR_AS_IF0_DESC_bLength = AUDIO_INTERFACE_DESC_SIZE,
    .AUDIO_SPKR_AS_IF0_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    .AUDIO_SPKR_AS_IF0_DESC_bInterfaceNumber = USBD_AUDIO_STREAM_IF_NUMBER,
    .AUDIO_SPKR_AS_IF0_DESC_bAlternateSetting = 0x00,
    .AUDIO_SPKR_AS_IF0_DESC_bNumEndpoints = 0x00,
    .AUDIO_SPKR_AS_IF0_DESC_bInterfaceClass = USB_DEVICE_CLASS_AUDIO,
    .AUDIO_SPKR_AS_IF0_DESC_bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOSTREAMING,
    .AUDIO_SPKR_AS_IF0_DESC_bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED,
    .AUDIO_SPKR_AS_IF0_DESC_iInterface = 0x00,
/* 09 byte*/

  /* USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational */
  /* Interface 1, Alternate Setting 1                                           */
    .AUDIO_SPKR_AS_IF1_DESC_bLength = AUDIO_INTERFACE_DESC_SIZE,
    .AUDIO_SPKR_AS_IF1_DESC_bDescriptorType = USB_DESC_TYPE_INTERFACE,
    .AUDIO_SPKR_AS_IF1_DESC_bInterfaceNumber = USBD_AUDIO_STREAM_IF_NUMBER,
    .AUDIO_SPKR_AS_IF1_DESC_bAlternateSetting = 0x01,
    .AUDIO_SPKR_AS_IF1_DESC_bNumEndpoints = 0x01,
    .AUDIO_SPKR_AS_IF1_DESC_bInterfaceClass = USB_DEVICE_CLASS_AUDIO,
    .AUDIO_SPKR_AS_IF1_DESC_bInterfaceSubClass = AUDIO_SUBCLASS_AUDIOSTREAMING,
    .AUDIO_SPKR_AS_IF1_DESC_bInterfaceProtocol = AUDIO_PROTOCOL_UNDEFINED,
    .AUDIO_SPKR_AS_IF1_DESC_iInterface = 0x00,
  /* 09 byte*/

  /* USB Speaker Audio Streaming Interface Descriptor */
    .AUDIO_SPKR_AUD_ST_IF_DESC_bLength = AUDIO_STREAMING_INTERFACE_DESC_SIZE,
    .AUDIO_SPKR_AUD_ST_IF_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    .AUDIO_SPKR_AUD_ST_IF_DESC_bDescriptorSubtype = AUDIO_STREAMING_GENERAL,
    .AUDIO_SPKR_AUD_ST_IF_DESC_bTerminalLink = 0x01,
    .AUDIO_SPKR_AUD_ST_IF_DESC_bDelay = 0x01,
    .AUDIO_SPKR_AUD_ST_IF_DESC_wFormatTag = 0x0001,
    
  /* 07 byte*/

  /* USB Speaker Audio Type III Format Interface Descriptor */
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bLength = 0x0B,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bDescriptorType = AUDIO_INTERFACE_DESCRIPTOR_TYPE,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bDescriptorSubtype = AUDIO_STREAMING_FORMAT_TYPE,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bFormatType = AUDIO_FORMAT_TYPE_I,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bNrChannels = 0x02,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bSubFrameSize = 0x02,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bBitResolution = 16,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bSamFreqType = 0x01,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bFreq0 = 0x80,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bFreq1 = 0xBB,
    .AUDIO_SPKR_AUD_T3FMT_IF_DESC_bFreq2 = 0x00,
  /* 11 byte*/

  /* Endpoint 1 - Standard Descriptor */
    .AUD_SPKR_OUT_EP_DESC_bLength = AUDIO_STANDARD_ENDPOINT_DESC_SIZE,
    .AUD_SPKR_OUT_EP_DESC_bDescriptorType = USB_DESC_TYPE_ENDPOINT,
    .AUD_SPKR_OUT_EP_DESC_bEndpointAddress = AUDIO_OUT_EP,
    .AUD_SPKR_OUT_EP_DESC_bmAttributes = USBD_EP_TYPE_ISOC,
    .AUD_SPKR_OUT_EP_DESC_wMaxPacketSize = AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),
    .AUD_SPKR_OUT_EP_DESC_bInterval = 0x01,
    .AUD_SPKR_OUT_EP_DESC_bRefresh = 0x00,
    .AUD_SPKR_OUT_EP_DESC_bSynchAddress = 0x00,
  /* 09 byte*/

  /* Endpoint - Audio Streaming Descriptor*/
    .AUD_STRM_GEN_EP_DESC_bLength = AUDIO_STREAMING_ENDPOINT_DESC_SIZE,
    .AUD_STRM_GEN_EP_DESC_bDescriptorType = AUDIO_ENDPOINT_DESCRIPTOR_TYPE,
    .AUD_STRM_GEN_EP_DESC_bDescriptor = AUDIO_ENDPOINT_GENERAL,
    .AUD_STRM_GEN_EP_DESC_bmAttributes = 0x00,
    .AUD_STRM_GEN_EP_DESC_bLockDelayUnits = 0x00,
    .AUD_STRM_GEN_EP_DESC_wLockDelay = 0x0000

  /* 07 byte*/ 