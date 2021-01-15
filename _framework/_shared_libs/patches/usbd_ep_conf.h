/**
  ******************************************************************************
  * @file    usbd_ep_conf.h
  * @brief   USB Device endpoints configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_EP_CONF_H
#define __USBD_EP_CONF_H

#ifdef USBCON

  #include <stdint.h>
  #include "usbd_def.h"

  typedef struct {
    uint32_t ep_adress; /* Endpoint address */
    uint32_t ep_size;   /* Endpoint size */
  #if defined (USB)
    uint32_t ep_kind;   /* PCD Endpoint Kind: PCD_SNG_BUF or PCD_DBL_BUF */
  #endif
  } ep_desc_t;

  // ------------------------------------------------------------
  // ------------------------------------------------------------

  #ifdef USBD_USE_CDC 

    // Interface address
    #define USBD_CDC_CMD_INTERFACE        0
    #define USBD_CDC_DATA_INTERFACE       1

    // Endpoints address
    #define CDC_OUT_EP                    (0x00 | 1)
    #define CDC_IN_EP                     (0x80 | 2)
    #define CDC_CMD_EP                    (0x80 | 3)

    // Endpoint sizes
    #define CDC_DATA_HS_MAX_PACKET_SIZE   USB_HS_MAX_PACKET_SIZE
    #define CDC_DATA_FS_MAX_PACKET_SIZE   USB_FS_MAX_PACKET_SIZE
    #define CDC_CMD_PACKET_SIZE           8U

    // For calculating offsets of further devices
    #define NUM_CDC_ENDPOINTS   (3)
    #define NUM_CDC_IFACES      (2)
  
  #else /* USBD_USE_CDC */

    // For calculating offsets of further devices
    #define NUM_CDC_ENDPOINTS   (0)
    #define NUM_CDC_IFACES      (0)

  #endif /* USBD_USE_CDC */

  // ------------------------------------------------------------
  // ------------------------------------------------------------

  #ifdef USBD_USE_HID_COMPOSITE

    // Interface address
    #define HID_MOUSE_INTERFACE           (NUM_CDC_IFACES + 0)
    #define HID_KEYBOARD_INTERFACE        (NUM_CDC_IFACES + 1)

    // Endpoint address
    #define HID_MOUSE_EPIN_ADDR           (0x80 | (NUM_CDC_ENDPOINTS + 1))
    #define HID_KEYBOARD_EPIN_ADDR        (0x80 | (NUM_CDC_ENDPOINTS + 2))

    // Endpoint sizes
    #define HID_MOUSE_EPIN_SIZE           0x04U
    #define HID_KEYBOARD_EPIN_SIZE        0x08U

    // For calculating offsets of further devices
    #define NUM_HID_COMPOSITE_ENDPOINTS   (2)
    #define NUM_HID_COMPOSITE_IFACES      (2)

  #else /* USBD_USE_HID_COMPOSITE */

    // For calculating offsets of further devices
    #define NUM_HID_COMPOSITE_ENDPOINTS   (0)
    #define NUM_HID_COMPOSITE_IFACES      (0)

  #endif /* USBD_USE_HID_COMPOSITE */

  // ------------------------------------------------------------
  // ------------------------------------------------------------

  #ifdef USBD_USE_AUDIO

    #include "usbd_audio.h"

    // Interface address
    #define USBD_AUDIO_INTERFACE_NUMBER   (NUM_CDC_IFACES + NUM_HID_COMPOSITE_IFACES + 0)
    #define USBD_AUDIO_STREAM_IF_NUMBER   (NUM_CDC_IFACES + NUM_HID_COMPOSITE_IFACES + 1)

    // Endpoint address
    #define AUDIO_OUT_EP                  (0x00 | (NUM_CDC_ENDPOINTS + NUM_HID_COMPOSITE_ENDPOINTS + 1))

    // Endpoint sizes
    #define AUDIO_OUT_EP_SIZE             AUDIO_PACKET_SIZE(USBD_AUDIO_FREQ)

    // For calculating offsets of further devices
    #define NUM_AUDIO_ENDPOINTS           (1)
    #define NUM_AUDIO_IFACES              (2)
  
  #else /* USBD_USE_AUDIO */

    // For calculating offsets of further devices
    #define NUM_AUDIO_ENDPOINTS           (0)
    #define NUM_AUDIO_IFACES              (0)
  
  #endif /* USBD_USE_AUDIO */

  // ------------------------------------------------------------
  // ------------------------------------------------------------
  // ------------------------------------------------------------
  // ------------------------------------------------------------

  // Device Endpoints number including EP0
  #define DEV_NUM_EP          (1 + NUM_CDC_ENDPOINTS + NUM_HID_COMPOSITE_ENDPOINTS + NUM_AUDIO_ENDPOINTS)
  #define DEV_NUM_IFACES      (NUM_CDC_IFACES + NUM_HID_COMPOSITE_IFACES + NUM_AUDIO_IFACES)

  // Require DEV_NUM_EP to be defined
  #if defined (USB)

    // ---------------------------------------

    // Size in words, byte size divided by 2
    #define PMA_EP0_OUT_ADDR    (8 * DEV_NUM_EP)
    #define PMA_EP0_IN_ADDR     (PMA_EP0_OUT_ADDR + USB_MAX_EP0_SIZE)

    // ---------------------------------------

    // CDC interace enabled?
    #ifdef USBD_USE_CDC

      // Yes! First endpoint comes directly after EP0
      #define PMA_CDC_OUT_BASE    (PMA_EP0_IN_ADDR + USB_MAX_EP0_SIZE)

      #define PMA_CDC_OUT_ADDR    ((PMA_CDC_OUT_BASE + USB_FS_MAX_PACKET_SIZE) | \
                                  (PMA_CDC_OUT_BASE << 16U))
   
      #define PMA_CDC_IN_ADDR     (PMA_CDC_OUT_BASE + USB_FS_MAX_PACKET_SIZE * 2)
   
      #define PMA_CDC_CMD_ADDR    (PMA_CDC_IN_ADDR + CDC_CMD_PACKET_SIZE)

      // Export running total for next device
      #define CDC_ADDR_OFFSET     PMA_CDC_CMD_ADDR

    #else /* USBD_USE_CDC */

      // If device not used then don't add any to the running total
      #define CDC_ADDR_OFFSET     PMA_EP0_IN_ADDR + 0

    #endif /* USBD_USE_CDC */

    // ---------------------------------------

    // KB & Mouse enabled?
    #ifdef USBD_USE_HID_COMPOSITE

      // KB + Mouse comes after CDC EPs
      #define PMA_MOUSE_IN_ADDR       (CDC_ADDR_OFFSET + HID_MOUSE_EPIN_SIZE)
      #define PMA_KEYBOARD_IN_ADDR    (PMA_MOUSE_IN_ADDR + HID_KEYBOARD_EPIN_SIZE)
      
      // Export running total for next device
      #define HID_COMP_ADDR_OFFSET    PMA_KEYBOARD_IN_ADDR

    #else /* USBD_USE_HID_COMPOSITE */

      // If device not used then don't add any to the running total
      #define HID_COMP_ADDR_OFFSET    CDC_ADDR_OFFSET + 0

    #endif /* USBD_USE_HID_COMPOSITE */

    // ---------------------------------------

    #ifdef USBD_USE_AUDIO

      // KB + Mouse comes after CDC EPs
      #define PMA_AUDIO_IN_ADDR       (HID_COMP_ADDR_OFFSET + AUDIO_OUT_EP_SIZE)
      
      // Export running total for next device
      #define AUD_ADDR_OFFSET         PMA_AUDIO_IN_ADDR

    #else /* USBD_USE_AUDIO */

      #define AUD_ADDR_OFFSET         CDC_ADDR_OFFSET + HID_COMP_ADDR_OFFSET + 0

    #endif /* USBD_USE_AUDIO */

  #endif /* USB */


  // ep_desc_t ep_def[] holds the endpoint descriptors (to be populated by...?)
  extern const ep_desc_t ep_def[DEV_NUM_EP + 1];

#endif /* USBCON */
#endif /* __USBD_EP_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/