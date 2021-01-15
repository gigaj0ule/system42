/**
  ******************************************************************************
  * @file    usbd_ep_conf.c
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
#if defined(HAL_PCD_MODULE_ENABLED) && defined(USBCON)

/* Includes ------------------------------------------------------------------*/
#include "usbd_ep_conf.h"

const ep_desc_t ep_def[] = {

  // ---------------------------------------------------
  // Define EP0 no matter what as it is always needed
  #ifdef USE_USB_HS
    {0x00,       CDC_DATA_HS_MAX_PACKET_SIZE},
    {0x80,       CDC_DATA_HS_MAX_PACKET_SIZE},
  #else /* USE_USB_HS */
    #ifdef USB_OTG_FS
      {0x00,       CDC_DATA_FS_MAX_PACKET_SIZE},
      {0x80,       CDC_DATA_FS_MAX_PACKET_SIZE},
    #else /* USB_OTG_FS */
      {0x00,       PMA_EP0_OUT_ADDR, PCD_SNG_BUF},
      {0x80,       PMA_EP0_IN_ADDR,  PCD_SNG_BUF},
    #endif /* USB_OTG_FS */
  #endif /* USE_USB_HS */

  // ---------------------------------------------------
  // Define endpoints for CDC
  #ifdef USBD_USE_CDC

    #ifdef USE_USB_HS
      // EP0 Already Defined
      {CDC_OUT_EP, CDC_DATA_HS_MAX_PACKET_SIZE},
      {CDC_IN_EP,  CDC_DATA_HS_MAX_PACKET_SIZE},
      {CDC_CMD_EP, CDC_CMD_PACKET_SIZE}
    #else /* USE_USB_FS */

      #ifdef USB_OTG_FS
        // EP0 already defined
        {CDC_OUT_EP, CDC_DATA_FS_MAX_PACKET_SIZE},
        {CDC_IN_EP,  CDC_DATA_FS_MAX_PACKET_SIZE},
        {CDC_CMD_EP, CDC_CMD_PACKET_SIZE}
      #else
        // EP0 already defined
        {CDC_OUT_EP, PMA_CDC_OUT_ADDR, PCD_SNG_BUF /*PCD_DBL_BUF*/},
        {CDC_IN_EP,  PMA_CDC_IN_ADDR,  PCD_SNG_BUF},
        {CDC_CMD_EP, PMA_CDC_CMD_ADDR, PCD_SNG_BUF},
      #endif /* USB_OTG_FS */
    #endif /* USE_USB_HS */
  #endif /* USBD_USE_CDC */

  // ---------------------------------------------------
  // Define endpoints for HID
  #ifdef USBD_USE_HID_COMPOSITE
    #if !defined (USB)
      {HID_MOUSE_EPIN_ADDR,    HID_MOUSE_EPIN_SIZE},
      {HID_KEYBOARD_EPIN_ADDR, HID_KEYBOARD_EPIN_SIZE},
    #else
      {HID_MOUSE_EPIN_ADDR,    PMA_MOUSE_IN_ADDR,    PCD_SNG_BUF},
      {HID_KEYBOARD_EPIN_ADDR, PMA_KEYBOARD_IN_ADDR, PCD_SNG_BUF},
    #endif
  #endif /* USBD_USE_HID_COMPOSITE */

  // ---------------------------------------------------
  // Define endpoints for HID
  #ifdef USBD_USE_AUDIO
    #if !defined (USB)
      {AUDIO_OUT_EP,    AUDIO_OUT_EP_SIZE},
    #else
      {AUDIO_OUT_EP,    PMA_AUDIO_IN_ADDR,    PCD_SNG_BUF},
    #endif
  #endif /* USBD_USE_HID_COMPOSITE */



};

#endif /* HAL_PCD_MODULE_ENABLED && USBCON */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

