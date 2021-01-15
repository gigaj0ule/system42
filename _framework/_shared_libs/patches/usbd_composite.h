/**
 * @file        usbd_composite.h
 * @author      Weyne
 * @version     V01
 * @date        2016.10.28
 * @brief       MSC + CDC 复合设备
 * @note
 * @attention   COYPRIGHT WEYNE
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_COMPOSITE_H
#define __USBD_COMPOSITE_H


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */

/** @defgroup usbd_composite
  * @brief This file is the Header file for usbd_composite.c
  * @{
  */

/** @defgroup usbd_cdc_Exported_Defines
  * @{
  */
 /**
  * @}
  */

/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */
typedef struct _USBD_COMPOSITE_Itf {
  int8_t (* Init)(void);
  int8_t (* DeInit)(void);
  int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
  int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);
  int8_t (* Transferred)(void);
} USBD_Composite_ItfTypeDef;
/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */

extern USBD_ClassTypeDef USBD_COMPOSITE;
#define USBD_COMPOSITE_CLASS    &USBD_COMPOSITE

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_Composite_RegisterInterface(USBD_HandleTypeDef   *pdev,
                                    USBD_Composite_ItfTypeDef *fops);

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif  /* __USBD_COMPOSITE_H */
/**
  * @}
  */

/************************ (C) COPYRIGHT WEYNE *****END OF FILE****/