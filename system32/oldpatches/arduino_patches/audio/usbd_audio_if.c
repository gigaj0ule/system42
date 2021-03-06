/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.c
  * @author  MCD Application Team
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}_audio.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_if.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */

static int8_t  USBD_AUDIO_Init(uint32_t  AudioFreq, uint32_t Volume, uint32_t options);
static int8_t  USBD_AUDIO_DeInit(uint32_t options);
static int8_t  USBD_AUDIO_AudioCmd(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t  USBD_AUDIO_VolumeCtl(uint8_t vol);
static int8_t  USBD_AUDIO_MuteCtl(uint8_t cmd);
static int8_t  USBD_AUDIO_PeriodicTC(uint8_t cmd);
static int8_t  USBD_AUDIO_GetState(void);

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Public_Variables
  * @{
  */
USBD_HandleTypeDef hUSBD_Device_AUDIO;

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_AudioCmd,
  USBD_AUDIO_VolumeCtl,
  USBD_AUDIO_MuteCtl,
  USBD_AUDIO_PeriodicTC,
  USBD_AUDIO_GetState,
};
/**
  * @}
  */


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  USBD_AUDIO_Init
  *         Initializes the AUDIO media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_AUDIO_Init(uint32_t  AudioFreq, uint32_t Volume, uint32_t options)
{
  /*
     Add your initialization code here
  */
  return (0);
}

/**
  * @brief  USBD_AUDIO_DeInit
  *         DeInitializes the AUDIO media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_AUDIO_DeInit(uint32_t options)
{
  /*
     Add your deinitialization code here
  */
  return (0);
}


/**
  * @brief  USBD_AUDIO_AudioCmd
  *         AUDIO command handler
  * @param  Buf: Buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: command opcode
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_AUDIO_AudioCmd(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  return (0);
}

/**
  * @brief  USBD_AUDIO_VolumeCtl
  * @param  vol: volume level (0..100)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_AUDIO_VolumeCtl(uint8_t vol)
{
  return (0);
}

/**
  * @brief  USBD_AUDIO_MuteCtl
  * @param  cmd: vmute command
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_AUDIO_MuteCtl(uint8_t cmd)
{
  return (0);
}

/**
  * @brief  USBD_AUDIO_PeriodicTC
  * @param  cmd
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_AUDIO_PeriodicTC(uint8_t cmd)
{
  return (0);
}

/**
  * @brief  USBD_AUDIO_GetState
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_AUDIO_GetState(void)
{
  return (0);
}
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

