/*
 *******************************************************************************
 * Copyright (c) 2020-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#if defined(ARDUINO_GENERIC_L072V8IX) || defined(ARDUINO_GENERIC_L072V8TX) ||\
    defined(ARDUINO_GENERIC_L072VBIX) || defined(ARDUINO_GENERIC_L072VBTX) ||\
    defined(ARDUINO_GENERIC_L072VZIX) || defined(ARDUINO_GENERIC_L072VZTX) ||\
    defined(ARDUINO_GENERIC_L073V8IX) || defined(ARDUINO_GENERIC_L073V8TX) ||\
    defined(ARDUINO_GENERIC_L073VBIX) || defined(ARDUINO_GENERIC_L073VBTX) ||\
    defined(ARDUINO_GENERIC_L073VZIX) || defined(ARDUINO_GENERIC_L073VZTX) ||\
    defined(ARDUINO_GENERIC_L083V8IX) || defined(ARDUINO_GENERIC_L083V8TX) ||\
    defined(ARDUINO_GENERIC_L083VBIX) || defined(ARDUINO_GENERIC_L083VBTX) ||\
    defined(ARDUINO_GENERIC_L083VZIX) || defined(ARDUINO_GENERIC_L083VZTX)
#include "pins_arduino.h"

/**
  * @brief  System Clock Configuration
  * @param  None
  * @retval None
  */
WEAK void SystemClock_Config(void)
{
  /* SystemClock_Config can be generated by STM32CubeMX */
#warning "SystemClock_Config() is empty. Default clock at reset is used."
}

#endif /* ARDUINO_GENERIC_* */
