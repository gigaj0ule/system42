/* HAL raised several warnings, ignore them */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

#ifdef STM32L5xx
  #include "stm32l5xx_hal_icache.c"
#elif STM32U5xx
  #include "stm32u5xx_hal_icache.c"
#endif
#pragma GCC diagnostic pop
