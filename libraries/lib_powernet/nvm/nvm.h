/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NVML_H
#define __NVM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdlib.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

int NVM_init(void);
int NVM_erase(void);
size_t NVM_get_max_read_length(void);
size_t NVM_get_max_write_length(void);
int NVM_read(size_t offset, uint8_t *data, size_t length);
int NVM_format(size_t length);
int NVM_write(size_t offset, uint8_t *data, size_t length);
int NVM_commit(void);

#ifdef INCLUDE_NVM_DEMO 
void NVM_demo(void);
#endif

#ifdef __cplusplus
}
#endif

#endif //__NVM_H