#ifndef JUMP_H
#define JUMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver_init.h"
/**
 * Initializes MCU, drivers and middleware in the project
 **/
uint8_t JumpFun(uint8_t *tempData);
void FLASH_init(void);
void FLASH_CLOCK_init(void);
void FLASH_flg(void);
#ifdef __cplusplus
}
#endif
#endif




