#ifndef _FLASH_STORAGE_H_
#define _FLASH_STORAGE_H_

#include "main.h"

void Flash_EraseMotorParam(void);
void Flash_SaveMotorParam(int poles, float zero_elec_angle, int dir);
int Flash_ReadMotorParam(int* poles, float* zero_elec_angle, int* dir);
void Flash_SaveMotorID(uint8_t id);
uint8_t Flash_ReadMotorID(void);

#endif
