/*
 *  Minimize changes required for main.c, just call BSP_main() with argument pointers to required peripherals
 *
 *
*/

#ifndef _BSP_MAIN_H
#define _BSP_MAIN_H

#include "main.h"

typedef struct BSP_ARGS
{
	CRC_HandleTypeDef *hcrc2;
	IPCC_HandleTypeDef *hipcc;
	TIM_HandleTypeDef *htim5;
} BSP_ARGS;

void BSP_main(BSP_ARGS *argv);

#endif //_BSP_MAIN_H
