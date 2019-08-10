/**
 ******************************************************************************
 * @file    MotionMC_Manager.h
 * @author  MEMS Software Solutions Team
 * @brief   This file contains definitions for the MotionMC_Manager.c file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MOTIONMC_MANAGER_H
#define MOTIONMC_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
#include "motion_mc.h"

#elif (defined (USE_STM32L0XX_NUCLEO))
#include "motion_mc_cm0p.h"

#else
#error Not supported platform
#endif

#include "iks01a2_motion_sensors.h"
#include "serial_protocol.h"

/* Extern variables ----------------------------------------------------------*/
extern volatile uint32_t TimeStamp;

/* Exported Functions Prototypes ---------------------------------------------*/
#if ((defined (USE_STM32F4XX_NUCLEO)) || (defined (USE_STM32L1XX_NUCLEO)) || (defined (USE_STM32L4XX_NUCLEO)))
void MotionMC_manager_init(int sampletime, unsigned short int enable);
void MotionMC_manager_update(MMC_Input_t *data_in);
void MotionMC_manager_get_params(MMC_Output_t *data_out);

#elif (defined (USE_STM32L0XX_NUCLEO))
void MotionMC_manager_init(int sampletime, MMC_CM0P_Mode_t mode, unsigned short int enable);
void MotionMC_manager_update(MMC_CM0P_Input_t *data_in);
void MotionMC_manager_get_params(MMC_CM0P_Output_t *data_out);

#else
#error Not supported platform
#endif

void MotionMC_manager_run(TMsg *Msg);
void MotionMC_manager_get_version(char *version, int *length);
void MotionMC_manager_compensate(IKS01A2_MOTION_SENSOR_Axes_t *data_raw, IKS01A2_MOTION_SENSOR_Axes_t *data_comp);

#ifdef __cplusplus
}
#endif

#endif /* MOTIONMC_MANAGER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
