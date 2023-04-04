/*
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/**
 * @file cy_audio_sw_codecs_log.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef __CY_AUDIO_SW_CODECS_LOG_H__
#define __CY_AUDIO_SW_CODECS_LOG_H__

#if (ENABLE_ASC_LOGS > 1)
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if ENABLE_ASC_LOGS == 3
#define cy_asc_log_info(format, ...) printf("[ASC] %s %d> " format " \n", __FUNCTION__, __LINE__, ##__VA_ARGS__);
#define cy_asc_log_err(ret_val, format, ...) printf("[ASC] %s %d> #Err:0x%lx " format " \n", __FUNCTION__, __LINE__, ret_val, ##__VA_ARGS__);
#ifdef ENABLE_ASC_DBG_LOGS
#define cy_asc_log_dbg(format, ...) printf("[ASC] %s %d> " format " \n", __FUNCTION__, __LINE__, ##__VA_ARGS__);
#else
#define cy_asc_log_dbg(format, ...)
#endif
#elif ENABLE_ASC_LOGS == 2
#define cy_asc_log_info(format, ...) printf("[ASC] " format " \n", ##__VA_ARGS__);
#define cy_asc_log_err(ret_val, format, ...) printf("[ASC] #Err:0x%lx " format " \n", ret_val, ##__VA_ARGS__);
#ifdef ENABLE_ASC_DBG_LOGS
#define cy_asc_log_dbg(format, ...) printf("[ASC] " format " \n", ##__VA_ARGS__);
#else
#define cy_as_log_dbg(format, ...)
#endif
#elif ENABLE_ASC_LOGS
#define cy_asc_log_info(format, ...) cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_INFO, "[ASC] " format " \n", ##__VA_ARGS__);
#define cy_asc_log_err(ret_val, format, ...) cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_ERR, "[ASC] [Err:0x%lx] " format " \n", ret_val, ##__VA_ARGS__);
#define cy_asc_log_dbg(format, ...) cy_log_msg(CYLF_MIDDLEWARE, CY_LOG_DEBUG "[ASC] " format " \n", ##__VA_ARGS__);
#else
#define cy_asc_log_info(format, ...)
#define cy_asc_log_err(ret_val, format, ...)
#define cy_asc_log_dbg(format, ...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __CY_AUDIO_SW_CODECS_LOG_H__ */
