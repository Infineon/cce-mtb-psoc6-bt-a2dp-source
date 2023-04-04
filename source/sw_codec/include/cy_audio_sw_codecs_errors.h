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
 * @file cy_audio_sw_codecs_errors.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef AUDIO_SEND_RECEIVE_UTILITY_SOURCE_AUDIO_SW_CODECS_SRC_INC_CY_AUDIO_SW_CODECS_ERRORS_H_
#define AUDIO_SEND_RECEIVE_UTILITY_SOURCE_AUDIO_SW_CODECS_SRC_INC_CY_AUDIO_SW_CODECS_ERRORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "cy_result.h"
//#include "cy_result_mw.h"

#ifndef CY_RSLT_MODULE_ASC_BASE
#define CY_RSLT_MODULE_ASC_BASE       CY_RSLT_MODULE_MIDDLEWARE_BASE + 20
#endif /* !CY_RSLT_MODULE_ASC_BASE */

/**
 * \defgroup staged_voice_control_results Staged voice control (SVC) results/error codes
 * @ingroup group_staged_voice_control_macros
 *
 * staged voice middleware APIs return results of type cy_rslt_t and consist of three parts:
 * - module base
 * - type
 * - error code
 *
 * \par Result Format
 *
   \verbatim
              Module base                            Type     Library-specific error code
      +--------------------------------------------+--------+------------------------------+
      |   CY_RSLT_MODULE_ASC_BASE       | 0x2   |           Error Code         |
      +--------------------------------------------+--------+------------------------------+
                            14 bits               2 bits            16 bits

   See the macro section of this document for library-specific error codes.
   \endverbatim
 *
 * The data structure cy_rslt_t is part of cy_result.h located in <core_lib/include>
 *
 * Module base: This base is derived from CY_RSLT_MODULE_MIDDLEWARE_BASE (defined in cy_result.h) and is an offset of the CY_RSLT_MODULE_MIDDLEWARE_BASE.
 *              The details of the offset and the middleware base are defined in cy_result_mw.h, which is part of the [GitHub connectivity-utilities] (https://github.com/cypresssemiconductorco/connectivity-utilities) repo.
 *              For example, the buffer pool library uses CY_RSLT_MODULE_ASC_BASE as the module base.
 *
 * Type: This type is defined in cy_result.h and can be one of CY_RSLT_TYPE_FATAL, CY_RSLT_TYPE_ERROR, CY_RSLT_TYPE_WARNING, or CY_RSLT_TYPE_INFO. AWS library error codes are of type CY_RSLT_TYPE_ERROR.
 *
 * Library-specific error code: These error codes are library-specific and defined in the macro section.
 *
 * Helper macros used for creating the library-specific result are provided as part of cy_result.h.
 * \{
 */

/** SVC error code base. */
#define CY_RSLT_ASC_ERR_BASE                         CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR, CY_RSLT_MODULE_ASC_BASE, 0)

#define CY_RSLT_ASC_OUT_OF_MEMORY                       ( CY_RSLT_ASC_ERR_BASE + 1 )
#define CY_RSLT_ASC_GENERIC_ERROR                       ( CY_RSLT_ASC_ERR_BASE + 2 )
#define CY_RSLT_ASC_BAD_ARG                             ( CY_RSLT_ASC_ERR_BASE + 3 )
#define CY_RSLT_ASC_INVALID_HANDLE                      ( CY_RSLT_ASC_ERR_BASE + 4 )
#define CY_RSLT_ASC_ALREADY_INITIALIZED                 ( CY_RSLT_ASC_ERR_BASE + 5 )
#define CY_RSLT_ASC_INVALID_MAX_INSTANCE_REACHED        ( CY_RSLT_ASC_ERR_BASE + 6 )
#define CY_RSLT_ASC_INSTANCE_NOT_FOUND                  ( CY_RSLT_ASC_ERR_BASE + 7 )
#define CY_RSLT_ASC_ALGO_INIT_FAILED                    ( CY_RSLT_ASC_ERR_BASE + 8 )
#define CY_RSLT_ASC_NOT_SUPPORTED                       ( CY_RSLT_ASC_ERR_BASE + 9 )
#define CY_RSLT_ASC_ALGO_INTERNAL_ERROR                 ( CY_RSLT_ASC_ERR_BASE + 10 )
#define CY_RSLT_ASC_INSUFFICIENT_INPUT_BUFF             ( CY_RSLT_ASC_ERR_BASE + 11 )
#define CY_RSLT_ASC_INSUFFICIENT_OUTPUT_BUFF            ( CY_RSLT_ASC_ERR_BASE + 12 )
#define CY_RSLT_ASC_HEADER_NOT_FOUND                    ( CY_RSLT_ASC_ERR_BASE + 13 )
#define CY_RSLT_ASC_PARTIAL_HEADER_IDENTIFIED          ( CY_RSLT_ASC_ERR_BASE + 14 )
#define CY_RSLT_ASC_CORRUPTED_DATA                      ( CY_RSLT_ASC_ERR_BASE + 15 )
#define CY_RSLT_ASC_PARTIAL_FRAME_LEN                  ( CY_RSLT_ASC_ERR_BASE + 16 )
#define CY_RSLT_ASC_HEADER_PARSING_ERROR                ( CY_RSLT_ASC_ERR_BASE + 17 )

/** \} group_staged_voice_control_macros */
#ifdef __cplusplus
} /*extern "C" */
#endif


#endif /* AUDIO_SEND_RECEIVE_UTILITY_SOURCE_AUDIO_SW_CODECS_SRC_INC_CY_AUDIO_SW_CODECS_ERRORS_H_ */
