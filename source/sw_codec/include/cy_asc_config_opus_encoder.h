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
 * @file cy_asc_config_opus.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

#ifndef __CY_ASC_CONFIG_OPUS_ENCODER_H__
#define __CY_ASC_CONFIG_OPUS_ENCODER_H__

/**
 * OPUS Encoder supported formats:
 *  Bit with         : 16
 *  Sampling rate    : 16000 32000 48000
 *  Channel          : mono stereo
 */

/** The below were config specific to opus encoder
 * these below macros were used in the opus codec middleware - Changing
 * any parameter requires a rebuild of the middleware
 */
#define config_opus_enc_APPLICATION              OPUS_APPLICATION_AUDIO
#define config_opus_enc_BANDWIDTH                OPUS_AUTO
#define config_opus_enc_VBR                      0
#define config_opus_enc_CONSTRAINT               0
#define config_opus_enc_COMPLEXITY               4
#define config_opus_enc_FORCE_CHANNELS           OPUS_AUTO
#define config_opus_enc_FORCE_MODE               MODE_CELT_ONLY
#define config_opus_enc_EXPERT_FRAME_DURATION    OPUS_FRAMESIZE_ARG

#endif /* __CY_ASC_CONFIG_OPUS_ENCODER_H__ */
