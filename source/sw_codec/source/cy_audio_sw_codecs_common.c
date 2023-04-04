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
 * @file cy_audio_sw_codecs_common.c
 *
 * @brief This file is the source file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include "cy_audio_sw_codecs_common.h"
#include "cy_audio_sw_codec_utils.h"
/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      Function decelerations
 ******************************************************************************/

/*******************************************************************************
 *                      Function definations
 ******************************************************************************/

/**
 * Cache the residual data if there were extra input data is provided
 *
 * @param input_buff                Internal Input buffer handle
 * @param user_buff                 User input buffer pointer
 * @param processed_user_buff_len   Bytes processed on user buffer
 * @param tot_user_buff_len         Total user buffer length
 */
void cy_asc_check_and_cache_any_residual_data(cy_audio_sw_buff_t *input_buff,
        uint8_t *user_buff, uint32_t processed_user_buff_len,
        uint32_t tot_user_buff_len)
{
    if( (processed_user_buff_len <  tot_user_buff_len) &&\
        ((tot_user_buff_len - processed_user_buff_len) < input_buff->buff_len) )
    {
        /* This some residual data which is less than recommended input size,
         * so caching to the internal input buffer.
         */
        input_buff->used_len = (tot_user_buff_len - processed_user_buff_len);
        memcpy(input_buff->buff, (user_buff+processed_user_buff_len)
                , input_buff->used_len );
    }
    else
    {
        /* Residual data greater than recommended input size, can not be cached */
        input_buff->used_len = 0;
    }
}


/**
 * check if sufficient data available to process. if the user data is less,
 * data is appended to the residual cache buffer
 *
 * @param input_buff        Internal Input buffer handle
 * @param user_buffer       User input buffer pointer
 * @param user_buffer_len   Bytes processed on user buffer
 * @return true             if user has provided sufficient data to process
 * @return false            No sufficient data available, and the data is
 *                          cached internally.
 */
bool cy_asc_check_sufficient_buffer_process(cy_audio_sw_buff_t *input_buff,
        uint8_t *user_buffer,uint32_t user_buffer_len )
{
        /* Check for sufficient input buffer to process */
    if(false == IS_SUFFICIENT_BUFF_LEN((user_buffer_len + input_buff->used_len),input_buff->buff_len))
    {
        /* Cannot encoded less data, so the data is cached internally */
        memcpy(   ( input_buff->buff + input_buff->used_len)
                , user_buffer, user_buffer_len );
        input_buff->used_len += user_buffer_len;

        cy_asc_log_dbg("Cannot encode less data, so data cached. Input prvd[user_buff_len:%lu] cachedSz:%lu"
                    , user_buffer_len, input_buff->used_len );

        return false;
    }
    return true;
}