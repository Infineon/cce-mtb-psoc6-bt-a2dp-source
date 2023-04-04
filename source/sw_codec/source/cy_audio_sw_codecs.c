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
 * @file cy_audio_sw_codecs.c
 *
 * @brief This file is the source file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */

/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include "cy_audio_sw_codecs.h"
#include "cy_audio_sw_codecs_private.h"
#include "cy_audio_sw_codecs_common.h"

/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      Const
 ******************************************************************************/

extern const cy_audio_sw_codec_encoder_ops_t *const audio_sw_encode_ops[];
extern const cy_audio_sw_codec_decoder_ops_t *const audio_sw_decode_ops[];
/*******************************************************************************
 *                      Function decelerations
 ******************************************************************************/

static cy_rslt_t cy_audio_sw_codec_encoder_init(void *codec_config, cy_audio_sw_codec_t *handle)
{
    uint8_t                     loop_idx = 0;
    cy_rslt_t                   result = CY_RSLT_ASC_NOT_SUPPORTED;
    cy_audio_sw_codec_type_t    codec_type = CY_AUDIO_CODEC_TYPE_UNKNOWN;

    codec_type = ((cy_audio_sw_codec_encode_config_t *)codec_config)->codec_type;

    for (loop_idx = 0; audio_sw_encode_ops[loop_idx];loop_idx++)
    {
        if(audio_sw_encode_ops[loop_idx]->type ==  codec_type)
        {
            result = audio_sw_encode_ops[loop_idx]->init(
                                    (cy_audio_sw_codec_encode_config_t*)codec_config, handle);
            if( CY_RSLT_SUCCESS != result)
            {
                cy_asc_log_err(result,"[ENC] Encoder-%d init failed",codec_type)
            }
            else
            {
                ((cy_audio_sw_codec_handle_t)*handle)->state            = CY_AUDIO_CODEC_STATE_INITIALIZED;
                ((cy_audio_sw_codec_handle_t)*handle)->codec_operation  = CY_AUDIO_CODEC_OPERATION_ENCODE;
                cy_asc_log_info("[ENC] Encoder[%d] init Success Sw hdl:%p",codec_type, *handle)
            }
            break;
        }
    }

    if(CY_RSLT_ASC_NOT_SUPPORTED == result)
    {
        cy_asc_log_err(result,"[ENC] Requested codec encoder operation on codec %d is not supported", codec_type)
    }

    return result;
}

static cy_rslt_t cy_audio_sw_codec_decoder_init( void *codec_config, cy_audio_sw_codec_t *handle)
{
    uint8_t                     loop_idx = 0;
    cy_rslt_t                   result = CY_RSLT_ASC_NOT_SUPPORTED;
    cy_audio_sw_codec_type_t    codec_type = CY_AUDIO_CODEC_TYPE_UNKNOWN;

    codec_type = ((cy_audio_sw_codec_decode_config_t *)codec_config)->codec_type;

    for (loop_idx = 0; audio_sw_decode_ops[loop_idx];loop_idx++)
    {
        if(audio_sw_decode_ops[loop_idx]->type ==  codec_type)
        {
            result = audio_sw_decode_ops[loop_idx]->init(
                                    (cy_audio_sw_codec_decode_config_t*)codec_config, handle);
            if( CY_RSLT_SUCCESS != result)
            {
                cy_asc_log_err(result,"decoder-%d init failed",codec_type)
            }
            else
            {
                ((cy_audio_sw_codec_handle_t)*handle)->state            = CY_AUDIO_CODEC_STATE_INITIALIZED;
                ((cy_audio_sw_codec_handle_t)*handle)->codec_operation  = CY_AUDIO_CODEC_OPERATION_DECODE;
                cy_asc_log_info("decoder[%d] init Success Sw hdl:%p",codec_type, *handle)
            }
        }
    }

    if(CY_RSLT_ASC_NOT_SUPPORTED == result)
    {
        cy_asc_log_err(result,"Requested codec decoder operation on codec %d is not supported", codec_type)
    }

    return result;
}

cy_rslt_t cy_audio_sw_codec_init(cy_audio_sw_codec_operation_t codec_operation,
        void *codec_config, cy_audio_sw_codec_t *handle)
{
    cy_rslt_t                   result = CY_RSLT_ASC_NOT_SUPPORTED;

    if(NULL == codec_config || NULL == handle)
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Nil param H:%p config:%p", handle, codec_config);
        return CY_RSLT_ASC_BAD_ARG;
    }

    *handle = NULL;
    if( CY_AUDIO_CODEC_OPERATION_ENCODE == codec_operation )
    {
        /* Encoder init */
        result = cy_audio_sw_codec_encoder_init(codec_config,handle);
    }
    else if(CY_AUDIO_CODEC_OPERATION_DECODE == codec_operation )
    {
        /* Decoder init */
        result = cy_audio_sw_codec_decoder_init(codec_config,handle);
    }
    else
    {
        cy_asc_log_err(result,"Requested codec operation:%d is not supported", codec_operation);
        return result;
    }

    if(CY_RSLT_ASC_NOT_SUPPORTED == result)
    {
        cy_asc_log_err(result,"Requested codec operation:%d on codec is not supported "
                            , codec_operation)
    }

    if(CY_RSLT_SUCCESS != result)
    {
        *handle = NULL;
    }

    return result;
}

cy_rslt_t cy_audio_sw_codec_encode(cy_audio_sw_codec_t handle,
        uint8_t *in_pcm_buff, uint32_t *in_pcm_data_len, uint8_t *out_buff,
        uint32_t *outbuf_len)
{
    cy_audio_sw_codec_encoder_ops_t *   audio_sw_decode_ops = NULL;
    cy_audio_sw_codec_handle_t          asc_handle          = NULL;
    cy_rslt_t                           result              = CY_RSLT_SUCCESS;

    if ( ( NULL == handle )  || (NULL == in_pcm_buff) || (0 == in_pcm_data_len) ||\
         (NULL == out_buff) || (0 == outbuf_len) )
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode operation");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    asc_handle = (cy_audio_sw_codec_handle_t)handle;
    if( (CY_AUDIO_CODEC_STATE_INITIALIZED != asc_handle->state) ||\
    ( CY_AUDIO_CODEC_OPERATION_ENCODE != asc_handle->codec_operation ) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Bad arguments State:%d CodecOps:%d", asc_handle->state, asc_handle->codec_operation );
        return CY_RSLT_ASC_BAD_ARG;
    }

    if( asc_handle->pst_audio_sw_codec_ops )
    {
        audio_sw_decode_ops = (cy_audio_sw_codec_encoder_ops_t*)asc_handle->pst_audio_sw_codec_ops;
        result = audio_sw_decode_ops->encode(asc_handle, in_pcm_buff,in_pcm_data_len, out_buff, outbuf_len );
        if(CY_RSLT_SUCCESS == result)
        {
#ifdef ENABLE_STATS
            asc_handle->statistics.enc_stat.TotalEncodedData += *outbuf_len;
            asc_handle->statistics.enc_stat.TotalInputData  += *in_pcm_data_len;
#endif /* ENABLE_STATS */
        }
    }
    else
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode operation");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    return result;
}

cy_rslt_t cy_audio_sw_codec_decode(cy_audio_sw_codec_t handle,
        uint8_t *in_encoded_buff, uint32_t *in_encoded_data_len,
        uint8_t *out_buff, uint32_t *outbuf_len)
{
    cy_audio_sw_codec_encoder_ops_t *   audio_sw_decode_ops = NULL;
    cy_audio_sw_codec_handle_t          asc_handle          = NULL;
    cy_rslt_t                           result              = CY_RSLT_SUCCESS;

    if ( ( NULL == handle )  || (NULL == in_encoded_buff) || (NULL == in_encoded_data_len) ||\
         (NULL == out_buff) || (NULL == outbuf_len) )
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode operation");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    asc_handle = (cy_audio_sw_codec_handle_t)handle;
    if( (CY_AUDIO_CODEC_STATE_INITIALIZED != asc_handle->state) ||\
        (CY_AUDIO_CODEC_OPERATION_DECODE != asc_handle->codec_operation) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Bad arguments State:%d CodecOps:%d", asc_handle->state, asc_handle->codec_operation );
        return CY_RSLT_ASC_BAD_ARG;
    }

    if( asc_handle->pst_audio_sw_codec_ops )
    {
        audio_sw_decode_ops = (cy_audio_sw_codec_encoder_ops_t*)asc_handle->pst_audio_sw_codec_ops;
        result = audio_sw_decode_ops->encode(asc_handle, in_encoded_buff, in_encoded_data_len, out_buff, outbuf_len );
        if(CY_RSLT_SUCCESS == result)
        {
#ifdef ENABLE_STATS
            asc_handle->statistics.dec_stat.TotalDecodedData += *outbuf_len;
            asc_handle->statistics.dec_stat.TotalInputData   += *in_encoded_data_len;
#endif /* ENABLE_STATS */
        }
    }
    else
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode operation");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    return result;
}

cy_rslt_t cy_audio_sw_codec_get_encoder_recommended_inbuf_size(
        cy_audio_sw_codec_t handle,
        uint32_t *recommended_in_size)
{
    cy_audio_sw_codec_encoder_ops_t *   audio_sw_encoder_ops = NULL;
    cy_audio_sw_codec_handle_t          asc_handle          = NULL;
    cy_rslt_t                           result              = CY_RSLT_ASC_GENERIC_ERROR;

    if ( NULL == recommended_in_size || NULL == handle )
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE
                            ,"Invalid handle to perfrom encode operation Hdl:%p recommended_out_size:%p"
                            , asc_handle, recommended_in_size);
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    asc_handle = (cy_audio_sw_codec_handle_t)handle;
    if( (CY_AUDIO_CODEC_STATE_INITIALIZED != asc_handle->state) ||\
        ( CY_AUDIO_CODEC_OPERATION_ENCODE != asc_handle->codec_operation ) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Bad arguments State:%d CodecOps:%d", asc_handle->state, asc_handle->codec_operation );
        return CY_RSLT_ASC_BAD_ARG;
    }

    if( asc_handle->pst_audio_sw_codec_ops )
    {
        audio_sw_encoder_ops = (cy_audio_sw_codec_encoder_ops_t*)asc_handle->pst_audio_sw_codec_ops;
        result = audio_sw_encoder_ops->get_recommended_inbuf_size(asc_handle, recommended_in_size);
    }
    else
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode operation");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    return result;
}

cy_rslt_t cy_audio_sw_codec_get_encoder_recommended_outbuf_size(
        cy_audio_sw_codec_t handle,
        uint32_t in_size,
        uint32_t *recommended_out_size)
{
    cy_audio_sw_codec_encoder_ops_t *   audio_sw_encoder_ops = NULL;
    cy_audio_sw_codec_handle_t          asc_handle           = NULL;
    cy_rslt_t                           result               = CY_RSLT_ASC_GENERIC_ERROR;

    if ( NULL == recommended_out_size || NULL == handle )
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE
                            ,"Invalid handle to perfrom encode operation Hdl:%p recommended_out_size:%p"
                            , asc_handle, recommended_out_size);
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    asc_handle = (cy_audio_sw_codec_handle_t)handle;
    if( (CY_AUDIO_CODEC_STATE_INITIALIZED != asc_handle->state) ||\
        ( CY_AUDIO_CODEC_OPERATION_ENCODE != asc_handle->codec_operation ) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Bad arguments State:%d CodecOps:%d", asc_handle->state, asc_handle->codec_operation );
        return CY_RSLT_ASC_BAD_ARG;
    }

    if( asc_handle->pst_audio_sw_codec_ops )
    {
        audio_sw_encoder_ops = (cy_audio_sw_codec_encoder_ops_t*)asc_handle->pst_audio_sw_codec_ops;
        result = audio_sw_encoder_ops->get_recommended_outbuf_size(asc_handle, in_size,recommended_out_size);
    }
    else
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode operation");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    return result;
}

cy_rslt_t cy_audio_sw_codec_get_decoder_recommended_outbuf_size(
        cy_audio_sw_codec_t handle,
        cy_audio_sw_codec_stream_info_t * stream_info,
        uint32_t *recommended_out_size)
{
    cy_audio_sw_codec_decoder_ops_t *   audio_sw_decoder_ops = NULL;
    cy_audio_sw_codec_handle_t          asc_handle           = NULL;
    cy_rslt_t                           result               = CY_RSLT_ASC_GENERIC_ERROR;

    if ( NULL == recommended_out_size || NULL == handle )
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE
                            ,"Invalid handle to perfrom decode operation Hdl:%p recommended_out_size:%p"
                            , asc_handle, recommended_out_size);
        return CY_RSLT_ASC_BAD_ARG;
    }

    asc_handle = (cy_audio_sw_codec_handle_t)handle;
    if( (CY_AUDIO_CODEC_STATE_INITIALIZED != asc_handle->state) ||\
        (CY_AUDIO_CODEC_OPERATION_DECODE != asc_handle->codec_operation) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Bad arguments State:%d CodecOps:%d", asc_handle->state, asc_handle->codec_operation );
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    if( asc_handle->pst_audio_sw_codec_ops )
    {
        audio_sw_decoder_ops = (cy_audio_sw_codec_decoder_ops_t*)asc_handle->pst_audio_sw_codec_ops;
        result = audio_sw_decoder_ops->get_recommended_size(asc_handle,stream_info, recommended_out_size);
    }
    else
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode operation");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    return result;
}

cy_rslt_t cy_audio_sw_codec_decode_stream_info_header(
            cy_audio_sw_codec_t handle,
            uint8_t *in_encoded_buff,
            uint32_t *in_encoded_data_len,
            cy_audio_sw_codec_stream_info_t *stream_info)
{
    cy_audio_sw_codec_decoder_ops_t *   audio_sw_decoder_ops = NULL;
    cy_audio_sw_codec_handle_t          asc_handle           = NULL;
    cy_rslt_t                           result               = CY_RSLT_ASC_GENERIC_ERROR;

    if ( ( NULL == handle )  || (NULL == in_encoded_buff) || (NULL == in_encoded_data_len) || NULL == stream_info )
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE
                            ,"Invalid handle to perfrom decode operation Hdl:%p in_encoded_buff:%p in_encoded_data_len:%p stream_info:%p"
                            , handle, in_encoded_buff, in_encoded_data_len, stream_info);
        return CY_RSLT_ASC_BAD_ARG;
    }

    asc_handle = (cy_audio_sw_codec_handle_t)handle;
    if( (CY_AUDIO_CODEC_STATE_INITIALIZED != asc_handle->state) ||\
        ( CY_AUDIO_CODEC_OPERATION_DECODE != asc_handle->codec_operation ) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Bad arguments State:%d CodecOps:%d", asc_handle->state, asc_handle->codec_operation );
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    if( asc_handle->pst_audio_sw_codec_ops )
    {
        audio_sw_decoder_ops = (cy_audio_sw_codec_decoder_ops_t*)asc_handle->pst_audio_sw_codec_ops;
        result = audio_sw_decoder_ops->decode_stream_info_header(handle,in_encoded_buff,in_encoded_data_len,stream_info);
    }
    else
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode operation");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    return result;
}

cy_rslt_t cy_audio_sw_codec_deinit(cy_audio_sw_codec_t *handle)
{
    cy_audio_sw_codec_handle_t          asc_handle          = NULL;
    cy_rslt_t                           result              = CY_RSLT_ASC_GENERIC_ERROR;

    if( NULL == handle  )
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode deinit");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    asc_handle = *handle;
    cy_asc_log_info("deinit Hdl:%p Ops:%d",asc_handle, asc_handle->codec_operation);
    if(( NULL == asc_handle ) || (CY_AUDIO_CODEC_STATE_INITIALIZED != asc_handle->state) )
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle/state to perfrom encode deinit");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    if( asc_handle->pst_audio_sw_codec_ops )
    {
        result = ((cy_audio_sw_codec_encoder_ops_t*)asc_handle->pst_audio_sw_codec_ops)->deinit(handle);
        if( CY_RSLT_SUCCESS == result)
        {
            cy_asc_log_info("deinit success");
        }
    }
    else
    {
        cy_asc_log_err(CY_RSLT_ASC_INVALID_HANDLE,"Invalid handle to perfrom encode operation");
        return CY_RSLT_ASC_INVALID_HANDLE;
    }

    return result;
}
