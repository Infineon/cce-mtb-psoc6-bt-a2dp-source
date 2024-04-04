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

/** @file cy_audio_sw_sbc_encoder.c
 *  Functions to perform SBC encoder operations
 *
 */


/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#ifdef ENABLE_SBC_ENCODE

#include "cy_audio_sw_sbc_encoder.h"
#include "cy_audio_sw_codec_utils.h"
#include "cy_audio_sw_codecs_private.h"
#include "cy_audio_sw_sbc_encoder_private.h"
/* Config param include */
#include "cy_asc_config_sbc_encoder.h"
#include "cy_audio_sw_sbc_utils.h"

/*******************************************************************************
 *                      Macros
 ******************************************************************************/
#define SAMPLE_WIDTH 2 /*16 bit*/
/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      Constant
 ******************************************************************************/
const cy_audio_sw_codec_encoder_ops_t cy_audio_sw_encode_sbc = {
    .type = CY_AUDIO_CODEC_TYPE_SBC,
    .init = cy_audio_sw_sbc_encoder_init,
    .encode = cy_audio_sw_sbc_encode,
    .get_recommended_inbuf_size = cy_audio_sw_sbc_get_encoder_recommended_inbuf_size,
    .get_recommended_outbuf_size = cy_audio_sw_sbc_get_encoder_recommended_outbuf_size,
    .deinit = cy_audio_sw_sbc_encoder_deinit
};

#define SBC_UUID_2_INDEX 0
static cy_asc_sbc_encode_config_t sbc_wide_band_param[SBC_UUID_2_INDEX+1] = {
    {//0
    .allocation_method = SBC_LOUDNESS,
    .bit_pool = 26,
    .channel_mode = 0, //mono
    .sub_bands = 8,
    .sub_blocks = 15,
    .sbc_mode = SBC_MODE_WIDE_BAND,
    .uui_id = 2,
    }
};

/* Currently SBC algo supports only single instance */
static volatile bool g_instance_active = false;

#ifdef VERIFY_INPUT_FRAME_MATCH
#include "cy_utils.h"
static uint32_t FrameCount = 0;
#endif

/*******************************************************************************
 *                      Function decelerations
 ******************************************************************************/

static cy_rslt_t cy_asc_sbc_encode_frame(
            sbc_encoder_handle_t sbc_object_hdl, uint8_t *pcm_bytes,
            uint32_t buff_sz, uint8_t *sbc_buff, uint32_t *out_bytes );

static cy_rslt_t cy_asc_configure_and_init_sbc_encoder( SBC_ENC_PARAMS *enc,
                                    const cy_audio_sw_codec_encode_config_t *codec_config);

static void cy_asc_destroy_sbc_encoder(sbc_encoder_handle_t handle);

static cy_rslt_t cy_asc_create_sbc_enc_internal_hdl(
                        sbc_encoder_handle_t * a_handle,
                        const cy_audio_sw_codec_encode_config_t *codec_config);

/*******************************************************************************
 *                      Function definations
 ******************************************************************************/

static cy_rslt_t cy_asc_is_valid_sbc_enc_config(const cy_audio_sw_codec_encode_config_t *codec_config)
{
    cy_asc_sbc_encode_config_t sbc = codec_config->codec_specific_config.sbc;

    cy_asc_log_info("config pram Bit: %d, CH: %d, SR: %d"
                        , codec_config->bitwidth
                        , codec_config->channel
                        , codec_config->sampling_rate );

    cy_asc_log_info("config pram sbc band:%u block:%u AM:%u CH mode:%u bit_pool:%u sbc mode:%u uui_id:%u"
                                ,sbc.sub_bands
                                ,sbc.sub_blocks
                                ,sbc.allocation_method
                                ,sbc.channel_mode
                                ,sbc.bit_pool
                                ,sbc.sbc_mode
                                ,sbc.uui_id );

    if( !((codec_config->bitwidth > CY_AUDIO_CODEC_BITWIDTH_UNKNOWN) &&\
        (codec_config->bitwidth < CY_AUDIO_CODEC_BITWIDTH_32) &&\
        (codec_config->channel > CY_AUDIO_CODEC_CHANNEL_UNKNOWN) &&\
        (codec_config->channel < CY_AUDIO_CODEC_CHANNEL_MAX ) &&\
        (codec_config->sampling_rate > CY_AUDIO_CODEC_SAMPLING_RATE_UNKNOWN) &&\
        (codec_config->sampling_rate < CY_AUDIO_CODEC_SAMPLING_RATE_MAX) ) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid config pram Bit: %d, CH: %d, SR: %d"
                                , codec_config->bitwidth
                                , codec_config->channel
                                , codec_config->sampling_rate );
        return CY_RSLT_ASC_BAD_ARG;
    }

    if(codec_config->bitwidth !=  CY_AUDIO_CODEC_BITWIDTH_16 )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Currently sbc mw is tested with only 16bit audio samples.");
        return CY_RSLT_ASC_BAD_ARG;
    }

    /**
     * SBC mode,  0 for A2DP mode, 1 for WideBand mode
     */
    if( sbc.sbc_mode != SBC_MODE_A2DP &&  sbc.sbc_mode != SBC_MODE_WIDE_BAND)
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid channel mode: %u",sbc.sbc_mode);
        return CY_RSLT_ASC_BAD_ARG;
    }

    if(sbc.sbc_mode == SBC_MODE_A2DP)
    {
        /**
         *  Number of sub bands - (4 or 8)
         */
        if( sbc.sub_bands != 4 && sbc.sub_bands != 8 )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid sub band: %u",sbc.sub_bands);
            return CY_RSLT_ASC_BAD_ARG;
        }

        /**
         * Number of sub blocks (4 or 8 or 12 or 16)
         */
        if( sbc.sub_blocks != 4 && sbc.sub_blocks != 8 &&\
            sbc.sub_blocks != 12 && sbc.sub_blocks != 16)
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid sub block: %u",sbc.sub_blocks);
            return CY_RSLT_ASC_BAD_ARG;
        }

        /**
         * Allocation method (Loudness:0 (SBC_LOUDNESS) , SNR: 1(SBC_SNR))
         */
        if( sbc.allocation_method != SBC_LOUDNESS &&  sbc.allocation_method != SBC_SNR)
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid allocation method: %u",sbc.allocation_method);
            return CY_RSLT_ASC_BAD_ARG;
        }

        /**
         * channel mode (0 for mono, 1 for Dual, 2 for stereo, 3 for joint stereo)
         */
        if( sbc.channel_mode > 3 )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid channel mode: %u",sbc.channel_mode);
            return CY_RSLT_ASC_BAD_ARG;
        }

        /**
         * Bit pool, ranging from 2 to 250
         */
        if( sbc.bit_pool < SBC_DEC_BIT_POOL_MIN || sbc.bit_pool > SBC_DEC_BIT_POOL_MAX )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid bit pool: %u",sbc.bit_pool);
            return CY_RSLT_ASC_BAD_ARG;
        }



        if ( ((codec_config->channel == CY_AUDIO_CODEC_CHANNEL_MONO) && (sbc.channel_mode != 0)) ||\
            ((codec_config->channel == CY_AUDIO_CODEC_CHANNEL_STEREO) && (sbc.channel_mode == 0)) )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Channel mode is not adapted for this input: CH:%d CH mode:%u",codec_config->channel,sbc.channel_mode);
            return CY_RSLT_ASC_NOT_SUPPORTED;
        }
    }
    else
    {
        if( sbc.uui_id != SBC_WB_SUPPORTED_UUID )
        {
            cy_asc_log_err(CY_RSLT_ASC_NOT_SUPPORTED, "Unsupported UUID: %u, only uuid 2 is supported",sbc.uui_id);
            return CY_RSLT_ASC_NOT_SUPPORTED;
        }

        if(codec_config->channel != CY_AUDIO_CODEC_CHANNEL_MONO)
        {
            cy_asc_log_err(CY_RSLT_ASC_NOT_SUPPORTED, "UUID: %u Support only mono channels",sbc.uui_id);
            return CY_RSLT_ASC_NOT_SUPPORTED;
        }

        if(codec_config->sampling_rate != CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ)
        {
            cy_asc_log_err(CY_RSLT_ASC_NOT_SUPPORTED, "Support input sample rate:%d in wideband mode ",codec_config->sampling_rate);
            return CY_RSLT_ASC_NOT_SUPPORTED;
        }
    }

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t cy_asc_sbc_get_pcm_frame_len (cy_asc_sbc_encode_config_t sbc, uint32_t *a_frameLen)
{
    uint8_t num_channels = (sbc.channel_mode == 0) ? 1 : 2;
    if(sbc.sbc_mode == SBC_MODE_WIDE_BAND)
    {
        sbc = sbc_wide_band_param[SBC_UUID_2_INDEX];
    }

    *a_frameLen = SAMPLE_WIDTH * (SBC_MAX_NUM_FRAME*sbc.sub_blocks*sbc.sub_bands*num_channels);

    return CY_RSLT_SUCCESS;
}

static cy_rslt_t cy_asc_sbc_get_encoded_frame_len (cy_asc_sbc_encode_config_t sbc, uint32_t *a_frameLen)
{
    uint8_t  num_channels, Join;
    uint32_t  FrameLen, tmp;

    if((sbc.sbc_mode == SBC_MODE_WIDE_BAND) && (sbc.uui_id != SBC_WB_SUPPORTED_UUID) )
    {
        cy_asc_log_err(CY_RSLT_ASC_NOT_SUPPORTED, "Unsupported UUID: %u, only uuid 2 is supported",sbc.uui_id);
        return CY_RSLT_ASC_NOT_SUPPORTED;
    }

    if(sbc.sbc_mode == SBC_MODE_WIDE_BAND)
    {
        sbc = sbc_wide_band_param[SBC_UUID_2_INDEX];
    }

    num_channels = (sbc.channel_mode == 0) ? 1 : 2;

    FrameLen = 4 + (4 * sbc.sub_bands * num_channels) / 8;

    Join = (sbc.channel_mode == 3) ? 1 : 0;

    if (sbc.channel_mode < 2)
        tmp =  (sbc.sub_blocks * num_channels * sbc.bit_pool);
    else
        tmp =  (Join * sbc.sub_bands + sbc.sub_blocks * sbc.bit_pool);

    FrameLen += tmp / 8 + ((tmp % 8) ? 1 : 0);

    *a_frameLen = FrameLen;

    return CY_RSLT_SUCCESS;
}

/**
 * @brief Create a sbc internal hdl object
 *
 * @param a_handle [out] - sbc object handle
 * @param codec_config [in] - encoder config
 * @return cy_rslt_t - CY_RSLT_SUCCESS on success; an error code on failure.
 */
static cy_rslt_t cy_asc_create_sbc_enc_internal_hdl(
                        sbc_encoder_handle_t * a_handle,
                        const cy_audio_sw_codec_encode_config_t *codec_config)
{
    sbc_encoder_handle_t handle = NULL;
    cy_rslt_t result = CY_RSLT_SUCCESS;
    SBC_ENC_PARAMS *enc;

    if(NULL != codec_config)
    {

        result = cy_asc_is_valid_sbc_enc_config(codec_config);
        if( CY_RSLT_SUCCESS != result )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid SBC config pram");
            return CY_RSLT_ASC_NOT_SUPPORTED;
        }

        /* Set initially as failure */
        result = CY_RSLT_ASC_ALGO_INIT_FAILED;

        handle = (sbc_encoder_handle_t)calloc(1,sizeof(sbc_encoder_handle));
        if(NULL == handle)
        {
            result = CY_RSLT_ASC_OUT_OF_MEMORY;
            cy_asc_log_err(result, "out of memory handle:0x%p-sz%u",handle,sizeof(sbc_encoder_handle));
            goto api_exit;
        }

        enc = (SBC_ENC_PARAMS*)calloc(1,sizeof(SBC_ENC_PARAMS));
        if(NULL == enc)
        {
            result = CY_RSLT_ASC_OUT_OF_MEMORY;
            cy_asc_log_err(result, "out of memory SBC_ENC_PARAMS:0x%p-sz%u",enc,sizeof(SBC_ENC_PARAMS));
            goto api_exit;
        }

        /* Backup the encoder handle */
        handle->encoder = enc;

        /* Configures the Encoder for CELT codec Mode */
        result = cy_asc_configure_and_init_sbc_encoder(enc,codec_config);
        if(CY_RSLT_SUCCESS == result)
        {
            handle->enc_config = *codec_config;
            cy_asc_sbc_get_pcm_frame_len(handle->enc_config.codec_specific_config.sbc, &handle->frameInfo.input_size );
            cy_asc_sbc_get_encoded_frame_len(handle->enc_config.codec_specific_config.sbc, &handle->frameInfo.output_size);

            /* Allocate memory for input buffer */
            handle->input_buff.buff         = (uint8_t *)malloc(handle->frameInfo.input_size);
            if(NULL == handle->input_buff.buff )
            {
                result = CY_RSLT_ASC_OUT_OF_MEMORY;
                cy_asc_log_err(result, "out of memory buff:0x%p-sz%lu",handle->input_buff.buff,handle->frameInfo.input_size);
                goto api_exit;
            }

            handle->input_buff.buff_len     = handle->frameInfo.input_size;
            handle->input_buff.used_len     = 0;
            *a_handle                       = handle;
            result                          = CY_RSLT_SUCCESS;

            cy_asc_log_info("SBC config input_size: %lu output_size: %lu"
                                    , handle->frameInfo.input_size
                                    , handle->frameInfo.output_size);
        }
    }

api_exit:
    if(CY_RSLT_SUCCESS != result)
    {
        cy_asc_destroy_sbc_encoder(handle);
        *a_handle = NULL;
    }
    return result;
}

static void cy_asc_destroy_sbc_encoder(sbc_encoder_handle_t handle)
{
    if(handle)
    {
        if(handle->encoder)
        {
            /* Destroy the algo handle */
            free(handle->encoder);
            handle->encoder = NULL;
        }

        if(handle->input_buff.buff)
        {
            free(handle->input_buff.buff);
            handle->input_buff.buff = NULL;
        }

        free(handle);
    }
}

static cy_rslt_t cy_asc_configure_and_init_sbc_encoder( SBC_ENC_PARAMS *enc,
                                    const cy_audio_sw_codec_encode_config_t *codec_config)
{
    cy_rslt_t result = CY_RSLT_ASC_ALGO_INIT_FAILED;
    SINT16 err;
    cy_asc_sbc_encode_config_t sbc = codec_config->codec_specific_config.sbc;

    enc->s16NumOfSubBands       = sbc.sub_bands;
    enc->s16NumOfBlocks         = sbc.sub_blocks;
    enc->s16AllocationMethod    = sbc.allocation_method;
    enc->s16ChannelMode         = sbc.channel_mode;
    enc->s16BitPool             = sbc.bit_pool;
    enc->sbc_mode               = sbc.sbc_mode;
    enc->uui_id                 = sbc.uui_id;
    enc->s16SamplingFreq        = cy_asc_sbc_sample_rate_enum_to_value(codec_config->sampling_rate);

    // Init SBC Encoder
    err = SBC_Encoder_Init(enc);
    if ( err == SBC_FAILURE)
    {
        result = CY_RSLT_ASC_ALGO_INIT_FAILED;
        cy_asc_log_err(result,"SBC_Encoder_Init Failed err:%x",err );
        goto api_exit;
    }

    result = CY_RSLT_SUCCESS;

api_exit:
    return result;
}

static cy_rslt_t cy_asc_sbc_encode_frame(
            sbc_encoder_handle_t sbc_object_hdl, uint8_t *pcm_bytes,
            uint32_t buff_sz, uint8_t *sbc_buff, uint32_t *out_bytes )
{
    int len;
    unsigned int num_samples;
    sbc_encoder_frame_info  frameInfo = sbc_object_hdl->frameInfo;

    if(buff_sz != frameInfo.input_size)
    {
        cy_asc_log_err(CY_RSLT_ASC_INSUFFICIENT_INPUT_BUFF,"Fatal! input sz less Req:%lu buff_sz:%lu"
                                                    , frameInfo.input_size
                                                    , buff_sz );
        return CY_RSLT_ASC_INSUFFICIENT_INPUT_BUFF;
    }

#ifdef VERIFY_INPUT_FRAME_MATCH
        bool is_same = false;
        extern bool verify_enc_Inputdata(uint8_t * Data_to_encode, uint32_t Size, uint32_t a_FrameCount);
        is_same = verify_enc_Inputdata(pcm_bytes, frameInfo.input_size,FrameCount);
        CY_ASSERT (is_same == true);
#endif

     /* Encodes the PCM frame and returns the no of encoded samples */
    num_samples = buff_sz/SAMPLE_WIDTH;
    len = SBC_Encoder_encode(sbc_object_hdl->encoder, pcm_bytes, sbc_buff, num_samples);

    if (len <= 0)
    {
        cy_asc_log_err(CY_RSLT_ASC_ALGO_INTERNAL_ERROR, "sbc_encode() returned failed %d", len);
        return CY_RSLT_ASC_ALGO_INTERNAL_ERROR;
    }

    *out_bytes = len;
#ifdef VERIFY_INPUT_FRAME_MATCH
    FrameCount++;
#endif
    return CY_RSLT_SUCCESS;
}

cy_rslt_t cy_audio_sw_sbc_encoder_init(
      cy_audio_sw_codec_encode_config_t *codec_config,
      cy_audio_sw_codec_t *handle)
{
    cy_audio_sw_codec_handle_t         audio_sw_hdl    = NULL;
    sbc_encoder_handle_t               codec_obj       = NULL;
    cy_rslt_t                           result          = CY_RSLT_SUCCESS;

    if(true == g_instance_active)
    {
        result  = CY_RSLT_ASC_INVALID_MAX_INSTANCE_REACHED;
        cy_asc_log_err(result, "Already a SBC encoder instance is active");
        return result;
    }

    /* Allocate memory for the handle */
    audio_sw_hdl = (cy_audio_sw_codec_handle_t)malloc(sizeof(cy_audio_sw_codec_handle));
    if(NULL == audio_sw_hdl)
    {
        result = CY_RSLT_ASC_OUT_OF_MEMORY;
        cy_asc_log_err(result, "out of memory sw codec hdl");
        goto error_return;
    }
    memset(audio_sw_hdl,0x00,sizeof(cy_audio_sw_codec_handle));

    /* Initialize API structure and set config params */
    result = cy_asc_create_sbc_enc_internal_hdl(&codec_obj,codec_config);
    if(CY_RSLT_SUCCESS != result)
    {
        cy_asc_log_err(result, "Failed to create sbc internal hdl.");
        goto error_return;
    }

    audio_sw_hdl->audio_sw_algo_object      = codec_obj;
    audio_sw_hdl->pst_audio_sw_codec_ops    = (void *)&cy_audio_sw_encode_sbc;
    /* Hardcoded here */
    audio_sw_hdl->header_schema             = CY_AUDIO_CODEC_NO_ALL_FRAME;
    *handle = audio_sw_hdl;

    cy_asc_log_dbg( "SBC encoder init success");
#ifdef VERIFY_INPUT_FRAME_MATCH
    FrameCount = 0;
#endif
    g_instance_active = true;
    return CY_RSLT_SUCCESS;

/* Do a clean return */
error_return:
    if(codec_obj)
    {
        cy_asc_destroy_sbc_encoder(codec_obj);
    }

    if(audio_sw_hdl)
    {
        free(audio_sw_hdl);
    }
    *handle = NULL;
    return result;

}

cy_rslt_t cy_audio_sw_sbc_encode(cy_audio_sw_codec_handle_t audio_sw_hdl,
      uint8_t *in_pcm_buff, uint32_t *in_pcm_data_len, uint8_t *out_buff,
      uint32_t *outbuf_len)
{
    sbc_encoder_handle_t           codec_obj           = NULL;
    cy_rslt_t                       result              = CY_RSLT_ASC_INVALID_HANDLE;
    uint32_t                        in_buff_pos         = 0;
    uint32_t                        out_buff_pos        = 0;
    // uint32_t                        header_len          = 0;
    uint32_t                        curr_enc_out_bytes  = 0;
    uint8_t                         num_frame_encode    = 0;
    uint32_t                        loop_index          = 0;
    uint32_t                        pcm_data_to_copy    = 0;
    uint32_t                        tot_pcm_data_len    = *in_pcm_data_len;
    uint32_t                        tot_outbuf_len      = *outbuf_len;
    uint8_t*                        buff_to_use         = NULL;

   codec_obj = (sbc_encoder_handle_t)audio_sw_hdl->audio_sw_algo_object;

    /* Check for sufficient output buffer provided */
    if(false == IS_SUFFICIENT_BUFF_LEN(*outbuf_len,codec_obj->frameInfo.output_size) )
    {
        cy_asc_log_err(CY_RSLT_ASC_INSUFFICIENT_OUTPUT_BUFF, "Insufficient output buffer sz:%lu Recommended:%lu"
                                    ,*outbuf_len, (codec_obj->frameInfo.output_size) );
        *in_pcm_data_len    = 0;
        *outbuf_len         = 0;
        return CY_RSLT_ASC_INSUFFICIENT_OUTPUT_BUFF;
    }

    /* Check for sufficient input buffer to process */
    if(false == cy_asc_check_sufficient_buffer_process(&codec_obj->input_buff,
                    in_pcm_buff,*in_pcm_data_len ) )
    {
        /* Cannot encoded less data, so the data is cached internally */
        *outbuf_len = 0;
        return CY_RSLT_SUCCESS;
    }

    num_frame_encode = (*in_pcm_data_len + codec_obj->input_buff.used_len)/codec_obj->frameInfo.input_size;
    *in_pcm_data_len = 0;
    *outbuf_len      = 0;

    /* Encode one frame by one. */
    for ( loop_index = 0;
         ((loop_index < num_frame_encode) &&\
         ((out_buff_pos + codec_obj->frameInfo.output_size)  <= tot_outbuf_len));
          loop_index++  )
    {
        pcm_data_to_copy = codec_obj->frameInfo.input_size - codec_obj->input_buff.used_len;

        if(0 != codec_obj->input_buff.used_len)
        {
            memcpy( (codec_obj->input_buff.buff +codec_obj->input_buff.used_len)
                    ,(in_pcm_buff + in_buff_pos),pcm_data_to_copy);
            buff_to_use = codec_obj->input_buff.buff;
        }
        else
        {
            buff_to_use = (uint8_t*)(in_pcm_buff + in_buff_pos);
        }

        codec_obj->input_buff.used_len = 0;
        /* Size of out buff */
        curr_enc_out_bytes = codec_obj->frameInfo.output_size;

        /* Perform encode */
        result =  cy_asc_sbc_encode_frame(codec_obj,buff_to_use, codec_obj->frameInfo.input_size,
                                                (out_buff+out_buff_pos)
                                                ,&curr_enc_out_bytes);
        if(CY_RSLT_SUCCESS != result)
        {
            cy_asc_log_err(result, "Opus encode operation failed.");
            goto error_return;
        }

        /* Update the input output bytes processed */
        out_buff_pos      += curr_enc_out_bytes;
        in_buff_pos       += pcm_data_to_copy;
        *in_pcm_data_len  += pcm_data_to_copy;
        *outbuf_len        = out_buff_pos;
    }

    cy_asc_check_and_cache_any_residual_data(&codec_obj->input_buff,in_pcm_buff,*in_pcm_data_len,tot_pcm_data_len);
    *in_pcm_data_len += codec_obj->input_buff.used_len;

    cy_asc_log_dbg("frames enc'd:%lu Input[pcm:%lu, out:%lu] Processed[pcm:%lu, out:%lu] cached:%lu"
                        , loop_index, *in_pcm_data_len, *outbuf_len, in_buff_pos, out_buff_pos
                        ,codec_obj->input_buff.used_len );
    return CY_RSLT_SUCCESS;

error_return:
    return result;

}

cy_rslt_t cy_audio_sw_sbc_get_encoder_recommended_inbuf_size(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        uint32_t *recommended_in_size)
{
    sbc_encoder_handle_t        codec_obj   = NULL;
    cy_rslt_t                   result      = CY_RSLT_ASC_INVALID_HANDLE;

    if(audio_sw_hdl && audio_sw_hdl->audio_sw_algo_object)
    {
        codec_obj = (sbc_encoder_handle_t)audio_sw_hdl->audio_sw_algo_object;
        *recommended_in_size = codec_obj->frameInfo.input_size;
        result = CY_RSLT_SUCCESS;
    }
    return result;
}

cy_rslt_t cy_audio_sw_sbc_get_encoder_recommended_outbuf_size(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        uint32_t in_size,
        uint32_t *recommended_out_size)
{
    sbc_encoder_handle_t        codec_obj   = NULL;
    cy_rslt_t                   result      = CY_RSLT_ASC_INVALID_HANDLE;

    *recommended_out_size = 0;

    if(audio_sw_hdl && audio_sw_hdl->audio_sw_algo_object)
    {

        codec_obj = (sbc_encoder_handle_t)audio_sw_hdl->audio_sw_algo_object;
        if(in_size >= codec_obj->frameInfo.input_size)
        {
            *recommended_out_size   = (in_size/codec_obj->frameInfo.input_size) * \
                                      codec_obj->frameInfo.output_size;
        }
        else
        {
            *recommended_out_size   = codec_obj->frameInfo.input_size;
        }

        result = CY_RSLT_SUCCESS;
    }

    return result;
}

cy_rslt_t cy_audio_sw_sbc_encoder_deinit(cy_audio_sw_codec_t *handle)
{
    cy_audio_sw_codec_handle_t  audio_sw_hdl    = NULL;
    cy_rslt_t                   result          = CY_RSLT_ASC_INVALID_HANDLE;

    audio_sw_hdl = (cy_audio_sw_codec_handle_t)(*handle);
    if(audio_sw_hdl)
    {
        cy_asc_log_dbg("SBC encoder deinit Req ASC hdl:%p", *handle);
        if(audio_sw_hdl->audio_sw_algo_object)
        {
            cy_asc_destroy_sbc_encoder((sbc_encoder_handle_t)audio_sw_hdl->audio_sw_algo_object);
            audio_sw_hdl->audio_sw_algo_object = NULL;
        }
        free(audio_sw_hdl);
        *handle = NULL;
        g_instance_active = false;
        result = CY_RSLT_SUCCESS;
        cy_asc_log_dbg("SBC encoder deinit success sw codec");
    }

    return result;
}
#endif /* ENABLE_SBC_ENCODE */