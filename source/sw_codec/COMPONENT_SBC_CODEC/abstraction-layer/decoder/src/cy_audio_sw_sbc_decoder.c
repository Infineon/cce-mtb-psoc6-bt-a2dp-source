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

/** @file cy_audio_sw_sbc_decoder.c
 *  Functions to perform SBC encoder operations
 *
 */

#ifdef ENABLE_SBC_DECODE
/*******************************************************************************
 *                      Includes
 ******************************************************************************/
#include <string.h>
#include "cyabs_rtos.h"
#include "cy_audio_sw_sbc_decoder.h"
#include "cy_audio_sw_codec_utils.h"
#include "cy_audio_sw_sbc_decoder_private.h"
#include "cy_audio_sw_sbc_utils.h"
#include "cy_audio_sw_codecs.h"


/*******************************************************************************
 *                      Macros
 ******************************************************************************/

#define SBC_A2DP_SYNC 0x9C
#define SBC_WIDEBAND_SYNC 0xAD
#define SBC_UUID_2_INDEX 0

#define SBC_UUID_2_INDEX_FRAME_LEN 57
/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      Constant
 ******************************************************************************/
const cy_audio_sw_codec_decoder_ops_t cy_audio_sw_decode_sbc = {
    .type = CY_AUDIO_CODEC_TYPE_SBC,
    .init = cy_audio_sw_sbc_decoder_init,
    .decode = cy_audio_sw_sbc_decode,
    .get_recommended_size = cy_audio_sw_sbc_get_decoder_recommended_outbuf_size,
    .decode_stream_info_header = cy_audio_sw_sbc_decode_stream_info_header,
    .deinit = cy_audio_sw_sbc_decoder_deinit
};

static cy_audio_sw_codec_stream_info_t sbc_wide_band_param[SBC_UUID_2_INDEX+1] = {
    {//0
    .sampling_rate = CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ,
    .channel = CY_AUDIO_CODEC_CHANNEL_MONO,
    .bitwidth = CY_AUDIO_CODEC_BITWIDTH_16,
    .codec_specific_info.sbc.allocation_method = SBC_LOUDNESS,
    .codec_specific_info.sbc.bit_pool = 26,
    .codec_specific_info.sbc.channel_mode = 0, //mono
    .codec_specific_info.sbc.sub_bands = 8,
    .codec_specific_info.sbc.sub_blocks = 15,
    .codec_specific_info.sbc.sbc_mode = SBC_MODE_WIDE_BAND,
    .codec_specific_info.sbc.uui_id = 2,
    }
};

/* Currently SBC algo supports only single instance */
static volatile bool g_instance_active = false;

#ifdef ASC_DEC_VERIFY_INPUT_FRAME_MATCH
    static uint32_t FrameCount = 0;
#endif

/*******************************************************************************
 *                      Function decelerations
 ******************************************************************************/
static bool is_valid_stream_info(const cy_audio_sw_codec_stream_info_t *stream_info);
static bool is_valid_sbc_stream_info(const cy_asc_sbc_decode_stream_info_t *sbc_stream_info);
static bool is_valid_sbc_decoder_config(const cy_audio_sw_codec_decode_config_t *codec_config);
static void cy_asc_free_sbc_dec_internal_hdl(sbc_decoder_handle_t* handle);
static cy_rslt_t cy_asc_initialize_algo( sbc_decoder_handle_t handle );
static cy_rslt_t cy_asc_create_sbc_dec_internal_hdl(
                        sbc_decoder_handle_t * a_handle,
                        const cy_audio_sw_codec_decode_config_t *codec_config);
static cy_rslt_t cy_sbc_parse_stream_info_on_decode(cy_audio_sw_codec_handle_t audio_sw_hdl,
                                                    uint8_t *in_buff, uint32_t *in_data_len );
static cy_rslt_t cy_asc_handle_decode_header_state (cy_audio_sw_codec_handle_t audio_sw_hdl, uint8_t *in_buff, uint32_t *in_buff_len);
static cy_rslt_t cy_sbc_handle_decode_frame_state(cy_audio_sw_codec_handle_t audio_sw_hdl,
                                                  uint8_t *in_buff, uint32_t *in_buff_len,
                                                  uint8_t *out_buff, uint32_t *out_bytes);
static cy_rslt_t cy_asc_parse_a2dp_frame(const uint8_t *pHeader, cy_audio_sw_codec_stream_info_t *stream_info, uint32_t *a_frame_len);
static void cy_sbc_update_and_send_callback_info(
        cy_audio_sw_codec_handle_t audio_sw_hdl, cy_audio_sw_codec_stream_info_t stream_info);
static cy_rslt_t cy_sbc_dec_parse_stream_info_from_header(cy_audio_sw_codec_handle_t audio_sw_hdl,
                                                          cy_audio_sw_codec_stream_info_t *stream_info,
                                                          uint8_t *in_buff, uint32_t *in_buff_len, bool is_internal_call);
static cy_rslt_t cy_sbc_dec_get_recommended_outbuf_size_internal(
                    sbc_decoder_handle_t codec_obj,
                    cy_audio_sw_codec_stream_info_t * stream_info,
                    uint32_t *recommended_out_size);

static cy_rslt_t cy_sbc_dec_do_decode(sbc_decoder_handle_t codec_obj, uint8_t *in_buff
                                    , uint8_t *out_buff, uint32_t *out_bytes);
static cy_rslt_t cy_audio_sw_sbc_decode_stream_info_header_internal(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        uint8_t *in_encoded_buff,
        uint32_t *in_encoded_data_len,
        cy_audio_sw_codec_stream_info_t *stream_info,bool is_internal_call);

/*******************************************************************************
 *                      Function definations
 ******************************************************************************/
static bool is_valid_stream_info(const cy_audio_sw_codec_stream_info_t *stream_info)
{
    bool is_valid_config = false;

    if(stream_info->bitwidth != CY_AUDIO_CODEC_BITWIDTH_16)
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Only 16 bit is supported in SBC algo");
        goto api_exit;
    }

    if( (stream_info->channel != CY_AUDIO_CODEC_CHANNEL_MONO) &&
        (stream_info->channel != CY_AUDIO_CODEC_CHANNEL_STEREO) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Invalid channel mode:%d", stream_info->channel);
        goto api_exit;
    }

    if( !(stream_info->sampling_rate >= CY_AUDIO_CODEC_SAMPLING_RATE_16000HZ) &&
        (stream_info->sampling_rate <= CY_AUDIO_CODEC_SAMPLING_RATE_48000HZ) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Invalid sample rate:%d", stream_info->sampling_rate);
        goto api_exit;
    }

    is_valid_config = is_valid_sbc_stream_info(&stream_info->codec_specific_info.sbc);

api_exit:
    return is_valid_config;
}

static bool is_valid_sbc_stream_info(const cy_asc_sbc_decode_stream_info_t *sbc_stream_info)
{
    bool is_valid_config = false;

    if( (sbc_stream_info->sbc_mode != SBC_MODE_A2DP) && (sbc_stream_info->sbc_mode != SBC_MODE_WIDE_BAND) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid SBC mode:%d",sbc_stream_info->sbc_mode);
        goto api_exit;
    }

    if(sbc_stream_info->sbc_mode == SBC_MODE_WIDE_BAND &&
         sbc_stream_info->uui_id == SBC_WB_SUPPORTED_UUID )
    {
        is_valid_config = true;
    }
    else if(sbc_stream_info->sbc_mode == SBC_MODE_A2DP )
    {
        if( (sbc_stream_info->allocation_method != SBC_LOUDNESS) && (sbc_stream_info->allocation_method != SBC_SNR) )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid SBC allocation method:%u",sbc_stream_info->allocation_method);
            goto api_exit;
        }

        if( !(sbc_stream_info->bit_pool >= SBC_DEC_BIT_POOL_MIN) && (sbc_stream_info->bit_pool <= SBC_DEC_BIT_POOL_MAX) )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid SBC bit pool:%u", sbc_stream_info->bit_pool);
            goto api_exit;
        }

        if( !(sbc_stream_info->channel_mode >= 0) && (sbc_stream_info->channel_mode <= 3) )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid SBC channel mode:%u",sbc_stream_info->channel_mode);
            goto api_exit;
        }

        if( (sbc_stream_info->sub_bands != 4) && (sbc_stream_info->sub_bands != 8) )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid SBC sub band:%u",sbc_stream_info->sub_bands);
            goto api_exit;
        }

        if( (sbc_stream_info->sub_blocks != 4) && (sbc_stream_info->sub_blocks != 8) &&\
            (sbc_stream_info->sub_blocks != 12) && (sbc_stream_info->sub_blocks != 16) )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid SBC sub block:%u",sbc_stream_info->sub_blocks);
            goto api_exit;
        }

        is_valid_config = true;
    }
    else if(sbc_stream_info->sbc_mode == SBC_MODE_WIDE_BAND &&
            sbc_stream_info->uui_id != SBC_WB_SUPPORTED_UUID )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "unknown UUID:%d with wide band mode, Supported uuid:%d"
                                , sbc_stream_info->uui_id
                                , SBC_WB_SUPPORTED_UUID );
        goto api_exit;
    }
    else
    {
        is_valid_config = false;
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid SBC A2DP stream info");
        goto api_exit;
    }

api_exit:
    return is_valid_config;
}

static bool is_valid_sbc_decoder_config(const cy_audio_sw_codec_decode_config_t *codec_config)
{
    bool is_valid_config = false;

    cy_asc_log_info("Decoder config pram provide Bit:%d, CH:%d, SR:%d sbc mode: %d, uui_id:%d"
                , codec_config->bitwidth
                , codec_config->channel
                , codec_config->sampling_rate
                , codec_config->codec_specific_config.sbc.sbc_mode
                , codec_config->codec_specific_config.sbc.uui_id);

    if( !((codec_config->bitwidth > CY_AUDIO_CODEC_BITWIDTH_INVALID) &&\
        (codec_config->bitwidth < CY_AUDIO_CODEC_BITWIDTH_MAX) &&\
        (codec_config->channel > CY_AUDIO_CODEC_CHANNEL_INVALID) &&\
        (codec_config->channel < CY_AUDIO_CODEC_CHANNEL_MAX ) &&\
        (codec_config->sampling_rate > CY_AUDIO_CODEC_SAMPLING_RATE_INVALID) &&\
        (codec_config->sampling_rate < CY_AUDIO_CODEC_SAMPLING_RATE_MAX) &&\
        (codec_config->codec_specific_config.sbc.sbc_mode >= SBC_MODE_A2DP) &&\
        (codec_config->codec_specific_config.sbc.sbc_mode <=  SBC_MODE_WIDE_BAND) )
          )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "Invalid config pram Bit: %d, CH: %d, SR: %d sbc mode: %d, uui_id:%d"
                                    , codec_config->bitwidth
                                    , codec_config->channel
                                    , codec_config->sampling_rate
                                    , codec_config->codec_specific_config.sbc.sbc_mode
                                    , codec_config->codec_specific_config.sbc.uui_id );
            goto api_exit;
        }

    if(codec_config->codec_specific_config.sbc.sbc_mode == SBC_MODE_WIDE_BAND &&
        codec_config->codec_specific_config.sbc.uui_id != SBC_WB_SUPPORTED_UUID)
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG, "unknown UUID:%d with wide band mode, Supported uuid:%d"
                                , codec_config->codec_specific_config.sbc.uui_id
                                , SBC_WB_SUPPORTED_UUID );
        goto api_exit;
    }

    is_valid_config = true;

api_exit:
    return is_valid_config;
}

static void cy_asc_free_sbc_dec_internal_hdl(sbc_decoder_handle_t* handle)
{
    if( NULL != handle && NULL != *handle)
    {
        if((*handle)->input_buff.buff)
        {
            free((*handle)->input_buff.buff);
        }

        if((*handle)->decoder)
        {
            free((*handle)->decoder);
        }
        free(*handle);
    }
    *handle = NULL;
}


static cy_rslt_t cy_asc_initialize_algo( sbc_decoder_handle_t handle )
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    uint16_t status = 0;

    handle->decoder = (SBC_DEC_PARAMS*)calloc(sizeof(SBC_DEC_PARAMS),1);
    if(NULL == handle->decoder)
    {
        result = CY_RSLT_ASC_OUT_OF_MEMORY;
        cy_asc_log_err(result, "out of memory SBC decoder algo hdl-Sz:%u",sizeof(SBC_DEC_PARAMS));
        goto api_exit;
    }

    handle->decoder->sbc_mode = handle->stream_info.codec_specific_info.sbc.sbc_mode;
    handle->decoder->uui_id = handle->stream_info.codec_specific_info.sbc.uui_id;

    status = SBC_Decoder_decode_Init(handle->decoder);
    if (status != SBC_SUCCESS)
    {
        result = CY_RSLT_ASC_ALGO_INIT_FAILED;
        cy_asc_log_err(result,"SBC algo Init Failed");
        goto api_exit;
    }

    handle->input_buff.buff = (uint8_t *)malloc(handle->frame_len);

    if(NULL == handle->input_buff.buff)
    {
        result = CY_RSLT_ASC_OUT_OF_MEMORY;
        cy_asc_log_err(result, "out of memory Input buffer - Sz:%lu",handle->frame_len);
        goto api_exit;
    }

    handle->input_buff.buff_len = handle->frame_len;
    handle->input_buff.used_len = 0;
    /* Moving to next state */
    handle->decode_state = ASC_SBC_DEC_PARSE_HEADER;

api_exit:
    return result;
}

static cy_rslt_t cy_asc_create_sbc_dec_internal_hdl(
                        sbc_decoder_handle_t * a_handle,
                        const cy_audio_sw_codec_decode_config_t *codec_config)
{
    sbc_decoder_handle_t handle = NULL;
    cy_rslt_t result = CY_RSLT_ASC_GENERIC_ERROR;

    if(NULL != codec_config)
    {
        if( false == is_valid_sbc_decoder_config(codec_config) )
        {
            cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Invalid config");
            return CY_RSLT_ASC_BAD_ARG;
        }
    }

    handle = (sbc_decoder_handle_t)malloc(sizeof(sbc_decoder_handle));
    if(NULL != handle)
    {
        memset(handle,0x00,sizeof(sbc_decoder_handle));
        handle->decode_state = ASC_SBC_DEC_STATE_ALGO_NOT_INITIALIZED;

        *a_handle = handle;
        result = CY_RSLT_SUCCESS;
    }

    if(CY_RSLT_SUCCESS != result )
    {
        if(NULL != handle)
        {
            cy_asc_free_sbc_dec_internal_hdl(&handle);
        }
    }

    return result;
}

static cy_rslt_t cy_sbc_parse_stream_info_on_decode(cy_audio_sw_codec_handle_t audio_sw_hdl,
                                                    uint8_t *in_buff, uint32_t *in_data_len )
{
    cy_rslt_t result;
    cy_audio_sw_codec_stream_info_t stream_info;

    /* stream is not provided by user or parsed internally, try parsing now */
    result = cy_audio_sw_sbc_decode_stream_info_header_internal(audio_sw_hdl, in_buff,
                                                                in_data_len, &stream_info,true);
    if((CY_RSLT_ASC_PARTIAL_HEADER_IDENTIFIED == result) || (CY_RSLT_SUCCESS != result) )
    {
        cy_asc_log_err(result,"%s",(CY_RSLT_ASC_PARTIAL_HEADER_IDENTIFIED == result)?\
                            "Partial header found, feed in more data" :\
                            "Stream info not provided & header is not found, cannot decode data.");
        return result;
    }

    cy_sbc_update_and_send_callback_info(audio_sw_hdl, stream_info);
    return result;
}

static cy_rslt_t cy_asc_handle_decode_header_state (cy_audio_sw_codec_handle_t audio_sw_hdl, uint8_t *in_buff, uint32_t *in_buff_len)
{
    sbc_decoder_handle_t codec_obj     = NULL;
    codec_obj = (sbc_decoder_handle_t)audio_sw_hdl->audio_sw_algo_object;
    uint32_t data_to_copy = 0;
    uint8_t *buff = NULL;
    uint32_t buff_len = 0;
    uint32_t tot_enc_data_len = *in_buff_len;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    //Accumulate buffer to parse header
    buff        = codec_obj->input_buff.buff;
    buff_len    = codec_obj->input_buff.used_len;

    if( codec_obj->input_buff.used_len > 0 &&  codec_obj->input_buff.used_len < SBC_ENC_HEADER_LEN )
    {
        buff        = codec_obj->input_buff.buff;
        buff_len    = codec_obj->input_buff.used_len;
        data_to_copy = GET_DATA_LEN_TO_COPY(SBC_ENC_HEADER_LEN,codec_obj->input_buff.used_len, tot_enc_data_len );
        memcpy( buff+buff_len, in_buff, data_to_copy);
        buff_len += data_to_copy;
        codec_obj->input_buff.used_len += data_to_copy;
    }
    else if ( codec_obj->input_buff.used_len > 0 &&  codec_obj->input_buff.used_len >=  SBC_ENC_HEADER_LEN )
    {
        buff        = codec_obj->input_buff.buff;
        buff_len    = codec_obj->input_buff.used_len;
    }
    else if(tot_enc_data_len >= SBC_ENC_HEADER_LEN)
    {
        buff        = in_buff;
        buff_len    = tot_enc_data_len;
    }
    else if(tot_enc_data_len < SBC_ENC_HEADER_LEN)
    {
        memcpy( codec_obj->input_buff.buff, in_buff, tot_enc_data_len);
        codec_obj->input_buff.used_len = tot_enc_data_len;
        *in_buff_len = tot_enc_data_len;
        return CY_RSLT_SUCCESS;
    }
    else
    {
        result = CY_RSLT_ASC_GENERIC_ERROR;
        cy_asc_log_err(result, "Error!!! Un handled scenario, tot_enc_data_len:%lu Input used:%lu"
                                                , tot_enc_data_len
                                                , codec_obj->input_buff.used_len);
        goto api_exit;
    }

    if(CY_RSLT_SUCCESS == result)
    {
        if( buff[0] != SBC_WIDEBAND_SYNC && buff[0] != SBC_A2DP_SYNC)
        {
            result = CY_RSLT_ASC_HEADER_PARSING_ERROR;
            cy_asc_log_err(result, "Error!!! Expected sync word, not found");
            goto api_exit;
        }

        result = cy_sbc_dec_parse_stream_info_from_header(audio_sw_hdl,&codec_obj->stream_info,buff,&buff_len,true);
        if(CY_RSLT_SUCCESS != result)
        {
            result = CY_RSLT_ASC_HEADER_PARSING_ERROR;
            cy_asc_log_err(result,"Frame header parsing failed");
            goto api_exit;
        }
        *in_buff_len = buff_len;
        /* Add if any data is copied */
        *in_buff_len += data_to_copy;
    }

api_exit:
    if(result == CY_RSLT_SUCCESS)
    {
        codec_obj->decode_state = ASC_SBC_DEC_DECODE_FRAME;
    }

    return result;
}

static cy_rslt_t cy_sbc_dec_do_decode(sbc_decoder_handle_t codec_obj, uint8_t *in_buff
                                    , uint8_t *out_buff, uint32_t *out_bytes)
{
    cy_rslt_t result = CY_RSLT_ASC_GENERIC_ERROR;
    int16_t bytes_decoded = 0;

#ifdef ASC_DEC_VERIFY_INPUT_FRAME_MATCH
    bool is_same = false;
    /* Assumption  - API is called for each frame serially */
    extern bool verify_dec_Inputdata(uint8_t * Data_to_encode, uint32_t Size, uint32_t a_FrameCount);
    is_same = verify_dec_Inputdata(in_buff, codec_obj->frame_len, FrameCount);
    FrameCount++;
    CY_ASSERT (is_same == true);
#endif
    *out_bytes = 0;

    bytes_decoded = SBC_Decoder_decoder(codec_obj->decoder, in_buff, codec_obj->frame_len, (SINT16*)out_buff);
    if(bytes_decoded <= 0)
    {
        result = CY_RSLT_ASC_ALGO_INTERNAL_ERROR;
        cy_asc_log_err(result,"SBC Decode error!");
    }

    *out_bytes = bytes_decoded * 2; //Samples to bytes
    result = CY_RSLT_SUCCESS;

    return result;
}

static cy_rslt_t cy_sbc_handle_decode_frame_state(cy_audio_sw_codec_handle_t audio_sw_hdl,
                                                  uint8_t *in_buff, uint32_t *in_buff_len,
                                                  uint8_t *out_buff, uint32_t *out_bytes)
{
    sbc_decoder_handle_t codec_obj = audio_sw_hdl->audio_sw_algo_object;
    cy_rslt_t result     = CY_RSLT_ASC_GENERIC_ERROR;
    uint32_t rem_len     = 0;
    uint8_t *buff_to_use = NULL;

    *out_bytes = 0;

    if(0 == codec_obj->frame_len)
    {
        cy_asc_log_err(result,"at this stage frame size cannot be unknown");
        *in_buff_len = 0;
        return result;
    }

    if( codec_obj->input_buff.used_len < codec_obj->frame_len )
    {
        rem_len = codec_obj->frame_len - codec_obj->input_buff.used_len;

        if(*in_buff_len < rem_len )
        {
            /* We have less data to decode, cache it and wait for more data */
            rem_len = *in_buff_len;
            memcpy(codec_obj->input_buff.buff+ codec_obj->input_buff.used_len, in_buff, rem_len);
            codec_obj->input_buff.used_len += rem_len;
            result = CY_RSLT_SUCCESS;
            goto api_exit;
        }
        else
        {
            if(codec_obj->input_buff.used_len > 0)
            {
               memcpy(codec_obj->input_buff.buff+ codec_obj->input_buff.used_len, in_buff, rem_len);
               buff_to_use = codec_obj->input_buff.buff;
            }
            else
            {
                buff_to_use = in_buff;
            }

            result = cy_sbc_dec_do_decode(codec_obj,buff_to_use,out_buff,out_bytes);
            if(result == CY_RSLT_SUCCESS)
            {
                *in_buff_len = rem_len;
                codec_obj->input_buff.used_len = 0;
                codec_obj->decode_state = ASC_SBC_DEC_PARSE_HEADER;
            }
        }
    }
    else
    {
        cy_asc_log_err(result,"SBC MW in Bad state, unknown error");
        *in_buff_len = 0;
        goto api_exit;
    }

api_exit:
    return result;
}

static cy_rslt_t cy_asc_parse_a2dp_frame(const uint8_t *pHeader, cy_audio_sw_codec_stream_info_t *stream_info, uint32_t *a_frame_len)
{
    const uint8_t *p_frame, *p_frame1, *p_frame2;
    uint8_t  blocks, channel_mode, sub_bands, bit_pool, channels, join, sampling_freq, allocation_method;
    uint32_t  frame_len, tmp;

    p_frame = pHeader;
    frame_len = 0;

    if (*p_frame != SBC_A2DP_SYNC)
    {
        cy_asc_log_err (CY_RSLT_ASC_HEADER_PARSING_ERROR,"Bad Sync Byte: 0x%02x !!!", *p_frame);
        return CY_RSLT_ASC_HEADER_PARSING_ERROR;
    }

    p_frame1 = p_frame + 1;
    p_frame2 = p_frame1 + 1;

    sampling_freq = (*(unsigned char *)p_frame1 & 0xC0) >> 6;
    blocks = (*(unsigned char *)p_frame1 & 0x30) >> 4;

    switch (blocks)
    {
        case 0x0:blocks = 4; break;
        case 0x1:blocks = 8; break;
        case 0x2:blocks = 12; break;
        case 0x3:blocks = 16; break;
        default: break;
    }
    channel_mode    = (*(unsigned char *)p_frame1 & 0x0C) >> 2;
    channels = (channel_mode == 0) ? 1 : 2;

    allocation_method = (*(unsigned char *)p_frame1 & 0x02) >> 1;
    sub_bands   = (*(unsigned char *)p_frame1 & 0x01);
    sub_bands = (sub_bands == 0) ? 4 : 8;

    bit_pool    = (*(unsigned char *)p_frame2 & 0xff);

    frame_len = 4 + (4 * sub_bands * channels) / 8;

    join = (channel_mode == 3) ? 1 : 0;

    if (channel_mode < 2)
        tmp =  (blocks * channels * bit_pool);
    else
        tmp =  (join * sub_bands + blocks * bit_pool);

    frame_len += tmp / 8 + ((tmp % 8) ? 1 : 0);

    *a_frame_len = frame_len;
    stream_info->codec_specific_info.sbc.allocation_method = allocation_method;
    stream_info->codec_specific_info.sbc.bit_pool = bit_pool;
    stream_info->codec_specific_info.sbc.channel_mode = channel_mode;
    stream_info->codec_specific_info.sbc.sub_bands = sub_bands;
    stream_info->codec_specific_info.sbc.sub_blocks = blocks;

    if(channels == 2)
    {
        stream_info->channel = CY_AUDIO_CODEC_CHANNEL_STEREO;
    }
    else
    {
        stream_info->channel = CY_AUDIO_CODEC_CHANNEL_MONO;
    }

    stream_info->bitwidth = CY_AUDIO_CODEC_BITWIDTH_16;
    stream_info->sampling_rate = cy_asc_sbc_sample_rate_value_to_enum(sampling_freq);
    stream_info->codec_specific_info.sbc.sbc_mode = SBC_MODE_A2DP;

    return CY_RSLT_SUCCESS;
}

static bool is_equal_sbc_stream_info(cy_asc_sbc_decode_stream_info_t * info1, cy_asc_sbc_decode_stream_info_t * info2)
{
    bool value = true;

    if (info1->sub_bands            != info2->sub_bands ||
        info1->sub_blocks           != info2->sub_blocks ||
        info1->allocation_method    != info2->allocation_method ||
        info1->channel_mode         != info2->channel_mode ||
        info1->bit_pool             != info2->bit_pool ||
        info1->sbc_mode             != info2->sbc_mode ||
        info1->uui_id               != info2->uui_id )
    {
        value = false;
    }

    return value;
}

static void cy_sbc_update_and_send_callback_info(
        cy_audio_sw_codec_handle_t audio_sw_hdl, cy_audio_sw_codec_stream_info_t stream_info)
{
    sbc_decoder_handle_t codec_obj = audio_sw_hdl->audio_sw_algo_object;
    cy_rslt_t       result              = CY_RSLT_ASC_INVALID_HANDLE;
    bool            stream_info_changed = false;

    if (codec_obj->stream_info.bitwidth != stream_info.bitwidth ||
        codec_obj->stream_info.channel != stream_info.channel ||
        codec_obj->stream_info.sampling_rate != stream_info.sampling_rate ||
        (false == is_equal_sbc_stream_info(&codec_obj->stream_info.codec_specific_info.sbc,&stream_info.codec_specific_info.sbc)) )
    {
        codec_obj->stream_info         = stream_info;
        codec_obj->stream_info_state   = ASC_SBC_DEC_STRM_INFO_AVAILABLE;
        stream_info_changed                 = true;
        cy_asc_log_info("Stream info change: From{Bit:%d,CH:%d,SR:%d} To{Bit:%d,CH:%d,SR:%d}"
                        ,codec_obj->stream_info.bitwidth, codec_obj->stream_info.channel, codec_obj->stream_info.sampling_rate
                        ,stream_info.bitwidth, stream_info.channel, stream_info.sampling_rate );
    }

    /* Check if the recommended output buffer is detected */
    if(0 == codec_obj->recommended_out_size || true == stream_info_changed)
    {
        result = cy_audio_sw_sbc_get_decoder_recommended_outbuf_size(audio_sw_hdl,
                                                &codec_obj->stream_info,
                                                (uint32_t *)&codec_obj->recommended_out_size);
        if(CY_RSLT_SUCCESS != result)
        {
            cy_asc_log_err(result, "Failed to get recommended out buff sz");
            return;
        }
    }

    if( (NULL != codec_obj->dec_config.stream_info) && (true == stream_info_changed) )
    {
        codec_obj->dec_config.stream_info((cy_audio_sw_codec_t)audio_sw_hdl,
                        stream_info,codec_obj->dec_config.callback_user_arg);
    }

    return;
}

static cy_rslt_t cy_sbc_dec_parse_stream_info_from_header(cy_audio_sw_codec_handle_t audio_sw_hdl,
                                                          cy_audio_sw_codec_stream_info_t *stream_info,
                                                          uint8_t *in_buff, uint32_t *in_buff_len, bool is_internal_call)
{
    uint32_t loopIdx = 0;
    cy_rslt_t result = CY_RSLT_ASC_INVALID_HANDLE;
    sbc_decoder_handle_t codec_obj = NULL;
    uint32_t buff_len = *in_buff_len;

    if(audio_sw_hdl && audio_sw_hdl->audio_sw_algo_object)
    {
        codec_obj = (sbc_decoder_handle_t)audio_sw_hdl->audio_sw_algo_object;

        while(loopIdx < buff_len )
        {
            if( (in_buff[loopIdx] == SBC_A2DP_SYNC) )
            {
                if(loopIdx + SBC_ENC_HEADER_LEN <= buff_len)
                {
                    result = cy_asc_parse_a2dp_frame((in_buff + loopIdx), stream_info, &codec_obj->frame_len);
                    if(result == CY_RSLT_SUCCESS)
                    {
                        /* Start pos of the header is returned */
                        *in_buff_len = loopIdx;
                    }
                }
                else
                {
                    /* Start pos of the header is returned */
                    *in_buff_len = loopIdx;
                    result = CY_RSLT_ASC_PARTIAL_HEADER_IDENTIFIED;
                    goto api_exit;
                }
                break;
            }
            else if((in_buff[loopIdx] == SBC_WIDEBAND_SYNC) )
            {
                /* Start pos of the header is returned */
                *in_buff_len = loopIdx;
                if(loopIdx + SBC_ENC_HEADER_LEN <= buff_len)
                {
                    *stream_info = sbc_wide_band_param[SBC_UUID_2_INDEX];
                    codec_obj->frame_len = SBC_UUID_2_INDEX_FRAME_LEN;
                    result = CY_RSLT_SUCCESS;
                }
                else
                {
                    result = CY_RSLT_ASC_PARTIAL_HEADER_IDENTIFIED;
                    goto api_exit;
                }
                break;
            }

            loopIdx++;
        }

        if(result != CY_RSLT_SUCCESS)
        {
            cy_asc_log_info("No header found in the encoded data");
            result = CY_RSLT_ASC_HEADER_NOT_FOUND;
        }
        else
        {
            if(ASC_SBC_DEC_STRM_INFO_NOT_AVAILABLE == codec_obj->stream_info_state)
            {
                if( false == is_valid_sbc_stream_info(&stream_info->codec_specific_info.sbc) )
                {
                    *in_buff_len = 0;
                    cy_asc_log_err(CY_RSLT_ASC_NOT_SUPPORTED,"unknow/unsupported stream ");
                    return CY_RSLT_ASC_NOT_SUPPORTED;
                }

                if(true == is_internal_call)
                {
                    /* Called from decode api, Check if callback needed to be provided */
                    cy_sbc_update_and_send_callback_info(audio_sw_hdl, *stream_info);
                }

                codec_obj->stream_info         = *stream_info;
                codec_obj->stream_info_state   = ASC_SBC_DEC_STRM_INFO_AVAILABLE;
            }
            else
            {
                cy_sbc_update_and_send_callback_info(audio_sw_hdl, *stream_info);
            }
        }
    }
api_exit:
    return result;
}

cy_rslt_t cy_audio_sw_sbc_decoder_init(
            cy_audio_sw_codec_decode_config_t *codec_config,
            cy_audio_sw_codec_t *handle)
{
    cy_audio_sw_codec_handle_t audio_sw_hdl = NULL;
    sbc_decoder_handle_t codec_obj  = NULL;
    cy_rslt_t result = CY_RSLT_SUCCESS;

    if(true == g_instance_active)
    {
        result  = CY_RSLT_ASC_INVALID_MAX_INSTANCE_REACHED;
        cy_asc_log_err(result, "Already a SBC decoder instance is active");
        return result;
    }

    /* Allocate memory for the handle */
    audio_sw_hdl = (cy_audio_sw_codec_handle_t)malloc(sizeof(cy_audio_sw_codec_handle));
    if(NULL == audio_sw_hdl)
    {
        result = CY_RSLT_ASC_OUT_OF_MEMORY;
        cy_asc_log_err(result, "out of memory sw codec hdl - Sz:%u",sizeof(cy_audio_sw_codec_handle));
        goto error_return;
    }
    memset(audio_sw_hdl,0x00,sizeof(cy_audio_sw_codec_handle));

    /* Initialize API structure and set config params to default */
    result = cy_asc_create_sbc_dec_internal_hdl(&codec_obj, codec_config);
    if(CY_RSLT_SUCCESS != result)
    {
        cy_asc_log_err(result, "Failed to create SBC internal SBC hdl.");
        goto error_return;
    }

    codec_obj->dec_config = *codec_config;
    audio_sw_hdl->audio_sw_algo_object = (void *)codec_obj;
    audio_sw_hdl->pst_audio_sw_codec_ops = (void *)&cy_audio_sw_decode_sbc;
    /* Hardcoded here */
    audio_sw_hdl->header_schema = CY_AUDIO_CODEC_NO_HEADER;
    *handle = audio_sw_hdl;

    cy_asc_log_dbg( "SBC decoder init success Hdl:%p", audio_sw_hdl);

#ifdef ASC_DEC_VERIFY_INPUT_FRAME_MATCH
    FrameCount = 0;
#endif /* ASC_DEC_VERIFY_INPUT_FRAME_MATCH */
    g_instance_active = true;
    return CY_RSLT_SUCCESS;

/* Do a clean return */
error_return:
    if(codec_obj)
    {
        cy_asc_free_sbc_dec_internal_hdl(&codec_obj);
    }

    if(audio_sw_hdl)
    {
        free(audio_sw_hdl);
    }
    *handle = NULL;
    return result;
}

cy_rslt_t cy_audio_sw_sbc_decode(
            cy_audio_sw_codec_handle_t audio_sw_hdl,
            uint8_t *in_encoded_buff,
            uint32_t *in_encoded_data_len,
            uint8_t *out_buff,
            uint32_t *outbuf_len)
{
    sbc_decoder_handle_t codec_obj     = NULL;
    cy_rslt_t                       result              = CY_RSLT_ASC_INVALID_HANDLE;
    uint32_t                        tot_outbuf_len      = *outbuf_len;
    uint32_t                        tot_inbuf_len       = *in_encoded_data_len;
    uint32_t                        in_buff_pos         = 0;
    uint32_t                        out_buff_pos        = 0;

    codec_obj = (sbc_decoder_handle_t)audio_sw_hdl->audio_sw_algo_object;

    if( ASC_SBC_DEC_STRM_INFO_AVAILABLE != codec_obj->stream_info_state )
    {
        result = cy_sbc_parse_stream_info_on_decode(audio_sw_hdl,in_encoded_buff,in_encoded_data_len);
        in_buff_pos = *in_encoded_data_len;
        if(CY_RSLT_SUCCESS != result )
        {
            // cy_asc_log_err(result,"Failed to get stream info"); - Log added in Deeper API
            return result;
        }
    }

    if(*outbuf_len < codec_obj->recommended_out_size)
    {
        cy_asc_log_err(CY_RSLT_ASC_INSUFFICIENT_OUTPUT_BUFF, "Insufficient output buff sz:%lu Recommended:%lu"
                                    ,*outbuf_len, codec_obj->recommended_out_size );
        *in_encoded_data_len  = in_buff_pos;
        *outbuf_len           = 0;
        return CY_RSLT_ASC_INSUFFICIENT_OUTPUT_BUFF;
    }

    result = CY_RSLT_ASC_GENERIC_ERROR;

    while( ( in_buff_pos < tot_inbuf_len)/* &&\
           ((tot_outbuf_len - out_buff_pos) >= codec_obj->config_param.output_size)*/ )
    {
        if(ASC_SBC_DEC_STATE_ALGO_NOT_INITIALIZED == codec_obj->decode_state)
        {
            result = cy_asc_initialize_algo(codec_obj);
            if ( CY_RSLT_SUCCESS != result)
            {
                cy_asc_log_err(result,"error rd next frame");
                break;
            }
        }

        if( ASC_SBC_DEC_PARSE_HEADER == codec_obj->decode_state)
        {
            uint32_t rem_len = tot_inbuf_len - in_buff_pos;
            result = cy_asc_handle_decode_header_state (audio_sw_hdl, (in_encoded_buff+in_buff_pos), &rem_len);
            if ( CY_RSLT_SUCCESS != result)
            {
                cy_asc_log_err(result,"error checking frame header state");
                break;
            }
            in_buff_pos += rem_len;
        }

        if(ASC_SBC_DEC_DECODE_FRAME == codec_obj->decode_state)
        {
            uint32_t rem_len = (tot_inbuf_len - in_buff_pos);
            uint32_t output_sz = 0;
            /* Check if we have enough outbuff space  */
            if( (tot_outbuf_len - out_buff_pos) < codec_obj->recommended_out_size)
            {
                /* Not enough space to decode a frame to output buff */
                if((codec_obj->input_buff.used_len+rem_len) < codec_obj->frame_len )
                {
                    memcpy(codec_obj->input_buff.buff + codec_obj->input_buff.used_len,
                            in_encoded_buff+in_buff_pos, rem_len);
                    in_buff_pos += rem_len;
                    codec_obj->input_buff.used_len += rem_len;
                }
                else
                {
                    /* More than one frame data to cache, do not cache */
                }
                break;
            }

            result = cy_sbc_handle_decode_frame_state(audio_sw_hdl
                                        , in_encoded_buff+in_buff_pos ,&rem_len
                                        , out_buff+out_buff_pos, &output_sz);
            if(result != CY_RSLT_SUCCESS)
            {
                cy_asc_log_err(result,"Failed at decode frame state");
                break;
            }
            out_buff_pos +=output_sz;
            in_buff_pos +=rem_len;
        }
    }
    *in_encoded_data_len = in_buff_pos;
    *outbuf_len = out_buff_pos;

    cy_asc_log_dbg("Input prvd[in:%lu, out:%lu] Procd[in:%lu, out:%lu] cached:%lu State:%d"
                        , tot_inbuf_len, tot_outbuf_len, in_buff_pos, out_buff_pos
                        , codec_obj->input_buff.used_len
                        , codec_obj->decode_state );
    return result;
}

cy_rslt_t cy_audio_sw_sbc_get_decoder_recommended_outbuf_size(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        cy_audio_sw_codec_stream_info_t * stream_info,
        uint32_t *recommended_out_size)
{
    cy_rslt_t result = CY_RSLT_ASC_INVALID_HANDLE;

    if(audio_sw_hdl && audio_sw_hdl->audio_sw_algo_object)
    {
        result = cy_sbc_dec_get_recommended_outbuf_size_internal(
            (sbc_decoder_handle_t)audio_sw_hdl->audio_sw_algo_object, stream_info, recommended_out_size);
    }
    else
    {
        cy_asc_log_err(result,"Invalid handle obj");
    }

    return result;
}

cy_rslt_t cy_audio_sw_sbc_decode_stream_info_header(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        uint8_t *in_encoded_buff,
        uint32_t *in_encoded_data_len,
        cy_audio_sw_codec_stream_info_t *stream_info)
{
    return cy_audio_sw_sbc_decode_stream_info_header_internal(
            audio_sw_hdl,in_encoded_buff,in_encoded_data_len,stream_info,false);
}

static cy_rslt_t cy_audio_sw_sbc_decode_stream_info_header_internal(
        cy_audio_sw_codec_handle_t audio_sw_hdl,
        uint8_t *in_encoded_buff,
        uint32_t *in_encoded_data_len,
        cy_audio_sw_codec_stream_info_t *stream_info,bool is_internal_call)
{
    sbc_decoder_handle_t codec_obj = NULL;
    cy_rslt_t result = CY_RSLT_ASC_INVALID_HANDLE;
    uint32_t in_buff_len = *in_encoded_data_len;
    uint8_t  *in_buff = NULL;

    if(NULL == stream_info)
    {
        cy_asc_log_err(result,"Invalid stream info");
        return result;
    }

    if(*in_encoded_data_len < SBC_ENC_HEADER_LEN)
    {
         cy_asc_log_err(CY_RSLT_ASC_INSUFFICIENT_INPUT_BUFF,"SBC need min 4B to decode header");
         return CY_RSLT_ASC_INSUFFICIENT_INPUT_BUFF;
    }

    if(audio_sw_hdl && audio_sw_hdl->audio_sw_algo_object)
    {
        codec_obj = (sbc_decoder_handle_t)audio_sw_hdl->audio_sw_algo_object;

        /* Check if any partial header is already backed up */
        if( codec_obj->input_buff.used_len > 0 )
        {
            uint32_t DataToCpy = GET_DATA_LEN_TO_COPY(codec_obj->input_buff.buff_len,codec_obj->input_buff.used_len,*in_encoded_data_len);
            memcpy( codec_obj->input_buff.buff + codec_obj->input_buff.used_len,
                    in_encoded_buff, DataToCpy);
            in_buff     = codec_obj->input_buff.buff;
            in_buff_len = DataToCpy + codec_obj->input_buff.used_len;
        }
        else
        {
            in_buff     = in_encoded_buff;
            in_buff_len = *in_encoded_data_len;
        }

        result = cy_sbc_dec_parse_stream_info_from_header(audio_sw_hdl,stream_info,in_buff,&in_buff_len,is_internal_call);
        if(CY_RSLT_SUCCESS == result)
        {
            if( codec_obj->input_buff.used_len > 0 )
            {
                *in_encoded_data_len = in_buff_len - codec_obj->input_buff.used_len;
                *in_encoded_data_len = 0;
            }
            else
            {
                *in_encoded_data_len = in_buff_len;
            }

            codec_obj->input_buff.used_len = 0;
        }
        else if(CY_RSLT_ASC_PARTIAL_HEADER_IDENTIFIED == result)
        {
            /* Buffer start is identified, but header is incomplete.
             * Until header start all data can be skipped, so skipped length is given as consumed buffers
             */
            *in_encoded_data_len = in_buff_len;
        }
        else
        {
            /* No header found */
            *in_encoded_data_len = 0;
            codec_obj->input_buff.used_len = 0;
        }
    }
    else
    {
        cy_asc_log_err(result,"Invalid handle obj");
    }

    return result;
}

cy_rslt_t cy_audio_sw_sbc_decoder_deinit(
            cy_audio_sw_codec_t *handle)
{
    cy_audio_sw_codec_handle_t  audio_sw_hdl    = NULL;
    cy_rslt_t                   result          = CY_RSLT_ASC_INVALID_HANDLE;

    audio_sw_hdl = (cy_audio_sw_codec_handle_t)(*handle);
    if(audio_sw_hdl)
    {
        cy_asc_log_dbg("SBC decoder deinit Req ASC hdl:%p", *handle);
        if(audio_sw_hdl->audio_sw_algo_object)
        {
            if(((sbc_decoder_handle_t)audio_sw_hdl->audio_sw_algo_object)->decoder)
            {
                SBC_Decoder_decode_DeInit(((sbc_decoder_handle_t)audio_sw_hdl->audio_sw_algo_object)->decoder);
            }
            cy_asc_free_sbc_dec_internal_hdl((sbc_decoder_handle_t*)&audio_sw_hdl->audio_sw_algo_object);
            audio_sw_hdl->audio_sw_algo_object = NULL;
        }
        free(audio_sw_hdl);
        g_instance_active = false;
        result = CY_RSLT_SUCCESS;
        cy_asc_log_dbg("SBC decoder deinit success sw codec");
    }
    else
    {
        cy_asc_log_err(result,"Invalid codec handle obj");
    }

    *handle = NULL;
    return result;
}

static cy_rslt_t cy_sbc_dec_get_recommended_outbuf_size_internal(
                    sbc_decoder_handle_t codec_obj,
                    cy_audio_sw_codec_stream_info_t * stream_info,
                    uint32_t *recommended_out_size)
{
    uint16_t channels       = cy_audio_sw_codec_get_channel_num(stream_info->channel);
    // uint32_t sampling_rate  = cy_audio_sw_codec_get_sampling_rate(stream_info->sampling_rate);
    // uint8_t bitwidth        = cy_audio_sw_codec_get_bitwidth(stream_info->bitwidth);

    cy_asc_log_info("Stream info bit:%d SR:%d CH:%d"
                                            ,stream_info->bitwidth
                                            ,stream_info->sampling_rate
                                            ,stream_info->channel);

    cy_asc_log_info("Decoder SBC specific config pram provide uui_id:%u AM:%u"
                         "bit_pool:%u CH mode:%u sbc mode:%u band:%u blocks:%u"
                , stream_info->codec_specific_info.sbc.uui_id
                , stream_info->codec_specific_info.sbc.allocation_method
                , stream_info->codec_specific_info.sbc.bit_pool
                , stream_info->codec_specific_info.sbc.channel_mode
                , stream_info->codec_specific_info.sbc.sbc_mode
                , stream_info->codec_specific_info.sbc.sub_bands
                , stream_info->codec_specific_info.sbc.sub_blocks );

    if( false == is_valid_stream_info(stream_info) )
    {
        cy_asc_log_err(CY_RSLT_ASC_BAD_ARG,"Invalid stream info passed");
        return CY_RSLT_ASC_BAD_ARG;
    }

    *recommended_out_size   = 2 * (stream_info->codec_specific_info.sbc.sub_blocks* stream_info->codec_specific_info.sbc.sub_bands * channels);
    codec_obj->recommended_out_size = *recommended_out_size;

    return CY_RSLT_SUCCESS;
}


#endif /* ENABLE_SBC_DECODE */