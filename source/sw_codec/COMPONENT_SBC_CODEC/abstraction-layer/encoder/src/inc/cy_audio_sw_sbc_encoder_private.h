/**
 * @file cy_audio_sw_sbc_encoder_private.h
 *
 * @brief This file is the header file for audio software codec library.
 * This library support encoding and decoding operation over various codecs
 */
#ifndef __CY_AUDIO_SW_SBC_ENCODER_PRIVATE_H__
#define __CY_AUDIO_SW_SBC_ENCODER_PRIVATE_H__
#ifdef __cplusplus
extern "C"
{
#endif
/*******************************************************************************
 *                      Includes
 ******************************************************************************/
/* Codec specific includes */
#include "sbc_encoder.h"

/*******************************************************************************
 *                      Macros
 ******************************************************************************/

/*******************************************************************************
 *                      Constants
 ******************************************************************************/

/*******************************************************************************
 *                      Enumerations
 ******************************************************************************/

/*******************************************************************************
 *                      struct
 ******************************************************************************/
typedef struct sbc_encoder_frame_info
{
    /* Input PCM frame sz */
    uint32_t input_size;

    /* Output encoded frame sz */
    uint32_t output_size;

}sbc_encoder_frame_info;

typedef struct sbc_encoder_handle_
{

    /* SBC encoder handle */
    SBC_ENC_PARAMS *encoder;

    /* Output Buffer */
    cy_audio_sw_buff_t  input_buff;

    /* Audio Frame per samples */
    sbc_encoder_frame_info frameInfo;

    /* Encoder config */
    cy_audio_sw_codec_encode_config_t enc_config;

}sbc_encoder_handle;

/*******************************************************************************
 *                      typedef
 ******************************************************************************/
typedef sbc_encoder_handle * sbc_encoder_handle_t;

#ifdef __cplusplus
}
#endif

#endif /* __CY_AUDIO_SW_SBC_ENCODER_PRIVATE_H__ */