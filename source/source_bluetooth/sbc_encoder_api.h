/******************************************************************************
* File Name:   sbc_encoder_api.h
*
* Description: Header file for SBC encoder.
*
*******************************************************************************
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
*******************************************************************************/

#ifndef SBC_ENCODER_API_H_
#define SBC_ENCODER_API_H_
#include "cy_audio_sw_codecs.h"
#include "cy_audio_sw_codec_utils.h"

/*******************************************************************************
* Structures
*******************************************************************************/

typedef struct sbc_encoder_params
{
    uint32_t sample_rate;
    uint8_t  channels;
    uint8_t  bit_width;
    cy_asc_sbc_encode_config_t  sbc_config;

} sbc_encoder_params_t;

/*******************************************************************************
* Function prototypes
*******************************************************************************/

void get_sbc_config( cy_asc_sbc_encode_config_t *sbc_config, bool wide_band, uint8_t quality, uint32_t n_channels, uint32_t sample_rate );
int sbc_encoder_app_init(cy_audio_sw_codec_t *enc_handle, sbc_encoder_params_t *sbc_encoder_params);
int sbc_encoder_app_encode(cy_audio_sw_codec_t enc_handle, uint8_t* input_buff, uint32_t* in_size, uint8_t * output_buff, uint32_t* out_size);
void sbc_encoder_app_deinit();

#endif /* SBC_ENCODER_API_H_ */

/* [] END OF FILE */
