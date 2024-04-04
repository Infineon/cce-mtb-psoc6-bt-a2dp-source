/******************************************************************************
* File Name:   a2dp_source.h
*
* Description: Header file for A2DP source task.
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
#ifndef A2DP_SOURCE_H
#define A2DP_SOURCE_H

#include "app_bt_utils/app_bt_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <ctype.h>
#include <string.h>
#include <sys/time.h>
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_types.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_source.h"
#include "wiced_bt_a2dp_defs.h"
#include "hcidefs.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "wiced_timer.h"
#include "sbc_encoder_api.h"
#include "asc_wav_header_parser.h"
#include "cy_em_eeprom.h"

typedef short SINT16;
typedef int SINT32;

/******************************************************************************
 * Macros
 *****************************************************************************/

#define A2DP_SOURCE_SDP_DB_SIZE         63

#define AUDIO_SF_48K                    1
#define AUDIO_SF_44_1K                  2
#define AUDIO_SF_32K                    3
#define AUDIO_SF_16K                    4

#define AUDIO_CHCFG_MONO                0
#define AUDIO_CHCFG_STEREO              1

#define A2DP_APP_SUCCESS                0
#define A2DP_APP_FAILED                 1

#define WICED_PIN_CODE_LEN              (4U)

#define KEY_INFO_POOL_BUFFER_SIZE       (145U)

#define KEY_INFO_POOL_BUFFER_COUNT      (2U)

#define SBC_MAX_NUM_OF_SUBBANDS         8
#define SBC_MAX_NUM_OF_CHANNELS         2
#define SBC_MAX_NUM_OF_BLOCKS           16

#define SBC_LOUDNESS                    0
#define SBC_SNR                         1

#define SUB_BANDS_8                     8
#define SUB_BANDS_4                     4

#define SBC_sf16000                     0
#define SBC_sf32000                     1
#define SBC_sf44100                     2
#define SBC_sf48000                     3

#define SBC_MONO                        0
#define SBC_DUAL                        1
#define SBC_STEREO                      2
#define SBC_JOINT_STEREO                3

#define SBC_BLOCK_0                     4
#define SBC_BLOCK_1                     8
#define SBC_BLOCK_2                     12
#define SBC_BLOCK_3                     16

#define A2DP_PEER_CONNECTED             1
#define A2DP_PEER_DISCONNECTED          0
#define A2DP_PEER_MAINTAIN_STATUS       2

/* EEPROM macros */
/* EEPROM is used to store the paired peer BD address  */

/* Logical Size of Emulated EEPROM in bytes. */
#define LOGICAL_EEPROM_SIZE     (6u)
#define LOGICAL_EEPROM_START    (0u)

/* EEPROM Configuration details. All the sizes mentioned are in bytes.
 * For details on how to configure these values refer to cy_em_eeprom.h. The
 * library documentation is provided in Em EEPROM API Reference Manual. The user
 * access it from ModusToolbox IDE Quick Panel > Documentation> 
 * Cypress Em_EEPROM middleware API reference manual
 */
#define EEPROM_SIZE             (256u)
#define BLOCKING_WRITE          (1u)
#define REDUNDANT_COPY          (1u)
#define WEAR_LEVELLING_FACTOR   (2u)
#define SIMPLE_MODE             (0u)

/* Set the macro FLASH_REGION_TO_USE to either USER_FLASH or
 * EMULATED_EEPROM_FLASH to specify the region of the flash used for
 * emulated EEPROM.
 */
#define USER_FLASH              (0u)
#define EMULATED_EEPROM_FLASH   (1u)

#if CY_EM_EEPROM_SIZE
/* CY_EM_EEPROM_SIZE to determine whether the target has a dedicated EEPROM region or not */
#define FLASH_REGION_TO_USE     EMULATED_EEPROM_FLASH
#else
#define FLASH_REGION_TO_USE     USER_FLASH
#endif

/******************************************************************************
 * Enumerations
 *****************************************************************************/

typedef enum
{
    AV_STATE_IDLE,              /* Initial state (channel is unused) */
    AV_STATE_CONFIGURE,         /* Remote has sent configuration request */
    AV_STATE_OPEN,              /* Data channel connected but not streaming */
    AV_STATE_STARTED,           /* Data streaming */
    AV_STATE_RECONFIG,          /* Reconfiguring stream */
    AV_STATE_DISCONNECTING      /* Disconnecting */
} AV_STATE;


typedef enum
{
    FMT_RIFF = 0,
    FMT_FILESIZE,
    FMT_WAVE,
    FMT_MRK,
    FMT_LENFMT,
    FMT_TYPE,
    FMT_NUMCH,
    FMT_SAMPLERATE,
    FMT_BYTERATE,
    FMT_BLKALIGN,
    FMT_BYTESPERSAMPLE,
    FMT_DATASTR,
    FMT_DATASIZE
}WAV_FORMAT_HDR;

/* SBC structure - temporary to remove */

/******************************************************************************
 * Structures
 *****************************************************************************/

typedef struct
{
    wiced_bt_device_address_t peer_bda;      /* Peer bd address */
    AV_STATE state;                          /* AVDT State machine state */
    wiced_bt_a2dp_codec_info_t codec_config; /*Codec config */
    uint16_t stream_mtu;                     /* MTU of stream */
    uint16_t lcid;                           /* Local identifier */
} tAV_APP_CB;


/*******************************************************************************
* Function prototypes
*******************************************************************************/
wiced_result_t a2dp_source_management_callback (
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data);

void av_app_init (void);
void a2dp_source_control_cback (wiced_bt_a2dp_source_event_t event,
                wiced_bt_a2dp_source_event_data_t *p_data);
void a2dp_source_bt_set_pairability ( uint8_t pairing_allowed );
void a2dp_source_bt_print_local_bda( void );
void a2dp_source_bt_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data );
wiced_result_t a2dp_source_bt_inquiry( uint8_t enable );
wiced_result_t a2dp_source_bt_set_visibility( uint8_t discoverability, uint8_t connectability );
wiced_result_t a2dp_source_command_connect(wiced_bt_device_address_t bd_addr, uint32_t len);
wiced_result_t a2dp_source_command_disconnect();
void a2dp_source_command_stream_config(uint8_t sf, uint8_t chcfg);
wiced_result_t a2dp_source_command_stream_start();
wiced_result_t a2dp_source_command_stream_stop();

#endif  /* A2DP_SOURCE_H */

