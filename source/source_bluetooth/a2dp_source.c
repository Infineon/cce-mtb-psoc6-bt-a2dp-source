/******************************************************************************
* File Name:   a2dp_source.c
*
* Description: Source file for A2DP source task.
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

/******************************************************************************
 * Includes
 *****************************************************************************/

#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif
#include "a2dp_source.h"
#include "cyhal_gpio.h"
#include "cybsp.h"
#include <wav_1_test.h>


/******************************************************************************
 * Macros
 *****************************************************************************/

#define A2DP_SOURCE_NVRAM_ID                0
#define WICED_HS_EIR_BUF_MAX_SIZE           (264U)

#define LINE_SPEED_44K                      (330U)
#define LINE_SPEED_48K                      (345U)
#define LINE_SPEED_DEF                      (229U)

#define MAX_KEY_SIZE                        (16U)

#define FMT_DATA_LEN_2                      (2U)
#define FMT_DATA_LEN_4                      (4U)

#define AUDIO_SAMPLE_RATE_48kHz             (48000U)
#define AUDIO_SAMPLE_RATE_44_1kHz           (44100U)
#define AUDIO_SAMPLE_RATE_32kHz             (32000U)
#define AUDIO_SAMPLE_RATE_16kHz             (16000U)

#define AUDIO_CHANNEL_MONO                  1
#define AUDIO_CHANNEL_STEREO                2
#define AUDIO_CHANNEL_DUAL_CHANNEL          2
#define AUDIO_CHANNEL_JOINT_STEREO          2

#define AUDIO_BIT_WIDTH                     16

#define DEFAULT_SBC_BITPOOL                 51
#define SBC_A2DP_MODE                       0
#define SBC_WIDE_BAND_MODE                  1

#define SEC1000_MICROSEC                    (1000000000U)
#define FRAME_TIME_STAMP                    (90000U)
#define MARKER_PAYLOAD_BYTE                 (0x60U)
#define INQUIRY_DURATION                    (5U)

#define A2DP_TASK_STACK_SIZE                512
#define A2DP_TASK_PRIORITY                  (configMAX_PRIORITIES - 3)

#define GPIO_INTERRUPT_PRIORITY             (7u)

#define TASK_DELAY_1MS                      1
#define TASK_DELAY_10MS                     10


/* Bluetooth Device Address of the Speaker/Earbud */
#define BDA_ADDRESS_BYTE_0                  0x4C
#define BDA_ADDRESS_BYTE_1                  0x72
#define BDA_ADDRESS_BYTE_2                  0x74
#define BDA_ADDRESS_BYTE_3                  0xED
#define BDA_ADDRESS_BYTE_4                  0x9F
#define BDA_ADDRESS_BYTE_5                  0xC1


/* ***************************************************************************
 * Static variables
 * **************************************************************************/

static uint16_t handle = 0x0000;
static uint32_t  SampleRate      = AUDIO_SAMPLE_RATE_48kHz;
static uint64_t  usTimeout;
static uint32_t dwPCMBytesPerFrame;
static uint32_t Timestamp;
static uint32_t TimestampInc;
static uint8_t  stream_avdt_handle;
static uint32_t frame_per_packet;
static unsigned char *pBitstream    = NULL;
static unsigned int asc_stream_read_max = 0;
static unsigned int asc_stream_read_counter = 0;
static TaskHandle_t a2dp_task_handle;

/* ***************************************************************************
 * Global variables
 * **************************************************************************/
tAV_APP_CB      av_app_cb;

SemaphoreHandle_t stream_buff_sem;
uint16_t delay_reported_from_sink_micro_sec = 0;
unsigned int    frame_size;
volatile wiced_bool_t bStreamingActive = WICED_FALSE;
wiced_bt_heap_t *p_default_heap = NULL;
uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };
cy_audio_sw_codec_encode_config_t enc_config;
cy_audio_sw_codec_t enc_handle;
uint32_t recommended_in_size;
uint32_t min_out_size;
uint32_t n_channels = 0;
uint32_t i_channel_mask;
uint32_t sample_rate;
uint32_t pcm_sz;
int32_t  length;
wiced_bt_device_link_keys_t nv_key_ram;
int16_t wav_header=0;
uint8_t *a2dp_frame;
uint8_t *encoder_output;
uint8_t  status_flag = 0;

#ifdef PSOC6_43012_BOARD
int8_t connection_status;
#endif

cyhal_gpio_callback_data_t gpio_btn_callback_data;

/******************************************************************************
 * Extern variables and functions
 *****************************************************************************/
extern uint32_t getTick();
extern const uint8_t                        a2dp_source_sdp_db[A2DP_SOURCE_SDP_DB_SIZE];
extern const wiced_bt_cfg_settings_t        a2dp_source_cfg_settings;
extern wiced_bt_a2dp_source_config_data_t   bt_audio_config;
extern tAV_APP_CB av_app_cb;

/*******************************************************************************
* Function prototypes
*******************************************************************************/

void a2dp_source_audio_thread (void* arg);
static uint8_t a2dp_source_encode_sendSBC ();
static void a2dp_source_parse_sbc_params (wiced_bt_a2d_sbc_cie_t * pSbc, uint32_t *pSf,
                                          uint32_t *pBlocks, uint32_t *pSubbands, uint32_t *pMode);
uint16_t a2dp_source_calculate_delay ();
static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event);

/******************************************************************************
 * Function definitions
 ******************************************************************************/

/* ****************************************************************************
 * Function Name: get_audio_freq_name
 *******************************************************************************
 * Summary:
 *          Gives the string equivalent for the Event type
 *
 * Parameters:
 *          freq - Sample Frequency type
 *
 * Return:
 *          Pointer to String containing the Frequency Name
 *
 * ***************************************************************************/
static const char *get_audio_freq_name(uint8_t freq)
{
    switch((int)freq)
    {
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_16)
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_32)
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_44)
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_48)
    }

    return NULL;
}

/* ****************************************************************************
 * Function Name: get_audio_chcfg_name
 *******************************************************************************
 * Summary:
 *          Gives the string equivalent for the Event type
 *
 * Parameters:
 *          chcfg - Channel Configuration type
 *
 * Return:
 *          Pointer to String containing the Channel Configuration Name
 *
 * ***************************************************************************/
static const char *get_audio_chcfg_name(uint8_t chcfg)
{
    switch((int)chcfg)
    {
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_MONO)
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_DUAL)
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_STEREO)
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_JOINT)
    }

    return NULL;
}

/* ****************************************************************************
 * Function Name: a2dp_source_deinit_streaming
 ******************************************************************************
 * Summary:
 *        De-initialize the buffers and variables used for Audio Streaming state
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_deinit_streaming()
{
    WICED_BT_TRACE ("De-iniatializing the stream buffer \n");
    bStreamingActive = FALSE;
    if (a2dp_frame != NULL)
    {
        free(a2dp_frame);
    }
    if (encoder_output != NULL)
    {
        free(encoder_output);
    }
    a2dp_frame = NULL;
    encoder_output = NULL;
}

/* ****************************************************************************
 * Function Name: a2dp_source_update_sink_rep_delay
 ******************************************************************************
 * Summary:
 *          Updates the delay report value to the latest as per the Sink
 *
 * Parameters:
 *          delay_ms: Delay in Milliseconds
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_update_sink_rep_delay(uint16_t delay_ms)
{
    WICED_BT_TRACE("Updated Sink reported delay %u ms", delay_ms);
    delay_reported_from_sink_micro_sec += delay_ms * 1000;
}

/******************************************************************************
 * Function Name: asc_reset_params()
 *******************************************************************************
 * Summary: Decodes a wav file and points to the data by skipping the wav header.
 *
 *
 * Parameters:
 *          None
 *
 *
 * Return:
 *          None
 *
 ******************************************************************************/


static void asc_reset_params()
{
#if !ENABLE_AM_FS
    pBitstream = (unsigned char *)&hex_array;
    asc_stream_read_max = hex_array_size;
    asc_stream_read_counter = 0;
#endif
    if( 0 != cy_wav_header_decode( &n_channels,
                                &i_channel_mask, &sample_rate,
                                &pcm_sz, &length,
                                pBitstream,&asc_stream_read_counter,asc_stream_read_max) )
    {
        cy_asc_log_err(CY_RSLT_ASC_GENERIC_ERROR, "Error wav header parsing failed");
    }

    cy_asc_log_info("Input Params:  sample_rate: %lu, pcm_sz: %lu, n_channels: %lu, length: %ld "
                            , sample_rate
                            , pcm_sz
                            , n_channels
                            , length );
}

/* ****************************************************************************
 * Function Name: a2dp_source_start_streaming
 ******************************************************************************
 * Summary:
 *          Loads the required .wav file to memory, initializes the Encoder,
 *          Starts a thread for encoding and streaming audio
 *
 * Parameters:
 *          bActivate: State of Streaming as requested by the user
 *          avdt_handle: AVDTP Connection handle as received in the event
 *                       WICED_BT_A2DP_SOURCE_START_IND_EVT
 *          p_stream_cfg: CODEC Configuration
 *          peer_mtu: Stream MTU of the Peer side
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_start_streaming (wiced_bool_t bActivate, uint8_t avdt_handle,
                                  wiced_bt_a2d_sbc_cie_t *p_stream_cfg, uint16_t peer_mtu)
{
    uint32_t  NumBlocks       = 0;
    uint32_t  NumSubBands     = 0;
    uint32_t  NumChannels     = 0;
    uint32_t  Mode            = 0;

    sbc_encoder_params_t sbc_params;

    unsigned int fr_per_1000sec, pkts_per_1000sec;
    int join;
    cy_rslt_t            result;

    /* Safety check in case we are called more than once to do the same thing */
    if (bActivate == bStreamingActive)
    {
        WICED_BT_TRACE ("Streaming State: Already in %s",
                      bActivate ? "ACTIVE" : "SUSPEND");
        return;
    }

    WICED_BT_TRACE ("Streaming State: setting to %s \n",
                  bActivate ? "ACTIVE" : "SUSPEND");

    if (!bActivate)
    {
        bStreamingActive = FALSE;
        return;
    }

    stream_avdt_handle = avdt_handle;

    WICED_BT_TRACE ("Streaming parameters: %u  samplesPerSec: %u  peer_mtu: %u",
                  bActivate, p_stream_cfg->samp_freq, peer_mtu);

    printf("\r\nPeer device configured to --> samp_freq %x ch_mode %d block_len %d num_subbands %d alloc_mthd %d max_bitpool %d min_bitpool %d \r\n",\
            p_stream_cfg->samp_freq,p_stream_cfg->ch_mode,p_stream_cfg->block_len,p_stream_cfg->num_subbands,p_stream_cfg->alloc_mthd,p_stream_cfg->max_bitpool,p_stream_cfg->min_bitpool);

    NumBlocks       = p_stream_cfg->block_len;
    NumSubBands     = p_stream_cfg->num_subbands;
    Mode            = p_stream_cfg->ch_mode;

    /* Extract the codec parameters */
    a2dp_source_parse_sbc_params (p_stream_cfg, &SampleRate, &NumBlocks, &NumSubBands, &Mode);

    WICED_BT_TRACE ("SampleRate: %u  NumBlocks: %u  NumSubBands: %u  Mode: %u",
                    SampleRate, NumBlocks, NumSubBands, Mode);

    NumChannels = Mode >= SBC_STEREO ? SBC_STEREO : SBC_DUAL;
    join = Mode == SBC_JOINT_STEREO ? SBC_DUAL : SBC_MONO;

    sbc_params.sbc_config.allocation_method = (p_stream_cfg->alloc_mthd == A2D_SBC_IE_ALLOC_MD_S ? SBC_SNR : SBC_LOUDNESS);
    sbc_params.sbc_config.channel_mode      = Mode;
    sbc_params.sbc_config.sbc_mode          = SBC_A2DP_MODE;
    sbc_params.sbc_config.sub_blocks        = NumBlocks;
    sbc_params.sbc_config.sub_bands         = NumSubBands;
    sbc_params.sbc_config.bit_pool          = DEFAULT_SBC_BITPOOL;
    sbc_params.sample_rate                  = AUDIO_SAMPLE_RATE_48kHz;
    sbc_params.channels                     = AUDIO_CHANNEL_STEREO;
    sbc_params.bit_width                    = AUDIO_BIT_WIDTH;

    asc_reset_params();
    wav_header = asc_stream_read_counter;

    printf("Wav header is %d \r\n",wav_header);

    result = sbc_encoder_app_init(&enc_handle, &sbc_params);
    if (result != CY_RSLT_SUCCESS)
    {
        cy_asc_log_err(result, "sw encoder init failed");
        return;
    }

    result = cy_audio_sw_codec_get_encoder_recommended_inbuf_size(enc_handle,&recommended_in_size);
    if(CY_RSLT_SUCCESS != result)
    {
        cy_asc_log_err(result, "failed to get recommended input buf sz.");
        return;
    }

    result = cy_audio_sw_codec_get_encoder_recommended_outbuf_size(enc_handle, recommended_in_size, &min_out_size);
    if(CY_RSLT_SUCCESS != result)
    {
        cy_asc_log_err(result, "failed to get recommended output buf sz.");
        return;
    }

    printf("SBC encoder recommended_in_size: %lu, min_out_size:%lu \r\n",recommended_in_size, min_out_size);

    /*FORMULA as in A2DP Spec:
     *
     * frame_length = 4 + (4 * nrof_subbands * nrof_channels ) / 8
     * nrof_blocks * nrof_channels * bitpool / 8 .
     * for the MONO and DUAL_CHANNEL channel modes, and
     * frame_length = 4 + (4 * nrof_subbands * nrof_channels ) / 8
     * (join * nrof_subbands + nrof_blocks * bitpool ) / 8
     */

    /* Calculate the periodicity to compress and send data in microseconds */
    frame_size = 4 + (4 * NumSubBands * sbc_params.channels) / 8
    + (join * NumSubBands + NumBlocks * sbc_params.sbc_config.bit_pool) / 8;

    frame_per_packet = (peer_mtu - AVDT_MEDIA_OFFSET) / frame_size;

    if (frame_per_packet > A2D_SBC_HDR_NUM_MSK)
      frame_per_packet = A2D_SBC_HDR_NUM_MSK;

    if (frame_per_packet == 0)
      frame_per_packet = 1;

    fr_per_1000sec = (SampleRate * 1000) / (NumSubBands * NumBlocks);

    pkts_per_1000sec = fr_per_1000sec / frame_per_packet;
    usTimeout = SEC1000_MICROSEC / pkts_per_1000sec;

    /*NumSubBands * NumBlocks * NumChannels* (bits_per_sample/2 )*/
    dwPCMBytesPerFrame = 2 * NumSubBands * NumBlocks * NumChannels;

    WICED_BT_TRACE ("Stream Starting:  pcm_bytes_per_frame %u \n", dwPCMBytesPerFrame);

    printf("Frame size is %u Frame per packet %lu \r\n",frame_size,frame_per_packet);
    /* Allocate memory */
    a2dp_frame = (uint8_t*)malloc(frame_size*frame_per_packet+1);
    encoder_output = (uint8_t*)malloc(min_out_size);

    Timestamp = 0;
    TimestampInc = ((NumBlocks * NumSubBands * FRAME_TIME_STAMP) / SampleRate) * frame_per_packet;
    printf("Timestamp increment is %ld \r\n",TimestampInc);

    printf("fr_per_1000sec %d \r\n",fr_per_1000sec);
    printf("pkts_per_1000sec %d \r\n",pkts_per_1000sec);
    printf("usTimeout %lld \r\n",usTimeout);




    WICED_BT_TRACE ("Stream Parameters: Sample rate %d  Blk: %d   SB: %d  BitP: %d  uS: %d  Frame Size: %d  Frame Per Packet: %d  fp1000s: %u \n",\
                    SampleRate, NumBlocks, NumSubBands, sbc_params.sbc_config.bit_pool, usTimeout, frame_size, frame_per_packet, fr_per_1000sec);


    result = xTaskCreate(a2dp_source_audio_thread, "Streaming task",A2DP_TASK_STACK_SIZE,NULL,A2DP_TASK_PRIORITY,&a2dp_task_handle);


    if (result != pdPASS)
    {
        cy_asc_log_info("Error in starting Streaming task \n");
    }

}

/* ****************************************************************************
 * Function Name: a2dp_source_encode_sendSBC
 ******************************************************************************
 * Summary:
 *          Calls Encoder to encode the PCM data and sends to Stack
 *
 * Parameters:
 *          None
 * Return:
 *          A2DP_APP_SUCCESS if the data is sent successfully
 *          A2DP_APP_FAILED if there was an error
 *
 * ***************************************************************************/
uint8_t a2dp_source_encode_sendSBC ()
{
    uint16_t    result      = A2DP_APP_SUCCESS;
    uint8_t     *pSBC;
    uint32_t    i;
    uint16_t    pkt_len     = 0;

    uint8_t     *output_buff;
    uint32_t    in_size     = 0;
    uint32_t    out_size    = 0;

    xSemaphoreTake(stream_buff_sem,portMAX_DELAY);

    pkt_len = frame_size*frame_per_packet;

       asc_stream_read_max = hex_array_size;

    if (asc_stream_read_counter+recommended_in_size>asc_stream_read_max)
    {
        asc_stream_read_counter = wav_header;
    }
    in_size = recommended_in_size;
    out_size = min_out_size;
    output_buff = a2dp_frame;

    memset(encoder_output,0x00,out_size);
    memset(a2dp_frame,0x00,pkt_len+1);

    for (i = 0; i < frame_per_packet; i++)
    {

        pSBC = (uint8_t *)hex_array;

        pSBC += asc_stream_read_counter;

        result = sbc_encoder_app_encode(enc_handle, pSBC, &in_size, encoder_output, &out_size);
        if (result != CY_RSLT_SUCCESS)
        {
            printf("Encoding error \r\n");
        }


        if (i == 0)
        {
            *output_buff = frame_per_packet;
            output_buff += 1;
        }
        memcpy(output_buff,encoder_output,out_size);
        output_buff += out_size;
        asc_stream_read_counter += recommended_in_size;
    }

    result = wiced_bt_avdt_write_req (stream_avdt_handle, a2dp_frame, pkt_len+1,Timestamp, MARKER_PAYLOAD_BYTE, 0);

    if (result != AVDT_SUCCESS)
    {
        printf("AVDT failed with result %d for timestamp %ld for outsize %d \r\n",result, Timestamp,pkt_len);
        result = A2DP_APP_FAILED;
    }

    Timestamp += TimestampInc;

    if(asc_stream_read_counter >= asc_stream_read_max)
    {
        asc_stream_read_counter = wav_header;
    }

    return result;
}

/* ****************************************************************************
 * Function Name: a2dp_source_calculate_delay
 ******************************************************************************
 * Summary:
 *          Calculate the required delay to wait for next audio write
 *
 * Parameters:
 *          None
 * Return:
 *          delay_in_micro_sec: Delay in microseconds
 *
 * ***************************************************************************/
uint16_t a2dp_source_calculate_delay()
{
    float bytes_per_pkt = 0;
    float delay_in_micro_sec = 0;

    bytes_per_pkt = frame_size * frame_per_packet;
    delay_in_micro_sec = ((bytes_per_pkt * 1000) / (SampleRate *2)) * 1000;
    WICED_BT_TRACE (" Required delay is %d between pkts of size %d to achieve SampleRate %d",
                    (uint16_t)delay_in_micro_sec, (uint16_t)bytes_per_pkt, SampleRate);
    return (uint16_t)delay_in_micro_sec;
}

/* ****************************************************************************
 * Function Name: a2dp_source_audio_thread
 ******************************************************************************
 * Summary:
 *          Thread to handle the audio encoding and streaming
 *
 * Parameters:
 *          NULL
 * Return:
 *          NULL
 *
 * ***************************************************************************/
void a2dp_source_audio_thread(void* arg)
{
    cy_rslt_t result = 0;
    uint16_t delay_in_micro_sec = 0;

    stream_buff_sem = xSemaphoreCreateBinary();
    if ( stream_buff_sem == NULL )
    {
        printf("Error in semaphore \r\n");
        return;
    }

    /* Initialize the User Button */
    result = cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error: Failed to initialize User Button \r\n");
        CY_ASSERT(0);
    }
    /* Configure GPIO interrupt */
    gpio_btn_callback_data.callback = gpio_interrupt_handler;
    cyhal_gpio_register_callback(CYBSP_USER_BTN, &gpio_btn_callback_data);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, GPIO_INTERRUPT_PRIORITY, true);

    WICED_BT_TRACE("a2dp_src_thread usTimeout: %u \n", usTimeout);

    delay_in_micro_sec = a2dp_source_calculate_delay() + delay_reported_from_sink_micro_sec;
    printf("Delay in micro sec is %d \r\n",delay_in_micro_sec);
    printf("portTICK_PERIOD_MS is %ld (float)(delay_in_micro_sec*0.001)/portTICK_PERIOD_MS is %f %f \r\n",portTICK_PERIOD_MS,(float)(delay_in_micro_sec*0.001)/portTICK_PERIOD_MS,(float)(delay_in_micro_sec*0.001)/portTICK_PERIOD_MS);

    printf("\r\nPress User Btn1 to play/pause the stream... \r\n");
    xSemaphoreGive(stream_buff_sem);

    while (1)
    {
        if (bStreamingActive == WICED_TRUE && status_flag == 1)
        {
            printf("Streaming Audio,Press User Btn1 to pause... \r\n");
            status_flag=0;
        }
        if (bStreamingActive == WICED_FALSE && status_flag == 1)
        {
            printf("Streaming Audio Paused !, Press User Btn1 to play ... \r\n");
            status_flag=0;
        }
        if (bStreamingActive == WICED_FALSE)
        {
            vTaskDelay(TASK_DELAY_10MS);
            continue;
        }

        if (A2DP_APP_FAILED == a2dp_source_encode_sendSBC())
        {
            WICED_BT_TRACE("Encode and Send failed. \n");
        }
        vTaskDelay(TASK_DELAY_1MS);
    }

    WICED_BT_TRACE("a2dp_src_thread() exit");
}

/* ****************************************************************************
 * Function Name: a2dp_source_parse_sbc_params
 ******************************************************************************
 * Summary:
 *          Parse SBC Parameters and set teh configuration for Encoder
 *
 * Parameters:
 *          NULL
 * Return:
 *          NULL
 *
 * ***************************************************************************/
static void a2dp_source_parse_sbc_params (wiced_bt_a2d_sbc_cie_t * pSbc, uint32_t *pSf,
                                          uint32_t *pBlocks, uint32_t *pSubbands, uint32_t *pMode)
{
    switch (pSbc->samp_freq)
    {
    case A2D_SBC_IE_SAMP_FREQ_16:
        *pSf = AUDIO_SAMPLE_RATE_16kHz;
        break;
    case A2D_SBC_IE_SAMP_FREQ_32:
        *pSf = AUDIO_SAMPLE_RATE_32kHz;
        break;
    case A2D_SBC_IE_SAMP_FREQ_44:
        *pSf = AUDIO_SAMPLE_RATE_44_1kHz;
        break;
    case A2D_SBC_IE_SAMP_FREQ_48:
        *pSf = AUDIO_SAMPLE_RATE_48kHz;
        break;
    }

    switch (pSbc->block_len)
    {
    case A2D_SBC_IE_BLOCKS_4:
        *pBlocks = SBC_BLOCK_0;
        break;
    case A2D_SBC_IE_BLOCKS_8:
        *pBlocks = SBC_BLOCK_1;
        break;
    case A2D_SBC_IE_BLOCKS_12:
        *pBlocks = SBC_BLOCK_2;
        break;
    case A2D_SBC_IE_BLOCKS_16:
        *pBlocks = SBC_BLOCK_3;
        break;
    }

    switch (pSbc->num_subbands)
    {
    case A2D_SBC_IE_SUBBAND_4:
        *pSubbands = SUB_BANDS_4;
        break;
    case A2D_SBC_IE_SUBBAND_8:
        *pSubbands = SUB_BANDS_8;
        break;
    }

    switch (pSbc->ch_mode)
    {
    case A2D_SBC_IE_CH_MD_MONO:
        *pMode = SBC_MONO;
        break;
    case A2D_SBC_IE_CH_MD_DUAL:
        *pMode = SBC_DUAL;
        break;
    case A2D_SBC_IE_CH_MD_STEREO:
        *pMode = SBC_STEREO;
        break;
    case A2D_SBC_IE_CH_MD_JOINT:
        *pMode = SBC_JOINT_STEREO;
        break;
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_set_audio_streaming
 ******************************************************************************
 * Summary:
 *          Set Audio Streaming state and initiate the action accordingly.
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
static void a2dp_source_set_audio_streaming(uint8_t handle, wiced_bool_t start_audio)
{
    WICED_BT_TRACE("Setting Audio Stream State to %d \n", start_audio);

    if (start_audio)
    {
        wiced_bt_dev_cancel_sniff_mode(av_app_cb.peer_bda);
        a2dp_source_start_streaming(WICED_TRUE, handle, &av_app_cb.codec_config.cie.sbc, av_app_cb.stream_mtu);
    }
    else
    {
        a2dp_source_start_streaming(WICED_FALSE, handle, NULL, 0);
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_control_cback
 ******************************************************************************
 * Summary:
 *          Control callback supplied by  the a2dp source profile code.
 *
 * Parameters:
 *          event - control event called back
 *          p_data - event data
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void  a2dp_source_control_cback (wiced_bt_a2dp_source_event_t event, wiced_bt_a2dp_source_event_data_t *p_data)
{
#if WICED_BT_A2DP_SOURCE_DEBUG
    printf("a2dp_source_control_cback is %d \r\n",event);
#endif

    switch (event)
    {
    case WICED_BT_A2DP_SOURCE_CONNECT_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        /**< Connected event, received on establishing connection to a peer device. Ready to stream. */
        if (p_data->connect.result == WICED_SUCCESS)
        {
            /* Save the address of the remote device on remote connection */
            memcpy (av_app_cb.peer_bda, p_data->connect.bd_addr,
                sizeof(wiced_bt_device_address_t));
            av_app_cb.lcid = p_data->connect.lcid;

            /* Maintain State */
            av_app_cb.state = AV_STATE_OPEN;

            WICED_BT_TRACE ("Connected to addr: <%x> Handle %d \n\r",p_data->connect.bd_addr,
                            p_data->connect.handle);

            handle = p_data->connect.handle;
#ifdef PSOC6_43012_BOARD
            connection_status = A2DP_PEER_CONNECTED;
#endif
            printf("Connected to BT Speaker/Earbuds ... \r\n\r\n");
            printf("Configuring A2DP stream to 48KHz, Stereo \r\n");
            a2dp_source_command_stream_config ((uint8_t)AUDIO_SF_48K, AUDIO_CHCFG_STEREO);


            wiced_bt_dev_setAclPacketTypes (
            av_app_cb.peer_bda,
            HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5 | /* Use 1 mbps 5 slot packets */
            HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DM3 | /* Use 1 mbps 3 slot packets */
            HCI_PKT_TYPES_MASK_DH1 | HCI_PKT_TYPES_MASK_DM1 | /* Use 1 mbps 1 slot packets */
            HCI_PKT_TYPES_MASK_NO_3_DH1 | /* Don't use 3 mbps 1 slot packets */
            HCI_PKT_TYPES_MASK_NO_3_DH3 | /* Don't use 3 mbps 3 slot packets */
            HCI_PKT_TYPES_MASK_NO_3_DH5); /* Don't use 3 mbps 5 slot packets */

            a2dp_source_command_stream_start ();


        }
        else
        {
            WICED_BT_TRACE (" a2dp source connection to <%x> failed %d \n", p_data->connect.bd_addr,
                  p_data->connect.result);
            printf("Connection Failed to %x:%x:%x:%x:%x:%x. Reset the board. \r\n",p_data->connect.bd_addr[0], p_data->connect.bd_addr[1], p_data->connect.bd_addr[2],p_data->connect.bd_addr[3],p_data->connect.bd_addr[4],p_data->connect.bd_addr[5]);
        }



        break;

    case WICED_BT_A2DP_SOURCE_DISCONNECT_EVT:
        /**< Disconnected event, received on disconnection from a peer device */
        /* Maintain State */
        av_app_cb.state = AV_STATE_IDLE;
        a2dp_source_deinit_streaming();
        handle = 0; /* reset connection handle */
        printf ("A2DP peer disconnected. Reset the board. \r\n");
#ifdef PSOC6_43012_BOARD
        connection_status = A2DP_PEER_DISCONNECTED;
#endif
        break;

    case WICED_BT_A2DP_SOURCE_CONFIGURE_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        /* Maintain State */
        av_app_cb.state = AV_STATE_CONFIGURE;
        memcpy (&av_app_cb.codec_config, p_data->set_config.codec_config,
          sizeof(wiced_bt_a2dp_codec_info_t));
        av_app_cb.stream_mtu = p_data->set_config.stream_mtu;
        break;

    case WICED_BT_A2DP_SOURCE_START_IND_EVT:
        printf("WICED_BT_A2DP_SOURCE_START_IND_EVT \r\n");
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        if (!wiced_bt_a2dp_source_send_start_response (p_data->start_ind.handle,
                             p_data->start_ind.label,
                             A2D_SUCCESS))
        {
            printf("wiced_bt_a2dp_source_send_start_response no error !!!! \r\n");
            if (av_app_cb.state != AV_STATE_STARTED)
            {
                WICED_BT_TRACE (" a2dp source streaming started handle:%d lcid %x \n",
                  p_data->start_cfm.handle, av_app_cb.lcid);
                printf("WICED_BT_A2DP_SOURCE_START_IND_EVT start streaming \r\n");

                a2dp_source_set_audio_streaming (p_data->start_ind.handle, WICED_TRUE);
                av_app_cb.state = AV_STATE_STARTED;
            }
            else
            {
                WICED_BT_TRACE (" a2dp source streaming already started \n");
            }
        }
        break;

    case WICED_BT_A2DP_SOURCE_START_CFM_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        /*Start stream event, received when audio streaming is about to start*/
        if (p_data->start_cfm.result != WICED_SUCCESS)
        {
            WICED_BT_TRACE ("Stream start Error \n");
            break;
        }
        if (av_app_cb.state != AV_STATE_STARTED)
        {
            WICED_BT_TRACE (" a2dp source streaming started handle:%d lcid %x \n",
              p_data->start_cfm.handle, av_app_cb.lcid);
            a2dp_source_set_audio_streaming (p_data->start_cfm.handle, WICED_TRUE);
            av_app_cb.state = AV_STATE_STARTED;
        }
        else
        {
            WICED_BT_TRACE (" a2dp source streaming already started \n");
        }
        break;

    case WICED_BT_A2DP_SOURCE_SUSPEND_EVT:
        /**< Suspend stream event, received when audio streaming is suspended */
        /* Maintain State */
        av_app_cb.state = AV_STATE_OPEN;
        WICED_BT_TRACE (" a2dp source streaming suspended \n");
        a2dp_source_set_audio_streaming (p_data->suspend.handle, WICED_FALSE);
        break;

    case WICED_BT_A2DP_SOURCE_WRITE_CFM_EVT:

        xSemaphoreGive(stream_buff_sem);

        break;

    case WICED_BT_A2DP_SOURCE_DELAY_RPT_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        WICED_BT_TRACE ("a2dp source WICED_BT_A2DP_SOURCE_DELAY_RPT_EVT delay is %d\n",
              p_data->delay_ms);
        a2dp_source_update_sink_rep_delay (p_data->delay_ms);
        break;

    default:
        WICED_BT_TRACE ("a2dp source Unhandled Event %d\n", event);
        break;
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_inquiry_result_cback
 ******************************************************************************
 * Summary:
 *          Handle Inquiry result callback from teh stack, format and
 *          send event over UART
 *
 * Parameters:
 *          p_inquiry_result - Inquiry result consisting BD address,
 *                             RSSI and Device class
 *          p_eir_data - EIR data
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_bt_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    uint8_t   len; /* length of EIR data */

    if (p_inquiry_result == NULL)
    {
        WICED_BT_TRACE( "Inquiry complete \n");
    }
    else
    {
        WICED_BT_TRACE( "Inquiry result: ");
        print_bd_address (p_inquiry_result->remote_bd_addr );
        WICED_BT_TRACE_ARRAY((uint8_t*)(p_inquiry_result->dev_class), 3, "COD :" );
        WICED_BT_TRACE( "RSSI: ", p_inquiry_result->rssi );

        /* currently callback does not pass the data of the adv data,
         * need to go through the data
         * zero len in the LTV means that there is no more data
         */
        if (( p_eir_data != NULL ) && ( len = *p_eir_data ) != 0)
        {
            WICED_BT_TRACE_ARRAY( ( uint8_t* )( p_eir_data ), len, "EIR :" );
        }
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_inquiry
 ******************************************************************************
 * Summary:
 *        Handle Inquiry command from user
 *
 * Parameters:
 *        enable - Enable Inquiry if 1, Cancel if 0
 *
 * Return:
 *        wiced_result_t: result of start or cancel inquiry operation initiation
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_bt_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {
        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = INQUIRY_DURATION;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &a2dp_source_bt_inquiry_result_cback );
        if (result == WICED_BT_PENDING)
        {
            result = WICED_BT_SUCCESS;
        }
        WICED_BT_TRACE( "Inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "Cancel inquiry:%d\n", result );
    }
    return result;
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_set_visibility
 ******************************************************************************
 * Summary:
 *          Handle Set Visibility command
 *
 * Parameters:
 *          discoverability: Discoverable if 1, Non-discoverable if 0
 *          connectability: Connectable if 1, Non-connectable if 0
 *
 * Return:
 *          wiced_result_t: result of set visibility operation
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_bt_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if ((discoverability > 1) || (connectability > 1))
    {
        WICED_BT_TRACE( "Invalid Input \n");
        status = WICED_BT_ERROR;
    }
    else if ((discoverability != 0) && (connectability == 0))
    {
        /* we cannot be discoverable and not connectable */
        WICED_BT_TRACE("we cannot be discoverable and not connectable \n");
        status = WICED_BT_ERROR;
    }
    else
    {
        wiced_bt_dev_set_discoverability((discoverability != 0) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability((connectability != 0) ? WICED_TRUE : WICED_FALSE,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);
    }
    return status;
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_set_pairability
 ******************************************************************************
 * Summary:
 *          Handle Set Pairability command
 *
 * Parameters:
 *          pairing_allowed: Pairing allowed if 1, not allowed if 0
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_bt_set_pairability( uint8_t pairing_allowed )
{
    wiced_bt_set_pairable_mode(pairing_allowed, TRUE);
    WICED_BT_TRACE( "Set the pairing allowed to %d \n", pairing_allowed );
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_print_local_bda
 ******************************************************************************
 * Summary:
 *          Print Local BDA command
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_bt_print_local_bda( void )
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE( "Local Bluetooth Device Address:");
    print_bd_address (bda);
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_connect
 ******************************************************************************
 * Summary:
 *          Connects to the A2DP Sink with the given BD-Address
 *
 * Parameters:
 *          bd_addr: Remote BD Address
 *          len: Length of the BD-address.
 *
 * Return:
 *          status: result of a2dp connect API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_connect(wiced_bt_device_address_t bd_addr, uint32_t len)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if (handle > 0)
    {
        WICED_BT_TRACE("Already Connected \n\r");
        status = WICED_BT_ERROR;
    }
    else
    {
        WICED_BT_TRACE("Connecting to [%x] \n\r", bd_addr );
        status = wiced_bt_a2dp_source_connect(bd_addr);
    }
    return status;
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_disconnect
 ******************************************************************************
 * Summary:
 *          Disconnects the A2DP Sink that is already connected
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status: result of a2dp disconnect API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_disconnect()
{
    wiced_result_t status = WICED_BT_SUCCESS;

    WICED_BT_TRACE( "Disconnecting Connection with Handle %d\n\r", handle );
    if (handle == 0)
    {
        status = WICED_BT_ERROR;
    }
    else
    {
        status = wiced_bt_a2dp_source_disconnect(handle);
        av_app_cb.state = AV_STATE_DISCONNECTING;
    }

    return status;
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_stream_config
 ******************************************************************************
 * Summary:
 *          Configures the A2DP Stream
 *
 * Parameters:
 *          sf: sample frequency
 *          chcfg: channel configuration.
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_command_stream_config(uint8_t sf, uint8_t chcfg)
{
    switch (sf)
    {
    case AUDIO_SF_16K:
        sf = A2D_SBC_IE_SAMP_FREQ_16;
        break;
    case AUDIO_SF_32K:
        sf = A2D_SBC_IE_SAMP_FREQ_32;
        break;
    case AUDIO_SF_44_1K:
        sf = A2D_SBC_IE_SAMP_FREQ_44;
        break;
    case AUDIO_SF_48K:
        sf = A2D_SBC_IE_SAMP_FREQ_48;
        break;
    default:
        sf = A2D_SBC_IE_SAMP_FREQ_48;
        break;
    }

    switch (chcfg)
    {
    case AUDIO_CHCFG_MONO:
        chcfg = A2D_SBC_IE_CH_MD_MONO;
        break;
    case AUDIO_CHCFG_STEREO:
        chcfg = A2D_SBC_IE_CH_MD_STEREO;
        break;
    default:
        chcfg = A2D_SBC_IE_CH_MD_STEREO;
        break;
    }

    if (!(sf == bt_audio_config.default_codec_config.cie.sbc.samp_freq) &&
     (chcfg == bt_audio_config.default_codec_config.cie.sbc.ch_mode))
    {
        a2dp_source_deinit_streaming();
        bt_audio_config.default_codec_config.cie.sbc.samp_freq = sf;
        bt_audio_config.default_codec_config.cie.sbc.ch_mode = chcfg;
    }
    WICED_BT_TRACE( "Handle %d sf:%s chcfg:%s\n\r", handle,
                    get_audio_freq_name (sf), get_audio_chcfg_name (chcfg) );
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_stream_start
 ******************************************************************************
 * Summary:
 *          Initiates Stream Start
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status: result of Stream Start API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_stream_start()
{
    WICED_BT_TRACE( "Stream Start for Connection Handle %d\n\r", handle);

    if (handle == 0)
    {
       return WICED_BT_ERROR;
    }
    return wiced_bt_a2dp_source_start(handle, &bt_audio_config.default_codec_config);
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_stream_stop
 ******************************************************************************
 * Summary:
 *          Initiates Stream Suspend
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status: result of Stream Suspend API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_stream_stop()
{
    WICED_BT_TRACE( "Stream Stop for Connection Handle %d\n\r", handle);

    if (handle == 0)
    {
        return WICED_BT_ERROR;
    }

    return wiced_bt_a2dp_source_suspend(handle);
}

/******************************************************************************
 * Function Name: a2dp_source_write_eir
 *******************************************************************************
 * Summary:
 *          Prepare extended inquiry response data.  Current version publishes
 *          audio source services.
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void a2dp_source_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t *p_tmp;
    uint8_t nb_uuid = 0;
    uint8_t length;

    /* Allocating a buffer from the public pool */
    pBuf = (uint8_t*) wiced_bt_get_buffer ( WICED_HS_EIR_BUF_MAX_SIZE);

    if (!pBuf)
    {
        WICED_BT_TRACE ("Buffer allocation for EIR Data failed \n");
        return;
    }

    WICED_BT_TRACE ("EIR allocated Buffer: %x\n", pBuf);
    p = pBuf;

    length = (uint8_t) strlen ((char*) a2dp_source_cfg_settings.device_name);

    WICED_BT_TRACE ("length %d\n", (uint8_t)length);
    UINT8_TO_STREAM (p, length + 1);
    UINT8_TO_STREAM (p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE);
    memcpy (p, a2dp_source_cfg_settings.device_name, length);
    p += length;

    /* Add other BR/EDR UUIDs */
    p_tmp = p;
    p++;
    UINT8_TO_STREAM (p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);
    UINT16_TO_STREAM (p, UUID_SERVCLASS_AUDIO_SOURCE);
    nb_uuid++;

    /* Now, we can update the UUID Tag's length */
    UINT8_TO_STREAM (p_tmp, (nb_uuid * LEN_UUID_16) + 1);

    /* Last Tag */
    UINT8_TO_STREAM (p, 0x00);

    /* print EIR data */
    WICED_BT_TRACE_ARRAY( ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ), "EIR :" );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    /* Allocated buffer not anymore needed. Free it */
    wiced_bt_free_buffer (pBuf);
}

/******************************************************************************
 * Function Name: a2dp_source_write_nvram
 *******************************************************************************
 * Summary:
 *          Write NVRAM function is called to store information but in PSoC6 it will not be stored.
 *
 * Parameters:
 *          nvram_id: NVRAM Id
 *          data_len: Length of the data to be written
 *          p_data: Data to be written
 *
 * Return:
 *          Number of bytes written
 *
 *****************************************************************************/
uint16_t a2dp_source_write_nvram( int nvram_id, int data_len, void *p_data)
{
    uint16_t        bytes_written = 0;
    int i=0;

    if (p_data != NULL)
    {
        bytes_written=data_len;
        memcpy(&nv_key_ram,p_data,sizeof(wiced_bt_device_link_keys_t));
        printf("Link Key is ");
        for (i=0; i<LINK_KEY_LEN; i++) {
            printf("%x ",nv_key_ram.key_data.br_edr_key[i]);
        }
        printf("\r\n");

        WICED_BT_TRACE("NVRAM ID:%d written :%d bytes \n", nvram_id, bytes_written);
    }

    return (bytes_written);
}


/******************************************************************************
 * Function Name: a2dp_source_read_nvram
 *******************************************************************************
 * Summary:
 *          Read data from the NVRAM and return in the passed buffer
 *
 * Parameters:
 *          nvram_id: NVRAM Id
 *          data_len: Length of the data to be read
 *          p_data: Data buffer pointer to hold read data
 *
 * Return:
 *          Number of bytes read
 *
 *****************************************************************************/
uint16_t a2dp_source_read_nvram( int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;

    if ((p_data != NULL) && (data_len >= sizeof(wiced_bt_device_link_keys_t)))
    {
        memset(&nv_key_ram,0xFF,sizeof(wiced_bt_device_link_keys_t));
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d \n", nvram_id,
                       sizeof(wiced_bt_device_link_keys_t), read_bytes );
    }
    return (read_bytes);
}

/******************************************************************************
 * Function Name: a2dp_source_app_init()
 *******************************************************************************
 * Summary:
 *   This function handles application level initialization tasks and is
 *   called from the BT management callback once the Bluetooth Stack enabled
 *   event (BTM_ENABLED_EVT) is triggered
 *   This function is executed in the BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 ******************************************************************************/
void a2dp_source_app_init( void )
{
    wiced_result_t result = WICED_BT_SUCCESS;
    av_app_cb.state = AV_STATE_IDLE;

    /* Register with the A2DP source profile code */
    result = wiced_bt_a2dp_source_init( &bt_audio_config,
                                        a2dp_source_control_cback, p_default_heap);
    if (result == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE( "A2DP Source initialized \n\r");

    }
    else
    {
        WICED_BT_TRACE( "A2DP Source initialization failed \n\r");
    }
}

/******************************************************************************
 * Function Name: a2dp_source_management_callback()
 *******************************************************************************
 * Summary:
 *   This is a Bluetooth stack event handler function to receive management
 *   events from the Bluetooth stack and process as per the application.
 *
 * Parameters:
 *   wiced_bt_management_evt_t event : BLE event code of one byte length
 *   wiced_bt_management_evt_data_t *p_event_data: Pointer to BTStack management
 *   event structures
 *
 * Return:
 *  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 ******************************************************************************/

wiced_result_t a2dp_source_management_callback( wiced_bt_management_evt_t event,
                                              wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_ble_advert_mode_t          *p_mode; /* Advertisement Mode */
    wiced_bt_device_address_t           bda = {0};
    wiced_result_t                      result = WICED_BT_SUCCESS;

    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_encryption_status_t   *p_encryption_status;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    int                                 pairing_result;
    wiced_bt_device_address_t           bda_str;

    WICED_BT_TRACE("A2dp source management callback: %s\n", get_bt_event_name(event));

    switch( event )
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Bluetooth Enabled \n");
            WICED_BT_TRACE("Local Bluetooth Address: ");
            print_bd_address(bda);

            /* Set Discoverable */
            a2dp_source_bt_set_visibility (WICED_FALSE, WICED_TRUE);

            /* Enable pairing */
            wiced_bt_set_pairable_mode(WICED_TRUE, 0);

            a2dp_source_write_eir();

            /* create SDP records */
            wiced_bt_sdp_db_init((uint8_t*) a2dp_source_sdp_db,
                sizeof(a2dp_source_sdp_db));

           /* start the a2dp application */
           a2dp_source_app_init();

           a2dp_source_bt_set_pairability(1);
           /* If Inquiry is required enable the a2dp_source_bt_inquiry API. */
           /* a2dp_source_bt_inquiry (1); */
           a2dp_source_bt_set_visibility (WICED_TRUE, WICED_TRUE);
           printf("Connecting to BT Speaker/Earbuds...\r\n");

           bda_str[0] = (uint8_t)BDA_ADDRESS_BYTE_0;
           bda_str[1] = (uint8_t)BDA_ADDRESS_BYTE_1;
           bda_str[2] = (uint8_t)BDA_ADDRESS_BYTE_2;
           bda_str[3] = (uint8_t)BDA_ADDRESS_BYTE_3;
           bda_str[4] = (uint8_t)BDA_ADDRESS_BYTE_4;
           bda_str[5] = (uint8_t)BDA_ADDRESS_BYTE_5;

           printf("Bluetooth Device Address of Speaker/Earbuds is %x:%x:%x:%x:%x:%x \r\n",bda_str[0], bda_str[1], bda_str[2],bda_str[3],bda_str[4],bda_str[5]);

           printf("Skip Bonding Request \r\n");
           /* If the below API is enabled, then a2dp_source_command_connect should be called inside BTM_PAIRING_COMPLETE_EVT */
           /* wiced_bt_dev_sec_bond (bda_str,BLE_ADDR_PUBLIC,BT_TRANSPORT_BR_EDR,0, NULL); */

           a2dp_source_command_connect (bda_str, BD_ADDR_LEN);
           printf("Connection Request sent. Waiting to connect with the BT Speaker/Earbuds ....\r\n");

        } else
        {
            WICED_BT_TRACE("Bluetooth Enable failed \n");
        }
        break;

    case BTM_DISABLED_EVT:
            WICED_BT_TRACE("Bluetooth Disabled \n");
        break;

    case BTM_SECURITY_FAILED_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("Security failed: %d / %d\n", p_event_data->security_failed.status,
                                                       p_event_data->security_failed.hci_status);
        break;

    case BTM_PIN_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("remote address= %x\n", p_event_data->pin_request.bd_addr);
        wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS,
                                                        WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* If this is just works pairing, accept.
         * Otherwise send event to the MCU to confirm the same value.
         */
        WICED_BT_TRACE("User Confirmation. BDA %x, Key %d \n", p_event_data->user_confirmation_request.bd_addr,
                                                   p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
        break;

    case BTM_PASSKEY_NOTIFICATION_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("PassKey Notification. BDA %x, Key %d \n",
                                   p_event_data->user_passkey_notification.bd_addr,
                                   p_event_data->user_passkey_notification.passkey);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* Use the default security for BR/EDR*/
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %x\n",
                       p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap =
                                                        BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req =
                                                        BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        p_event_data->pairing_io_capabilities_br_edr_request.oob_data = WICED_FALSE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req =
                                                        BTM_AUTH_ALL_PROFILES_NO;
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* Use the default security for BLE */
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %x\n",
                p_event_data->pairing_io_capabilities_ble_request.bd_addr);
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap  =
                                                    BTM_IO_CAPABILITIES_DISPLAY_AND_YES_NO_INPUT;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req =
                                                    BTM_LE_AUTH_REQ_SC_MITM_BOND;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size  = MAX_KEY_SIZE;
        p_event_data->pairing_io_capabilities_ble_request.init_keys =
                                                    BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys =
                                                    BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_pairing_cmpl = &p_event_data->pairing_complete;
        if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
        }
        else
        {
            pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
        }
        WICED_BT_TRACE( "Pairing Result: %d\n", pairing_result );


        bda_str[0] = (uint8_t)BDA_ADDRESS_BYTE_0;
        bda_str[1] = (uint8_t)BDA_ADDRESS_BYTE_1;
        bda_str[2] = (uint8_t)BDA_ADDRESS_BYTE_2;
        bda_str[3] = (uint8_t)BDA_ADDRESS_BYTE_3;
        bda_str[4] = (uint8_t)BDA_ADDRESS_BYTE_4;
        bda_str[5] = (uint8_t)BDA_ADDRESS_BYTE_5;

        // Enable here if bonding request API is called so that connection happens after pairing.
        //a2dp_source_command_connect (bda_str, BD_ADDR_LEN);
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE( "Encryption Status:(%x) res:%d\n", p_encryption_status->bd_addr,
                                                           p_encryption_status->result );
        break;

    case BTM_SECURITY_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE( "Security Request Event \n");
        wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT - dev: [%x]  Len:%d\n",
                             p_event_data->paired_device_link_keys_update.bd_addr,
                             sizeof(wiced_bt_device_link_keys_t));

        /* This application supports a single paired host, we can save keys
         * under the same NVRAM ID overwriting previous pairing if any */
        a2dp_source_write_nvram(A2DP_SOURCE_NVRAM_ID,
                sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update);

         break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        WICED_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT - dev: [%x]  Len:%d \r\n",
                            p_event_data->paired_device_link_keys_request.bd_addr,
                            sizeof(wiced_bt_device_link_keys_t));
        printf("\r\nLink key request - %x:%x:%x:%x:%x:%x\r\n",p_event_data->paired_device_link_keys_request.bd_addr[0], p_event_data->paired_device_link_keys_request.bd_addr[1], p_event_data->paired_device_link_keys_request.bd_addr[2],p_event_data->paired_device_link_keys_request.bd_addr[3],p_event_data->paired_device_link_keys_request.bd_addr[4],p_event_data->paired_device_link_keys_request.bd_addr[5]);

        /* read existing key from the NVRAM  */
        if (a2dp_source_read_nvram (A2DP_SOURCE_NVRAM_ID,
                  &p_event_data->paired_device_link_keys_request,
                  sizeof(wiced_bt_device_link_keys_t)) != 0)
        {
            result = WICED_BT_SUCCESS;
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE ("Key retrieval failure\n");
        }
        break;

    case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* save keys to NVRAM */
        //p_keys = (uint8_t*) &p_event_data->local_identity_keys_update;
        WICED_BT_TRACE("  result: %d \n", result);
        break;

    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* read keys from NVRAM */
        //p_keys = (uint8_t*) &p_event_data->local_identity_keys_request;
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
        WICED_BT_TRACE( "Power mgmt status event: bd ( %x ) status:%d hci_status:%d\n",
                        p_power_mgmt_notification->bd_addr, p_power_mgmt_notification->status,
                        p_power_mgmt_notification->hci_status);
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        p_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE( "Advertisement State Change: %d\n", *p_mode);
        break;

    case BTM_BLE_CONNECTION_PARAM_UPDATE:
        if (p_event_data == NULL)
        {
            WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
            break;
        }
        /* Connection parameters updated */
        if (WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
        {
            WICED_BT_TRACE("Supervision Time Out = %d\n",
                    (p_event_data->ble_connection_param_update.supervision_timeout * 10));
        }
        break;

    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x %s\n", event, get_bt_event_name(event));
        break;
    }
    return result;
}

/*******************************************************************************
* Function Name: gpio_interrupt_handler
********************************************************************************
* Summary:
*   GPIO interrupt handler.
*
* Parameters:
*  void *handler_arg (unused)
*  cyhal_gpio_event_t (unused)
*
*******************************************************************************/

static void gpio_interrupt_handler(void *handler_arg, cyhal_gpio_event_t event)
{
    status_flag = 1;
    if (bStreamingActive == WICED_FALSE)
    {
        bStreamingActive = WICED_TRUE;
    } else if (bStreamingActive == WICED_TRUE)
    {
        bStreamingActive = WICED_FALSE;
    }
}

/* [] END OF FILE */
