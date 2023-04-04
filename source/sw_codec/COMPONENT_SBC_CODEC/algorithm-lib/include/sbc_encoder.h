/******************************************************************************
**
**  File Name:   $RCSfile: sbc_encoder.h,v $
**
**  Description: This file contains constants and structures used by Encoder.
**
**  Revision :   $Id: sbc_encoder.h,v 1.28 2006/06/27 12:29:32 mjougit Exp $
**
**  Copyright (c) 1999-2008, Broadcom Corp., All Rights Reserved.
**  Broadcom Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/

#ifndef SBC_ENCODER_H
#define SBC_ENCODER_H

// Based on SBC 22 rel
#define ENCODER_VERSION 003
#ifdef BUILDCFG
    #include "bt_target.h"
#endif

/*DEFINES*/
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE (!FALSE)
#endif


/* Set SBC_WB to TRUE to enable Wide Band Speech SBC Codec */
#ifndef SBC_WB
#define SBC_WB  TRUE
#endif

#ifndef SBC_MAX_NUM_FRAME
#define SBC_MAX_NUM_FRAME 1
#endif

/* Set SBC_NEW_API  to TRUE to enable New API requested by FW team. SBC_NO_PCM_CPY_OPTION should be set to TRUE  */
#ifndef SBC_NEW_API
#define SBC_NEW_API TRUE
#endif

#ifndef SBC_MAX_PACKET_LENGTH
#define SBC_MAX_PACKET_LENGTH 520
#endif

#ifndef SBC_DSP_OPT
#define SBC_DSP_OPT FALSE
#endif

/* Set SBC_USE_ARM_PRAGMA to TRUE to use "#pragma arm section zidata" */
#ifndef SBC_USE_ARM_PRAGMA
#define SBC_USE_ARM_PRAGMA FALSE
#endif

/* Set SBC_ARM_ASM_OPT to TRUE in case the target is an ARM */
/* this will replace all the 32 and 64 bit mult by in line assembly code */
#ifndef SBC_ARM_ASM_OPT
#define SBC_ARM_ASM_OPT FALSE
#endif

/* green hill compiler option -> Used to distinguish the syntax for inline assembly code*/
#ifndef SBC_GHS_COMPILER
#define SBC_GHS_COMPILER FALSE
#endif

/* ARM compiler option -> Used to distinguish the syntax for inline assembly code */
#ifndef SBC_ARM_COMPILER
#define SBC_ARM_COMPILER FALSE
#endif

/* Set SBC_IPAQ_OPT to TRUE in case the target is an ARM */
/* 32 and 64 bit mult will be performed using SINT64 ( usualy __int64 ) cast that usualy give optimal performance if supported */
#ifndef SBC_IPAQ_OPT
#define SBC_IPAQ_OPT FALSE
#endif

/* Debug only: set SBC_IS_64_MULT_IN_WINDOW_ACCU to TRUE to use 64 bit multiplication in the windowing */
/* -> not recomended, more MIPS for the same restitution.  */
#ifndef SBC_IS_64_MULT_IN_WINDOW_ACCU
#define SBC_IS_64_MULT_IN_WINDOW_ACCU  FALSE
#endif /*SBC_IS_64_MULT_IN_WINDOW_ACCU */

/* Set SBC_IS_64_MULT_IN_IDCT to TRUE to use 64 bits multiplication in the DCT of Matrixing */
/* -> more MIPS required for a better audio quality. comparasion with the SIG utilities shows a division by 10 of the RMS */
/* CAUTION: It only apply in the if SBC_FAST_DCT is set to TRUE */
#ifndef SBC_IS_64_MULT_IN_IDCT
#define SBC_IS_64_MULT_IN_IDCT  FALSE
#endif /*SBC_IS_64_MULT_IN_IDCT */

/* set SBC_IS_64_MULT_IN_QUANTIZER to TRUE to use 64 bits multiplication in the quantizer */
/* setting this flag to FALSE add whistling noise at 5.5 and 11 KHz usualy not perceptible by human's hears. */
#ifndef SBC_IS_64_MULT_IN_QUANTIZER
#define SBC_IS_64_MULT_IN_QUANTIZER  TRUE
#endif /*SBC_IS_64_MULT_IN_IDCT */

/* Debug only: set this flag to FALSE to disable fast DCT algorithm */
#ifndef SBC_FAST_DCT
#define SBC_FAST_DCT  TRUE
#endif /*SBC_FAST_DCT */

/* In case we do not use joint stereo mode the flag save some RAM and ROM in case it is set to FALSE */
#ifndef SBC_JOINT_STE_INCLUDED
#define SBC_JOINT_STE_INCLUDED TRUE
#endif

/* TRUE -> application should provide PCM buffer, FALSE PCM buffer reside in SBC_ENC_PARAMS */
#ifndef SBC_NO_PCM_CPY_OPTION
#define SBC_NO_PCM_CPY_OPTION TRUE
#endif

#define MINIMUM_ENC_VX_BUFFER_SIZE (8*10*2)
#ifndef ENC_VX_BUFFER_SIZE
#define ENC_VX_BUFFER_SIZE (MINIMUM_ENC_VX_BUFFER_SIZE + 64)
/*#define ENC_VX_BUFFER_SIZE MINIMUM_ENC_VX_BUFFER_SIZE + 1024*/
#endif

/*constants used for index calculation*/
#define SBC_BLK (SBC_MAX_NUM_OF_CHANNELS * SBC_MAX_NUM_OF_SUBBANDS)

#if ( (SBC_NO_PCM_CPY_OPTION==FALSE) && (SBC_NEW_API == TRUE))
    #error SBC_NEW_API = TRUE only available with SBC_NO_PCM_CPY_OPTION = TRUE !!
#endif



/* For SBC WB */
enum
{
    SBC_ENC_MODE_A2DP,      // Original SBC for A2DP
    SBC_ENC_MODE_WB         // SBC for WideBand
};


#define SBC_NB_OF_UUI 1
#define SBC_MAX_NUM_OF_BLOCKS_WB   15


#define SBC_MAX_NUM_OF_SUBBANDS 8
#define SBC_MAX_NUM_OF_CHANNELS 2

 #define SBC_MAX_NUM_OF_BLOCKS   16

#define SBC_LOUDNESS    0
#define SBC_SNR 1

#define SUB_BANDS_8 8
#define SUB_BANDS_4 4

#define SBC_sf16000 0
#define SBC_sf32000 1
#define SBC_sf44100 2
#define SBC_sf48000 3

#define SBC_MONO    0
#define SBC_DUAL    1
#define SBC_STEREO  2
#define SBC_JOINT_STEREO    3

#define SBC_BLOCK_0 4
#define SBC_BLOCK_1 8
#define SBC_BLOCK_2 12
#define SBC_BLOCK_3 16

#define SBC_NULL    0

#define SBC_FAILURE 0
#define SBC_SUCCESS 1


/* Compute optimum buffer size */
#define SB_BUFFER_SIZE_WB (1 * SBC_MAX_NUM_OF_SUBBANDS * SBC_MAX_NUM_OF_BLOCKS_WB)
#define SB_BUFFER_SIZE_SIG (SBC_MAX_NUM_OF_CHANNELS * SBC_MAX_NUM_OF_SUBBANDS * SBC_BLOCK_3)

#if (SBC_WB)
#if (SB_BUFFER_SIZE_WB >SB_BUFFER_SIZE_SIG)
#define SB_BUFFER_SIZE SB_BUFFER_SIZE_WB
#else
#endif
#define SB_BUFFER_SIZE SB_BUFFER_SIZE_SIG
#endif


#include "sbc_types.h"

typedef struct SBC_ENC_PARAMS_TAG
{
    SINT16 s16SamplingFreq;                         /* 16k, 32k, 44.1k or 48k*/
    SINT16 s16ChannelMode;                          /* mono, dual, streo or joint streo*/
    SINT16 s16NumOfSubBands;                        /* 4 or 8 */
    SINT16 s16NumOfChannels;
    SINT16 s16NumOfBlocks;                          /* 4, 8, 12 or 16*/
    SINT16 s16AllocationMethod;                     /* loudness or SNR*/
    SINT16 s16BitPool;                              /* 16*numOfSb for mono & dual;
                                                       32*numOfSb for stereo & joint stereo */
    UINT16 u16BitRate;
    UINT8   u8NumPacketToEncode;                    /* number of sbc frame to encode. Default is 1 */
#if (SBC_JOINT_STE_INCLUDED == TRUE)
    SINT16 as16Join[SBC_MAX_NUM_OF_SUBBANDS];       /*1 if JS, 0 otherwise*/
#endif

    SINT16 s16MaxBitNeed;
    SINT16 as16ScaleFactor[SBC_MAX_NUM_OF_CHANNELS*SBC_MAX_NUM_OF_SUBBANDS];

    SINT16 *ps16NextPcmBuffer;
#if (SBC_NO_PCM_CPY_OPTION == TRUE)
    SINT16 *ps16PcmBuffer;
#else
    SINT16 as16PcmBuffer[SBC_MAX_NUM_FRAME*SBC_MAX_NUM_OF_BLOCKS * SBC_MAX_NUM_OF_CHANNELS * SBC_MAX_NUM_OF_SUBBANDS];
#endif

    SINT16  s16ScartchMemForBitAlloc[16];

    SINT32  s32SbBuffer[SB_BUFFER_SIZE];

    SINT16 as16Bits[SBC_MAX_NUM_OF_CHANNELS*SBC_MAX_NUM_OF_SUBBANDS];

    UINT8  *pu8Packet;
    UINT8  *pu8NextPacket;
    UINT16 FrameHeader;
    UINT16 u16PacketLength;
#if (SBC_WB)
    UINT16 sbc_mode;            /*0: A2DP mode, 1: WideBand mode */
    UINT16 uui_id;              /*UUI id  */
#endif

}SBC_ENC_PARAMS;

#if (SBC_WB)
/* In SBC WideBand, the SBC parameters are defined according to UUI */
typedef struct SBC_ENC_WB_UUI_TAG
{
    SINT16 NumOfSubBands;
    SINT16 NumOfBlock;
    SINT16 Bitpool;
    UINT16 uui_id;
}SBC_ENC_WB_UUI_PARAM;
#endif



#ifdef __cplusplus
extern "C"
{
#endif
#ifndef SBC_API
#define SBC_API
#endif
#if (SBC_NEW_API)
SBC_API extern int SBC_Encoder_encode(SBC_ENC_PARAMS *pstrEncParams, unsigned char * pcm_in, unsigned char * sbc_out, unsigned int len);
#endif
SBC_API extern void SBC_Encoder(SBC_ENC_PARAMS *strEncParams);
SBC_API extern SINT16 SBC_Encoder_update_uui(SBC_ENC_PARAMS *pstrEncParams, UINT16 new_uui);
SBC_API extern SINT16 SBC_Encoder_Init(SBC_ENC_PARAMS *strEncParams);
#ifdef __cplusplus
}
#endif
#endif
