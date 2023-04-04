/******************************************************************************
**
**  File Name:   $RCSfile: sbc_types.h,v $
**
**  Description: Data type declarations.
**
**  Revision :   $Id: sbc_types.h,v 1.6 2004/03/30 16:48:17 slefort Exp $
**
**  Copyright (c) 1999-2002, Widcomm Inc., All Rights Reserved.
**  Widcomm Bluetooth Core. Proprietary and confidential.
**
******************************************************************************/

#ifndef SBC_TYPES_H
#define SBC_TYPES_H
#define CY_ASC_SBC_MODIFICATION 1

#ifdef CY_ASC_SBC_MODIFICATION
#include <stdint.h>
#endif

typedef short SINT16;
typedef int SINT32;
#ifdef CY_ASC_SBC_MODIFICATION
/* In PSoC, with out the below line, compiler error is thrown */
typedef unsigned short UINT16;
typedef unsigned char UINT8;
typedef uint32_t UINT32;
#endif

#ifdef LINUX_PLATFORM
typedef long long SINT64;
#else
#ifdef CY_ASC_SBC_MODIFICATION
#ifdef _WIN32
typedef __int64 SINT64;
#else
typedef int64_t SINT64;
#endif
#else
typedef __int64 SINT64;
#endif
#endif


#define abs32(x) ( (x >= 0) ? x : (-x) )

#ifndef DATA_TYPES_DEFINED
#define DATA_TYPES_DEFINED
//==================================================================================================
// Types
//==================================================================================================

//! Unsigned 8-bit integer.
typedef unsigned char uint8_t;

//! Signed 8-bit integer.
typedef signed char int8_t;

//! Unsigned 16-bit integer.
typedef unsigned short int uint16_t;

//! Signed 16-bit integer.
typedef signed short int int16_t;

#ifdef _WIN32
//! Unsigned 32-bit integer.
typedef unsigned int uint32_t;
// NOTE: not long int, because on 64-bit compilers (and more to the point, coverity running on
// 64-bit linux), long int is treated as 64 bits.

//! Signed 32-bit integer.
typedef signed int int32_t;
// NOTE: not long int, because on 64-bit compilers (and more to the point, coverity running on
// 64-bit linux), long int is treated as 64 bits.
#endif

#ifndef LINUX_PLATFORM
//! Unsigned 64-bit integer.
typedef unsigned long long int uint64_t;
#endif

//! Byte type (unsigned 8-bit integer).
typedef unsigned char BYTE;

//! Boolean type in its most efficient form, for use in function arguments and return values.
typedef unsigned int BOOL32;

//! Boolean type in its most size-efficient form, for use in structures.
typedef unsigned char BOOL8;

#ifdef _64_BIT_
typedef INT64           INTPTR;
typedef uint64_t          UINTPTR;
typedef uint64_t          TIME_STAMP;
#else
typedef int32_t           INTPTR;
typedef uint32_t        UINTPTR;
typedef uint32_t        TIME_STAMP;
#endif

#endif

#endif
