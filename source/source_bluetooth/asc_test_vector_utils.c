/******************************************************************************
* File Name:   asc_test_vector_utils.c
*
* Description: Source for Audio codec MW helper.
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
#include "asc_test_vector_utils.h"

/******************************************************************************
 * Function definitions
 ******************************************************************************/

/******************************************************************************
 * Function Name: asc_byte_stream_read()
 *******************************************************************************
 * Summary:
 *          Reads byte stream.
 *
 * Parameters:
 *          Byte stream configuration.
 *
 *
 * Return:
 *          Size of read bytes.
 *
 ******************************************************************************/

int asc_byte_stream_read(unsigned char *bitstream, unsigned char *bytes, unsigned int size,unsigned int *current_read_pos, unsigned int bitstream_max )
{
    int      i = 0;

    unsigned short int i_buff_size = 0;
    for (i = 0; i < size && (*current_read_pos < bitstream_max); i++)
    {
        bytes[i] = bitstream[*current_read_pos];
        *current_read_pos = *current_read_pos + 1;
        i_buff_size++;
    }

    return i_buff_size;
}

/******************************************************************************
 * Function Name: asc_byte_stream_write()
 *******************************************************************************
 * Summary: Function to write byte stream (empty now)
 *
 *
 * Parameters:
 *
 *
 *
 * Return:
 *
 *
 ******************************************************************************/

int asc_byte_stream_write(unsigned char *bytes, unsigned int size)
{
    /* Do nothing for PSoC6*/
    return size;
}

/* [] END OF FILE */
