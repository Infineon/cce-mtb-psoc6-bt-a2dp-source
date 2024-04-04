/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 6 MCU: Bluetooth Classic -
 *              A2DP source code example.
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
#include "cybsp.h"
#include "cybt_platform_trace.h"
#include "wiced_bt_utils.h"
#include "wiced_bt_cfg.h"
#include "app_bt_utils/app_bt_utils.h"
#include "a2dp_source.h"
#include "cy_retarget_io.h"
#include "cyhal.h"
#include "cyhal_gpio.h"

/******************************************************************************
 * Macros
 *****************************************************************************/
#define RESPONSE_YES_CAPITAL    ('Y')
#define RESPONSE_YES_SMALL      ('y')

#define RESPONSE_NO_CAPITAL    ('N')
#define RESPONSE_NO_SMALL      ('n')

/* UART function parameter value to wait forever */
#define UART_WAIT_FOR_EVER           (0)

#define LED_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE)
#define LED_TASK_PRIORITY            (configMAX_PRIORITIES - 6)

#define BT_STACK_HEAP_SIZE           (0XF00)

#define BT_TASK_STACK_SIZE           (1024 * 6)
#define BT_TASK_PRIORITY             (configMAX_PRIORITIES - 6)

#define LED_BLINK_TIMER_CLOCK_HZ     (10000)
#define LED_BLINK_TIMER_PERIOD       (9999)

#define TASK_DELAY_50MS              (50)
#define TASK_DELAY_100MS             (100)
#define TASK_DELAY_1SEC              (1000)
#define TIMER_INTERRUPT_PRIORITY     (7)

/******************************************************************************
 * Extern variables
 *****************************************************************************/
extern bool initiate_bt_device_connection;
extern bool skip_bt_scan;
extern wiced_bt_heap_t *p_default_heap;
extern const wiced_bt_cfg_settings_t a2dp_source_cfg_settings;
extern int8_t connection_status;

/* ***************************************************************************
 * Global variables
 * **************************************************************************/

  /* EEPROM macros */
/* EEPROM is used to store the paired peer BD address  */

/* EEPROM configuration and context structure. */
cy_stc_eeprom_config_t Em_EEPROM_config =
{
        .eepromSize = EEPROM_SIZE,
        .blockingWrite = BLOCKING_WRITE,
        .redundantCopy = REDUNDANT_COPY,
        .wearLevelingFactor = WEAR_LEVELLING_FACTOR,
};

cy_stc_eeprom_context_t Em_EEPROM_context;

#if (EMULATED_EEPROM_FLASH == FLASH_REGION_TO_USE)
CY_SECTION(".cy_em_eeprom")
#endif /* #if(FLASH_REGION_TO_USE) */

CY_ALIGN(CY_EM_EEPROM_FLASH_SIZEOF_ROW)

/* EEPROM storage in user flash or emulated EEPROM flash. */
const uint8_t eeprom_storage[CY_EM_EEPROM_GET_PHYSICAL_SIZE(EEPROM_SIZE, SIMPLE_MODE, WEAR_LEVELLING_FACTOR, REDUNDANT_COPY)] = {0u};

/* RAM arrays for holding EEPROM read and write data respectively. */
uint8_t eeprom_read_array[LOGICAL_EEPROM_SIZE];

/* ***************************************************************************
 * Static variables
 * **************************************************************************/
static TaskHandle_t bt_task_handle;
static TaskHandle_t led_task_handle;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/******************************************************************************
 * Function definitions
 ******************************************************************************/

/******************************************************************************
 * Function Name: application_start()
 *******************************************************************************
 * Summary:
 *          Starts BT stack and runs the BT thread monitoring for connection status.
 *
 * Parameters:
 *          Task parameters.
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void application_start(void *task_params)
{
	wiced_result_t wiced_result = WICED_BT_ERROR;
	wiced_bt_device_address_t bda_str;
	/* Variable for storing character read from terminal */
	uint8_t uart_response;
	const char *value = (const char *) &uart_response;

	/* Register call back and configuration with stack */
	wiced_result = wiced_bt_stack_init(a2dp_source_management_callback,&a2dp_source_cfg_settings);

	/* Check if stack initialization was successful */
	if (WICED_BT_SUCCESS == wiced_result)
	{
		/* Create a buffer heap, make it the default heap.  */
		p_default_heap = wiced_bt_create_heap("app", NULL, BT_STACK_HEAP_SIZE,NULL, WICED_TRUE);
	}

	if ((WICED_BT_SUCCESS == wiced_result) && (NULL != p_default_heap))
	{
		fprintf(stdout, "BT Stack Initialization Successful... \r\n");
		printf("A2DP Source Started \r\n");
	} 
    else /* Exit App if stack init was not successful or heap creation failed */
	{
		fprintf(stderr,
				"Error: BT Stack Initialization or heap creation failed!! Exiting App... \r\n");
		CY_ASSERT(0);
	}

	for (;;)
	{
		if (initiate_bt_device_connection)
		{
            memset(bda_str, 0, sizeof(bda_str));
			printf("Enter the Bluetooth Device Address of the Speaker/Earbuds (XX XX XX XX XX XX) : \r\n");
			for (int i = 0; i < BD_ADDR_LEN; i++)
			{
				for (int j = 0; j < 2; j++)
				{
					if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_response,UART_WAIT_FOR_EVER) == CY_RSLT_SUCCESS)
							
					{
						if (j == 0)
						{
							bda_str[i] = (((uint8_t) utl_strtoul(value,NULL,16)) << 4);	
						} 
						else
						{
							bda_str[i] |= (uint8_t) utl_strtoul(value, NULL,16);	
						}
						printf(
								"Bluetooth Device Address of the Speaker/Earbuds: %.2x:%.2x:%.2x:%.2x:%.2x:%.2x \r\n",
								bda_str[0], bda_str[1], bda_str[2], bda_str[3],
								bda_str[4], bda_str[5]);
								printf("\x1b[1F");
					}
				}
			}
			
            printf("Bluetooth Device Address of the Speaker/Earbuds is %x:%x:%x:%x:%x:%x \r\n",
					bda_str[0], bda_str[1], bda_str[2], bda_str[3], bda_str[4],
					bda_str[5]);

			Waiting_for_the_user_response:
			    printf("Do you want to connect to the Bluetooth Speaker/Earbuds (Y/N) ?\r\n");
			    cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_response,UART_WAIT_FOR_EVER);
			    printf("Received response: %c \r\n", uart_response);
			    if ((RESPONSE_YES_CAPITAL != uart_response) && (RESPONSE_YES_SMALL != uart_response) && (RESPONSE_NO_CAPITAL != uart_response) && (RESPONSE_NO_SMALL != uart_response))
			    {
		            printf("Invalid response \r\n");
				    printf("Response should be either Y/y or N/n \r\n");
					goto Waiting_for_the_user_response;
				}

			if ((RESPONSE_YES_CAPITAL == uart_response) || (RESPONSE_YES_SMALL == uart_response))
			{
				wiced_result = a2dp_source_bt_set_visibility(WICED_TRUE,WICED_TRUE);
						
				if (WICED_BT_SUCCESS == wiced_result)
				{
					printf("Connecting to Bluetooth Speaker/Earbuds...\r\n");
					wiced_result = a2dp_source_command_connect(bda_str,BD_ADDR_LEN);
		
					if (WICED_BT_SUCCESS == wiced_result)
					{
						printf("Connection Request sent. Waiting to connect with the Bluetooth Speaker/Earbuds ....\r\n");
						initiate_bt_device_connection = false;
					} 
					else
					{
						printf("Error: Failed to connect \r\n");
					}
				} 
				else
				{
					printf("Error: Failed to set the Bluetooth visibility \r\n");
				}
			}
		}

		vTaskDelay(TASK_DELAY_50MS);
	}
}

/******************************************************************************
 * Function Name: led_task()
 *******************************************************************************
 * Summary:
 *          Starts LED task.
 *
 * Parameters:
 *          Task parameters.
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void led_task(void *task_params)
{
    connection_status = A2DP_PEER_DISCONNECTED;

	for (;;)
	{
        /* Invert the USER LED state */
        cyhal_gpio_toggle(CYBSP_USER_LED);

		if (A2DP_PEER_CONNECTED == connection_status)
		{
 			vTaskDelay(TASK_DELAY_100MS);
 		}
 		else if (A2DP_PEER_DISCONNECTED == connection_status)
		{
 			vTaskDelay(TASK_DELAY_1SEC);
 		}
	}
}

/******************************************************************************
 * Function Name: bt_task_create()
 *******************************************************************************
 * Summary:
 *           BT task creation function wrapper
 *
 * Parameters:
 *           None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void bt_task_create(void)
{
	BaseType_t status;

	status = xTaskCreate(application_start, "BT task", BT_TASK_STACK_SIZE, NULL,BT_TASK_PRIORITY, &bt_task_handle);
		
	if (pdPASS != status)
	{
		printf("Error: Failed to start the BT task \r\n");
		CY_ASSERT(0);
	}

	status = xTaskCreate(led_task, "LED TASK", LED_TASK_STACK_SIZE, NULL,LED_TASK_PRIORITY, &led_task_handle);		

	if (pdPASS != status)
	{
		printf("Error: Failed to start the LED task \r\n");
		CY_ASSERT(0);
	}
}

/******************************************************************************
 * Function Name: main()
 *******************************************************************************
 * Summary:
 *          A2DP Source application entry function
 *
 * Parameters:
 *          None
 *
 *
 * Return:
 *          Status code
 *
 ******************************************************************************/
int main(void)
{
	cy_rslt_t result = CY_RSLT_TYPE_ERROR;
	/* Variable for storing character read from terminal */
	uint8_t uart_response;
    /* Return status for EEPROM. */
    cy_en_em_eeprom_status_t eeprom_return_value;

	/* Initialize and Verify the BSP initialization */
	result = cybsp_init();

	/* Board init failed. Stop program execution */
	if (CY_RSLT_SUCCESS != result)
	{
		CY_ASSERT(0);
	}

	/* Enable global interrupts */
	__enable_irq();

	result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
			CY_RETARGET_IO_BAUDRATE);

	/* Retarget-io init failed. Stop program execution */
	if (CY_RSLT_SUCCESS != result)
	{
		CY_ASSERT(0);
	}

	/* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
	printf("\x1b[2J\x1b[;H");
	printf("****************** "
			" PSoC™ 6 MCU: Bluetooth® Classic- A2DP source "
			"****************** \r\n\n");

    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

	if (CY_RSLT_SUCCESS != result)
	{
		printf("Error: Failed to initialize LED \r\n");
		CY_ASSERT(0);
	}

	/* Initialize the flash start address in EEPROM configuration structure. */
    Em_EEPROM_config.userFlashStartAddr = (uint32_t) eeprom_storage;

    eeprom_return_value = Cy_Em_EEPROM_Init(&Em_EEPROM_config, &Em_EEPROM_context);
    
	if (CY_EM_EEPROM_SUCCESS != eeprom_return_value)
    {
        printf("Error: Failed to initialize the EEPROM \r\n");
		CY_ASSERT(0);
    }

    /* Read the previously paired bluetooth device address from the EEPROM */
    eeprom_return_value = Cy_Em_EEPROM_Read(LOGICAL_EEPROM_START, eeprom_read_array,
                                          LOGICAL_EEPROM_SIZE, &Em_EEPROM_context);

    if (CY_EM_EEPROM_SUCCESS != eeprom_return_value)
    {
        printf("Error: Failed to read the previously paired bluetooth device address from the EEPROM \r\n");
		CY_ASSERT(0);
    }
    
	if (eeprom_read_array[0] != false)
	{
	printf("Previously paired Bluetooth Device Address of the Speaker/Earbuds is %.2x:%.2x:%.2x:%.2x:%.2x:%.2x \r\n\r\n",
								eeprom_read_array[0], eeprom_read_array[1], eeprom_read_array[2], eeprom_read_array[3],
								eeprom_read_array[4], eeprom_read_array[5]);
	}

	/* Configure platform specific settings for the BT device */
	cybt_platform_config_init(&cybsp_bt_platform_cfg);

    Waiting_for_the_user_response:
	    printf("Do you want to skip the Bluetooth Speaker/Earbuds scan and directly enter the address of the Bluetooth Speaker/Earbuds (Y/N) ?\r\n");
	    cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_response,UART_WAIT_FOR_EVER);
	    printf("Received response: %c\r\n", uart_response);
		if ((RESPONSE_YES_CAPITAL != uart_response) && (RESPONSE_YES_SMALL != uart_response) && (RESPONSE_NO_CAPITAL != uart_response) && (RESPONSE_NO_SMALL != uart_response))
		{
		    printf("Invalid response \r\n");
			printf("Response should be either Y/y or N/n \r\n");
			goto Waiting_for_the_user_response;
		}

	if ((RESPONSE_YES_CAPITAL == uart_response)|| (RESPONSE_YES_SMALL == uart_response))
	{
		skip_bt_scan = true;
		printf("Skipped the Bluetooth Speaker/Earbuds scanning\r\n");
	}

	/* Creating BT task */
	bt_task_create();

	/* Start the FreeRTOS scheduler */
	vTaskStartScheduler();

	/* Should never get here */
	CY_ASSERT(0);
}

/* [] END OF FILE */
