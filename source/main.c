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
 *
 *****************************************************************************/
#include "cybsp.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "app_bt_utils/app_bt_utils.h"
#include "a2dp_source.h"
#include "cy_retarget_io.h"
#include "cyhal.h"
#include "cyhal_gpio.h"
#ifdef PSOC6_43012_BOARD
#include "cy_rgb_led.h"
#endif

/******************************************************************************
 * Macros
 *****************************************************************************/

#define BT_STACK_HEAP_SIZE              (0XF00U)

#define BT_TASK_STACK_SIZE              (128)
#define BT_TASK_PRIORITY                (configMAX_PRIORITIES - 6)

#define LED_BLINK_TIMER_CLOCK_HZ        (10000)
#define LED_BLINK_TIMER_PERIOD          (9999)

#define TASK_DELAY_1MS                  1
#define TIMER_INTERRUPT_PRIORITY        7

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static void timer_init(void);
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);

/******************************************************************************
 * Extern variables
 *****************************************************************************/

extern wiced_bt_heap_t *p_default_heap;
extern const wiced_bt_cfg_settings_t        a2dp_source_cfg_settings;

#ifdef PSOC6_43012_BOARD
extern int8_t connection_status;
#endif

/* ***************************************************************************
 * Global variables
 * **************************************************************************/
volatile bool timer_interrupt_flag = false;

/* ***************************************************************************
 * Static variables
 * **************************************************************************/

static TaskHandle_t bt_task_handle;
static cyhal_timer_t led_blink_timer;

/******************************************************************************
 * Function definitions
 ******************************************************************************/

/******************************************************************************
 * Function Name: hci_control_proc_rx_cmd()
 *******************************************************************************
 * Summary:
 *          Function to handle HCI receive
 *
 * Parameters:
 *          uint8_t* p_buffer  : rx buffer
 *          uint32_t length     : rx buffer length
 *
 * Return:
 *          status code
 *
 ******************************************************************************/
uint32_t hci_control_proc_rx_cmd(uint8_t *p_buffer, uint32_t length)
{
    return 0;
}

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
    wiced_result_t wiced_result = WICED_BT_SUCCESS;

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(a2dp_source_management_callback, &a2dp_source_cfg_settings);

    WICED_BT_TRACE("BT Stack Init - A2DP Source Start");

    /* Check if stack initialization was successful */

    if (WICED_BT_SUCCESS == wiced_result)
    {
        /* Create a buffer heap, make it the default heap.  */
        p_default_heap = wiced_bt_create_heap("app", NULL, BT_STACK_HEAP_SIZE,
                NULL, WICED_TRUE);
        printf("Creating heap \r\n");
    }

    if ((WICED_BT_SUCCESS == wiced_result) && (NULL != p_default_heap))
    {
        fprintf(stdout, "Bluetooth Stack Initialization Successful...\n\r");
    }
    else /* Exit App if stack init was not successful or heap creation failed */
    {
        fprintf(stderr, "Bluetooth Stack Initialization or heap creation failed!! Exiting App...\n\r");
        CY_ASSERT(0);
    }
    printf("Task wait \n\r");
    timer_init();

#ifdef PSOC6_43012_BOARD
    connection_status = A2DP_PEER_DISCONNECTED;
#endif

    for (;;)
    {         /* Check if timer elapsed (interrupt fired) and toggle the LED */
#ifdef PSOC6_43012_BOARD
        if (connection_status == A2DP_PEER_CONNECTED)
        {
            cy_rgb_led_on(CY_RGB_LED_COLOR_GREEN, CY_RGB_LED_MAX_BRIGHTNESS);
            connection_status = A2DP_PEER_MAINTAIN_STATUS;
        }
        if (connection_status == A2DP_PEER_DISCONNECTED)
        {
            cy_rgb_led_on(CY_RGB_LED_COLOR_RED, CY_RGB_LED_MAX_BRIGHTNESS);
            connection_status = A2DP_PEER_MAINTAIN_STATUS;
        }
#endif
        if (timer_interrupt_flag)
        {
            /* Clear the flag */
            timer_interrupt_flag = false;

#ifdef PSOC6_43012_BOARD
            /* Toggle RGB LED */
            cy_rgb_led_toggle();
#endif

#ifdef PSOC6_4343W_BOARD
            /* Invert the USER LED state */
            cyhal_gpio_toggle(CYBSP_USER_LED);
#endif
        }
           vTaskDelay(TASK_DELAY_1MS);
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
    status = xTaskCreate(application_start, "BT task",BT_TASK_STACK_SIZE,NULL,BT_TASK_PRIORITY,&bt_task_handle);
    if (status != pdPASS)
    {
        printf("Error in starting BT task \n");
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
    cy_rslt_t result;

    /* Initialize and Verify the BSP initialization */
    result = cybsp_init();
    CY_ASSERT(CY_RSLT_SUCCESS == result);

    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

/* Enable global interrupts */
    __enable_irq();
#ifdef PSOC6_43012_BOARD
    result = cy_rgb_led_init(CYBSP_LED_RGB_RED, CYBSP_LED_RGB_GREEN, CYBSP_LED_RGB_BLUE, CY_RGB_LED_ACTIVE_LOW);
#endif
#ifdef PSOC6_4343W_BOARD
    result = cyhal_gpio_init(CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
#endif

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error: Failed to initialize LED \r\n");
        CY_ASSERT(0);
    }

   /* Initialising the HCI UART for Host control */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Creating BT task */
    bt_task_create();

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: timer_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks
* continuously and produces a periodic interrupt on every terminal count
* event. The period is defined by the 'period' and 'compare_value' of the
* timer configuration structure 'led_blink_timer_cfg'. Without any changes,
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
*******************************************************************************/
void timer_init(void)
{
    cy_rslt_t result;

    const cyhal_timer_cfg_t led_blink_timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = LED_BLINK_TIMER_PERIOD,   /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
        * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&led_blink_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction,
        duration */
    cyhal_timer_configure(&led_blink_timer, &led_blink_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&led_blink_timer, isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                            TIMER_INTERRUPT_PRIORITY, true);

    /* Start the timer with the configured settings */
    cyhal_timer_start(&led_blink_timer);
}


/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    timer_interrupt_flag = true;
}
/* [] END OF FILE */
