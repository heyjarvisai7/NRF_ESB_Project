/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example_freertos
 *
 * @brief Blinky FreeRTOS Example Application main file.
 *
 * This file contains the source code for a sample application using FreeRTOS to blink LEDs.
 *
 */

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "bsp.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "app_error.h"

#include "nrf_esb.h"

#include <stdbool.h>
#include <stdint.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_error.h"
#include "boards.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_uart.h"
#include "nrfx_uarte.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"


typedef enum
{
    EVENT_NOT_DEFINED,
    TX_SUCCESS,
    TX_FAILED,
    RX_SUCCESS

}rf_event;


uint8_t RF_EVENT = EVENT_NOT_DEFINED;
nrf_esb_payload_t rx_payload;

static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);

uint8_t arrays[4][4] =
{
    {0xDC, 0xDC, 0xDC, 0xDC},   // ARRAY_1
    {0x0B, 0x0B, 0x0B, 0x0B},   // ARRAY_2
    {0x0C, 0x0C, 0x0C, 0x0C},   // ARRAY_3
    {0x0D, 0x0D, 0x0D, 0x0D}    // ARRAY_4
};



uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
uint8_t base_addr_1[4] = {0xC1, 0xC1, 0xC1, 0xC1};
uint8_t addr_prefix[8] = {0x01};

uint8_t DCU_BASE[4] = { 0xDC, 0xDC, 0xDC, 0xDC };
uint8_t DCU_PREFIX[1] = { 0x01 };
uint8_t Current_Circle = 0;

#if LEDS_NUMBER <= 2
#error "Board is not equipped with enough amount of LEDs"
#endif

////#define TASK_DELAY        200           /**< Task delay. Delays a LED0 task for 200 ms */
////#define TIMER_PERIOD      1000          /**< Timer period. LED1 timer will expire after 1000 ms */

TaskHandle_t  listentask;
TaskHandle_t  sendtask;
TaskHandle_t  ins_task;




/**@brief LED0 task entry function.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the task.
 */
//static void led_toggle_task_function (void * pvParameter)
//{
//    UNUSED_PARAMETER(pvParameter);
//    while (true)
//    {
//        bsp_board_led_invert(BSP_BOARD_LED_0);

//        /* Delay a task for a given number of ticks */
//        vTaskDelay(TASK_DELAY);

//        /* Tasks must be implemented to never return... */
//    }
//}

///**@brief The function to call when the LED1 FreeRTOS timer expires.
// *
// * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
// */
//static void led_toggle_timer_callback (void * pvParameter)
//{
//    UNUSED_PARAMETER(pvParameter);
//    bsp_board_led_invert(BSP_BOARD_LED_1);
//}

QueueHandle_t rfQueue;



void  nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
                                        NRF_LOG_DEBUG("TX SUCCESS EVENT");
                                        NRF_LOG_FLUSH();
                
                                        RF_EVENT = TX_SUCCESS;                                       

                                        break;

        case NRF_ESB_EVENT_TX_FAILED:
                                        NRF_LOG_DEBUG("TX FAILED EVENT");
                                        NRF_LOG_FLUSH();
                                        RF_EVENT = TX_FAILED;
                                     

                                        break;

        case NRF_ESB_EVENT_RX_RECEIVED:
                                        if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
                                        {
                                              NRF_LOG_DEBUG("RX SUCCESS EVENT");
                                              NRF_LOG_FLUSH();
                                              RF_EVENT = RX_SUCCESS;    
                                              
                                              xQueueSendFromISR(
                                                            rfQueue,
                                                            &rx_payload,
                                                            &xHigherPriorityTaskWoken
                                              );

                                              portYIELD_FROM_ISR(xHigherPriorityTaskWoken);                                               
                                        }  

                                        break;
    }
}


uint32_t esb_init( void )
{

    uint32_t err_code;

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_count         = 1;
    nrf_esb_config.retransmit_delay         = 600;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack       = false;


    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(arrays[Current_Circle]);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 1);
    VERIFY_SUCCESS(err_code);

    return err_code;
}

uint32_t set_slave_adress(uint8_t slaveid,uint8_t *base_address)
{

    uint32_t err_code;

    err_code = nrf_esb_set_base_address_0(base_address);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(&slaveid, 1);
    VERIFY_SUCCESS(err_code);
    nrf_esb_flush_tx();
    nrf_esb_flush_rx();

    return err_code;
}

void sendDataToNextSlave(void)
{      
        
        if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
        {
              nrf_delay_us(50000); //to send the data 
              set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
              nrf_esb_start_rx();
        }
        else
        {
               NRF_LOG_INFO("FAILED to send the data");
        }  
}

void send_INS_packet( void )
{
      nrf_esb_stop_rx();
      memcpy( tx_payload.data, addr_prefix, 1 );
      memcpy( tx_payload.data + 1 , "01000001",sizeof("01000001") );
      tx_payload.length =  sizeof("01000001");
      
      set_slave_adress(DCU_PREFIX[0], DCU_BASE); 
    
      sendDataToNextSlave();
}




#if 0
void send_INS_task(void *argc)
{
    send_INS_packet();
    vTaskDelete(NULL);  
}


void listen_task(void *argc)
{
    uint32_t err_code;

    set_slave_adress(addr_prefix[0], arrays[Current_Circle]);
    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);

    while (1)
    {      
        nrf_delay_us(100);  
    }
}

void send_task(void *argc)
{
    while (1)
    {
        if ( RF_EVENT == RX_SUCCESS )  
        {
            RF_EVENT = EVENT_NOT_DEFINED; 

            set_slave_adress(DCU_PREFIX[0], DCU_BASE);
            memset( &tx_payload, 0, sizeof(tx_payload) );
            memcpy(tx_payload.data, "send_task", 9);
            tx_payload.length = 9;
            if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
            {
                nrf_delay_us(50000);
            }
        }
        nrf_delay_us(100);  
    }
}


#endif

static TaskHandle_t task1_led0;
static TaskHandle_t task2_led1;
static TaskHandle_t print_task;

void RF_Print_Task(void *pvParameters)
{
    nrf_esb_payload_t received_payload;

    while (1)
    {
        if (xQueueReceive(rfQueue,
                          &received_payload,
                          portMAX_DELAY) == pdPASS)
        {
            NRF_LOG_INFO("Received Data:");

            for (int i = 0; i < received_payload.length; i++)
            {
                NRF_LOG_INFO("Byte %d: %02X", i,
                              received_payload.data[i]);
            }

            NRF_LOG_FLUSH();
        }
    }
}

void task1( void *argc )
{
    while (true)
    {
      bsp_board_led_invert(BSP_BOARD_LED_0);

      vTaskDelay(200);
    }

}


void task2( void *argc )
{
    while (true)
    {
      bsp_board_led_invert(BSP_BOARD_LED_1);
      vTaskDelay(200);
    }

}


int main(void)
{
    ret_code_t err_code;

    
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

      err_code = NRF_LOG_INIT(NULL);  
      APP_ERROR_CHECK(err_code);

      //NRF_LOG_DEFAULT_BACKENDS_INIT();

      err_code = esb_init();
      APP_ERROR_CHECK(err_code);

      NRF_LOG_DEBUG("RTOS code started");


      err_code = nrf_esb_start_rx();
      APP_ERROR_CHECK(err_code);

    bsp_board_init(BSP_INIT_LEDS);

    rfQueue = xQueueCreate(
            5,                          // 5 payloads buffer
            sizeof(nrf_esb_payload_t)    // Full payload structure
          );

    xTaskCreate(task1, "task1", configMINIMAL_STACK_SIZE + 200, NULL, 2, task1_led0);
    xTaskCreate(task2, "task2", configMINIMAL_STACK_SIZE + 200, NULL, 2, task2_led1);
    xTaskCreate(RF_Print_Task, "print_task", configMINIMAL_STACK_SIZE + 200, NULL, 2, print_task);
    

    
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    vTaskStartScheduler();

    while (true)
    {
        
    }
}

/**
 *@}
 **/
