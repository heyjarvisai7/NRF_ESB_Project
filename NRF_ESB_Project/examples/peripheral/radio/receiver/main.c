/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
*
* @defgroup nrf_dev_radio_rx_example_main main.c
* @{
* @ingroup nrf_dev_radio_rx_example
* @brief Radio Receiver example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO peripheral.
*
*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "radio_config.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"
#include "nrf_error.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

static uint32_t                   packet;              /**< Packet to transmit. */

/**@brief Function for initialization oscillators.
 */
void clock_initialization()
{
    /* Start 16 MHz crystal oscillator */
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    /* Start low frequency crystal oscillator for app_timer(used by bsp)*/
    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    //while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    //{
    //    // Do nothing.
    //}
}


/**@brief Function for reading packet.
 */

void set_receiver_slave(uint8_t slaveid)
{

          //  #define SLAVE_ID 0x02  // Example slave ID
          //uint8_t slave

    NRF_RADIO->PREFIX0 =
        ((uint32_t)swap_bits(0x00) << 24)  // pipe3 (unused)
      | ((uint32_t)swap_bits(0x00) << 16)  // pipe2 (unused)
      | ((uint32_t)swap_bits(0x00) << 8)   // pipe1 (unused)
      | ((uint32_t)swap_bits(slaveid) << 0); // pipe0 = slave ID

}

uint32_t read_packet()
{
    uint32_t result = 0;




      NRF_RADIO->RXADDRESSES = 1 << 0; // Only pipe0 enabled
    NRF_RADIO->EVENTS_READY = 0U;
    // Enable radio and wait for ready
    NRF_RADIO->TASKS_RXEN = 1U;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;

    // Wait for end of packet or buttons state changed
    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    if (NRF_RADIO->CRCSTATUS == 1U)
    {
        result = packet;
    }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
    return result;
}


/**
 * @brief Function for application main entry.
 * @return 0. int return type required by ANSI/ISO standard.
 */

 void send_packet()
{
    // send the packet:
    NRF_RADIO->EVENTS_READY = 0U;
    NRF_RADIO->TASKS_TXEN   = 1;

    while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END  = 0U;
    NRF_RADIO->TASKS_START = 1U;

    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    uint32_t err_code = bsp_indication_set(BSP_INDICATE_SENT_OK);
    NRF_LOG_INFO("The packet was sent");
    APP_ERROR_CHECK(err_code);

    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
}

int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    clock_initialization();

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);

    // Set radio configuration parameters
    radio_configure();
    NRF_RADIO->PACKETPTR = (uint32_t)&packet;

    err_code = bsp_indication_set(BSP_INDICATE_USER_STATE_OFF);
    NRF_LOG_INFO("Radio receiver example started.");
    NRF_LOG_INFO("Wait for first packet");
    APP_ERROR_CHECK(err_code);
    NRF_LOG_FLUSH();

    while (true)
    {

      //  set_receiver_slave(0x02);
        uint32_t received = read_packet();
        //packet = received;
        err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
        NRF_LOG_INFO("Packet was received");
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("The contents of the package is %u", (unsigned int)received);
        
        set_target_slave(0x04);
        
        send_packet(); 
        NRF_LOG_INFO("The contents of the package was %u", (unsigned int)packet);

        uint32_t received1 = read_packet();
        NRF_LOG_INFO("The contents of the package is %u", (unsigned int)received1);

        packet = received1;

        set_receiver_slave(0x02);

        send_packet(); 
        NRF_LOG_INFO("The contents of the package was %u", (unsigned int)packet);

       // packet = 0;
        //NRF_LOG_FLUSH();
        //        uint32_t received1 = read_packet();

        //err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
        //NRF_LOG_INFO("Packet was received");
        //APP_ERROR_CHECK(err_code);

        //NRF_LOG_INFO("The contents of the package is %u", (unsigned int)received1);
        NRF_LOG_FLUSH();

    }
}

/**
 *@}
 **/
