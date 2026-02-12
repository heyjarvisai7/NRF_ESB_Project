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
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_delay.h"
#include "app_util.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"




/*
 *        MACROS
 */

#define     MAX_CIRCLE                        4
#define     ARRRAY_SIZE                       4
#define     PACKET_HEADER_SIZE                sizeof(PACKET_HEADER)
#define     PACKET_INS_SIZE                   sizeof(PACKET_INS)

#define     POS_HEADER                        0
#define     POS_INS_PACKET                     PACKET_HEADER_SIZE


#define     POS_PACKET_TYPE                   0
#define     POS_DIRECTION                     sizeof(headeer.packet_type)
#define     POS_LENGTH                        POS_DIRECTION + sizeof(headeer.Direction) 
#define     POS_CIRCLE_ARRAY                  POS_LENGTH + sizeof(headeer.length)
#define     POS_DIRTY_FLAG                    POS_CIRCLE_ARRAY + sizeof(headeer.circle_array)
#define     POS_PACKETNO                      POS_DIRTY_FLAG + sizeof(header.dirtyflag)
#define     POS_RESERVED                      POS_PACKETNO + sizeof(header.packetNo)

#define     POS_SERIAL_NO                     POS_RESERVED + sizeof(headeer.reserved) 
#define     POS_PATH                          POS_SERIAL_NO + sizeof(ins_Packet.serial_no)


#define     MAX_NODES                         255
#define     MIN_NODES                         0

#define     INS_PACKET_ARRAY_SIZE             50
#define     SIZE_OF_SERIALNUM                 8
#define     NOT_FOUND                         0xFF

#define     RX_QUEUE_SIZE                     7
#define     APP_BUF_SIZE                      500 


/*
 *        ENUMS
 */

typedef enum
{
    BACKWORD,
    FORWARD

}DIRECTION;

typedef enum 
{
    DATA_PACKET = 1,
    PING_PACKET,
    INS_PACKET

}DATA_TYPE;



/*
 *        STRUCTURES
 */

typedef struct __attribute__((packed)) Packet_Header
{
      uint8_t packet_type; 
      uint8_t Direction;
      uint16_t length;
      uint8_t circle_array[MAX_CIRCLE];
      uint8_t dirtyflag;
      uint8_t packetNo;
      uint8_t reserved;

}PACKET_HEADER;

typedef struct __attribute__((packed)) Ins_Packet
{
	uint8_t serial_no[SIZE_OF_SERIALNUM];
        uint8_t path[4];

}PACKET_INS;

typedef struct
{
    uint8_t  data[APP_BUF_SIZE];
    uint16_t length;
} rx_packet_t;



/*
 *        PROTOTYPES
 */

void fillHeader(uint8_t packt_Type, uint8_t direction, uint8_t length, uint8_t *path, uint8_t dirtFlag, uint8_t reserved );
void sendDataBidirectional(uint8_t direction);
uint32_t set_slave_adress(uint8_t slaveid,uint8_t *base_address);
uint8_t matchSerialNo(uint8_t index, uint8_t *seralNo);
void storePath( void );
void sendDataToNextSlave(void);




/*
 *        VARIABLES
 */

PACKET_HEADER headeer;
PACKET_INS ins_Packet;
static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);

static nrf_esb_payload_t        rx_payload;

PACKET_INS PATH_ARRAY[INS_PACKET_ARRAY_SIZE];
uint8_t path_Index = 0;
uint8_t loop_Index = 0;

uint8_t PACKET = 0;
uint8_t RX_SUCESS_FLAG = 0;
uint8_t volatile Send_Data_To_Nxt_Slave = 0;

uint8_t Nxt_Circle_Base_Addr[4] = {0x0A, 0x0A, 0x0A, 0x0A};

uint8_t  open_request[20] = {0xE7,0xE6, 0x00,0x00,0x00,0x09,0x00,0x00,0x7E, 0xA0, 0x07, 0x03,0x21, 0x93, 0x0F, 0x01, 0x7E};


uint8_t  open_request1[20] = {0x7E, 0xA0, 0x07, 0x03,0x21, 0x93, 0x0F, 0x01, 0x7E};
uint8_t  addr[4] = {0xE7,0xE6, 0x00,0x00};

uint8_t server_SerialNo[8] = { 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01 };

uint8_t base_addr_0[4] = {0xDC, 0xDC, 0xDC, 0xDC};
uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
uint8_t addr_prefix[1] = {0x01};

volatile rx_packet_t rx_queue[RX_QUEUE_SIZE];
volatile uint8_t rx_head    =   0;
volatile uint8_t rx_tail    =   0;
volatile uint8_t buf_count  =   0;


void storePath( void )
{
      memcpy( &PATH_ARRAY[path_Index].serial_no, rx_payload.data + sizeof(headeer), SIZE_OF_SERIALNUM);
      memcpy( &PATH_ARRAY[path_Index].path, &rx_payload.data[POS_CIRCLE_ARRAY],ARRRAY_SIZE);
   
      path_Index++;
}

uint8_t matchSerialNo(uint8_t index, uint8_t *serialNo)
{
      uint8_t loop_index;
      
      for(loop_index = index; loop_index < INS_PACKET_ARRAY_SIZE ; loop_index++ )
      {

          if ( memcmp( PATH_ARRAY[loop_index].serial_no, serialNo, SIZE_OF_SERIALNUM ))
          {
              loop_Index = loop_index;
              return loop_index;
          }
      }
      return  NOT_FOUND;
}

void fillHeader(uint8_t packt_Type, uint8_t direction, uint8_t length, uint8_t *path, uint8_t dirtFlag, uint8_t reserved )
{
    headeer.packet_type         =       packt_Type;
    headeer.Direction           =       direction;
    headeer.length              =       length;
    memcpy(headeer.circle_array, path, MAX_CIRCLE);
    headeer.dirtyflag           =       dirtFlag;
    headeer.reserved            =       reserved;
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

void sendDataBidirectional(uint8_t direction)
{
    if ( direction == FORWARD )
    {
          /*
           * Send to NODES
           */
        nrf_delay_us(50000); //to send the data 
        set_slave_adress(PATH_ARRAY[0].path[0], Nxt_Circle_Base_Addr);
        sendDataToNextSlave();
    }
    else
    {
          /*
           * Send to server 
           */

           NRF_LOG_INFO("Sending to server...");
           NRF_LOG_FLUSH();
    }
}

void sendDataToNextSlave(void)
{
        Send_Data_To_Nxt_Slave = 0; 
       
        //nrf_esb_stop_rx();
        
        
        if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
        {
              nrf_delay_us(50000); //to send the data 
              set_slave_adress(addr_prefix[0], base_addr_0); // DCU lisitining
              nrf_esb_start_rx();
        }
        else
        {
               NRF_LOG_INFO("FAILED to send the data");
               /*
                * If one path files means it will search another path for that same serial number 
                */
               if( loop_Index == ( INS_PACKET_ARRAY_SIZE - 1 ) ) 
               {
                    loop_Index = 0;

                    /*
                     * Need to send next circle active node 
                     */
                    
               }

               if ( !matchSerialNo( loop_Index, server_SerialNo ))
               {
                    sendDataBidirectional( FORWARD );
               }
               else
               {
                    /*
                     * Next path for that same serial number not found 
                     */

                     NRF_LOG_INFO("Next path for that same serial number not found ");
               }

        }
     
}

#if 0  // for debugging 

bool rx_queue_push(uint8_t *in_data, uint16_t in_len)
{
    NRF_LOG_INFO("buf_count %d",buf_count);
    NRF_LOG_INFO("RX_QUEUE_SIZE %d",RX_QUEUE_SIZE);
    NRF_LOG_FLUSH();
   
    if ( buf_count >= RX_QUEUE_SIZE)
    {
        NRF_LOG_INFO("PUSHED FAIL");
        NRF_LOG_FLUSH();
        return false;
    }
    memcpy((void *)rx_queue[rx_head].data, in_data, in_len);
    rx_queue[rx_head].length = in_len;

    buf_count++;
    NRF_LOG_INFO("rx_head-- %d",rx_head);
    NRF_LOG_INFO("rx_tail-- %d",rx_tail);
 //   if((rx_head + 1 % RX_QUEUE_SIZE) != rx_tail)
    {
        rx_head = (rx_head + 1) % RX_QUEUE_SIZE;
        NRF_LOG_INFO("++");
        NRF_LOG_FLUSH();
    }
    
    nrf_esb_flush_rx();

    NRF_LOG_INFO("DATA PUSHED");
    NRF_LOG_INFO("rx_head %d",rx_head);
    NRF_LOG_INFO("buf_count %d",buf_count);
    NRF_LOG_FLUSH();

    for(int j = 0; j < RX_QUEUE_SIZE; j++)
    {
        NRF_LOG_INFO("Length of %d is : %d", j, rx_queue[j].length);
        NRF_LOG_FLUSH();
    }

    return true;
}


bool rx_queue_pop(void)
{
    if ( buf_count <= 0 )
    {
        NRF_LOG_INFO("POPED FAIL");
        NRF_LOG_FLUSH();
        return false;   
    }

    // Get oldest packet
    //pkt = &rx_queue[rx_tail];

    // Optional: clear memory (true delete)
    memset((void *)&rx_queue[rx_tail], 0, sizeof(rx_queue[rx_tail]));
    //rx_queue[rx_tail].len = 0;

    // Move tail forward
    rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE;

    buf_count--;

    NRF_LOG_INFO("DATA POPED");
    NRF_LOG_INFO("rx_tail %d",rx_tail);
    NRF_LOG_INFO("buf_count %d",buf_count);
    NRF_LOG_FLUSH();

    for(int j = 0; j < RX_QUEUE_SIZE; j++)
    {
        NRF_LOG_INFO("Length of %d is : %d", j, rx_queue[j].length);
        NRF_LOG_FLUSH();
    }
    return true;
}

#else 

bool rx_queue_push(uint8_t *in_data, uint16_t in_len)
{
    if ( buf_count >= RX_QUEUE_SIZE)
    {
        NRF_LOG_INFO("PUSHED FAIL");
        NRF_LOG_FLUSH();
        return false;
    }
    memcpy((void *)rx_queue[rx_head].data, in_data, in_len);
    rx_queue[rx_head].length = in_len;

    buf_count++;
   
    rx_head = (rx_head + 1) % RX_QUEUE_SIZE;
    
    nrf_esb_flush_rx();
    nrf_esb_flush_tx();

    NRF_LOG_INFO("DATA PUSHED");
    //NRF_LOG_FLUSH();

    return true;
}


bool rx_queue_pop(void)
{
    if ( buf_count <= 0 )
    {
        NRF_LOG_INFO("POPED FAIL");
        NRF_LOG_FLUSH();
        return false;   
    }

    memset((void *)&rx_queue[rx_tail], 0, sizeof(rx_queue[rx_tail]));
   
    rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE;

    buf_count--;

    NRF_LOG_INFO("DATA POPED");
    NRF_LOG_FLUSH();

    return true;
}

#endif



void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
	switch (p_event->evt_id)
	{
              case    NRF_ESB_EVENT_TX_SUCCESS:

                      NRF_LOG_DEBUG("TX SUCCESS EVENT");                      

              break;

              case    NRF_ESB_EVENT_TX_FAILED:

                      NRF_LOG_DEBUG("TX FAILED EVENT");
                     
                      (void) nrf_esb_flush_tx();
                      (void) nrf_esb_start_tx();

              break;


              case    NRF_ESB_EVENT_RX_RECEIVED:
                      

                      if (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
                      {

                           /*Need to store in comming packets*/
                           RX_SUCESS_FLAG = 1;
                     
                            
                           switch ( rx_payload.data[POS_PACKET_TYPE] )
                           {
                                case    PING_PACKET :
                                      
                                            PACKET   =   PING_PACKET;
                                break;

                                case    INS_PACKET  :
                                      
                                            PACKET   =   INS_PACKET;
                                break;

                                case    DATA_PACKET :
                                    
                                            PACKET   =    DATA_PACKET;   
                                            rx_queue_push(rx_payload.data, rx_payload.length);
                                break;
                           } 
                           NRF_LOG_INFO("rx_payload.length : %d",rx_payload.length);                     
                      }  

                break;
	}
}


void clocks_start( void )
{
	NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
	NRF_CLOCK->TASKS_HFCLKSTART = 1;

	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


void gpio_init( void )
{
	nrf_gpio_range_cfg_output(8, 15);
	bsp_board_init(BSP_INIT_LEDS);
}



uint32_t esb_init( void )
{
	uint32_t err_code;
	
	
	nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
	nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
	nrf_esb_config.retransmit_delay         = 600;
	nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
	nrf_esb_config.event_handler            = nrf_esb_event_handler;
	nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
	nrf_esb_config.selective_auto_ack       = false;

	err_code = nrf_esb_init(&nrf_esb_config);

	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_base_address_0(base_addr_0);
	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_base_address_1(base_addr_1);
	VERIFY_SUCCESS(err_code);

	err_code = nrf_esb_set_prefixes(addr_prefix, 1);
	VERIFY_SUCCESS(err_code);

	return err_code;
}

        uint8_t insDone = 0;

int main(void)
{
	ret_code_t err_code;

	struct Packet_Header hdr;
	uint32_t nextBytes = 0;
        uint8_t matchIndex = 0;

	gpio_init();

	err_code = NRF_LOG_INIT(NULL);  
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEFAULT_BACKENDS_INIT();

	clocks_start();

	err_code = esb_init();
	APP_ERROR_CHECK(err_code);

	NRF_LOG_DEBUG("DCU code started");
        NRF_LOG_FLUSH();

        err_code = nrf_esb_start_rx();
        APP_ERROR_CHECK(err_code);

        while( true )
        {
           NRF_LOG_FLUSH();

           switch (  PACKET )
           {
              
                NRF_LOG_FLUSH();

                case      DATA_PACKET: 
                            if ( buf_count != 0 )
                            {
                                  sendDataBidirectional( BACKWORD );
                                  rx_queue_pop();
                                  PACKET = rx_queue[rx_tail].data[POS_PACKET_TYPE];
                            }
                break;

                case      PING_PACKET: // do whatever
                
                break;

                case      INS_PACKET: 

                            storePath();
                          //  rx_queue_pop();
                            insDone = 1;
                            PACKET = rx_queue[rx_tail].data[POS_PACKET_TYPE];
                break;

           }                        
            

            if ( path_Index != 0 && insDone  == 1 ) // need to start the sending req/data packet 
            {
                  /*
                   * This if PASS means something path is present at DCU 
                   * Need to check the serial number comming from SERVER with DCU having serial no
                   * If match is there means then the SERVER request is send through the PATH to that SERIAL number node
                   * Then same in that path only RESPONSE will come  
                   */
                   insDone = 0;
                   matchIndex = matchSerialNo( 0, server_SerialNo );
                  if( matchIndex != 0xFF )
                  {
                      /*
                       * Actually this header filling data need to take from UART(GSM)
                       */
                      nrf_esb_stop_rx();
                      fillHeader(DATA_PACKET, FORWARD, sizeof(open_request1), PATH_ARRAY[matchIndex].path, 0, 0);
                      memcpy(tx_payload.data, &headeer, sizeof(headeer));
                      memcpy(tx_payload.data + sizeof(headeer), open_request1, sizeof(open_request1));

                      tx_payload.length = ( PACKET_HEADER_SIZE + (open_request1[2] + 2));

                      sendDataBidirectional( FORWARD );
                  } 
            }
            
        }	
}



