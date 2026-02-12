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


void pingPacket(void);
uint32_t esb_init1( void );
uint32_t esb_init( void );
uint32_t set_slave_adress(uint8_t slaveid,uint8_t *base_address);
void scan_device(uint8_t length);
void sendDataToNextSlave(void);
void reRouting(void);
void doDiagnosticTest(void);
static void diagnostic_tx(uint8_t neighbour, uint8_t circle);

void sendDataBidirectional(uint8_t direction);

uint8_t led_nr;
uint8_t Fwrd_Dirct = 0;
uint8_t Rvsr_Dirct = 0;

nrf_esb_payload_t rx_payload;

static nrf_esb_payload_t        tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);

/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */

#define START_BYTE                            PACKET_HEADER_SIZE

#define     PACKET_HEADER_SIZE                sizeof(PACKET_HEADER)
#define     PACKET_INS_SIZE                   sizeof(PACKET_INS)

#define     POW_CNTRL_PIN                     7
#define     ARRRAY_SIZE                       4
#define     MAX_CIRCLE                        4
#define     MAX_NEIGHBORS                     10
#define     MIN_CIRCLE                        0

#define     POS_PACKET_TYPE                   0
#define     POS_DIRECTION                     sizeof(header.packet_type)
#define     POS_LENGTH                        POS_DIRECTION + sizeof(header.Direction) 
#define     POS_CIRCLE_ARRAY                  POS_LENGTH + sizeof(header.length)
#define     POS_DIRTY_FLAG                    POS_CIRCLE_ARRAY + sizeof(header.circle_array)
#define     POS_RESERVED                      POS_DIRTY_FLAG + sizeof(header.dirtyflag)

//#define     POS_SERIAL_NO                     POS_RESERVED + sizeof(header.reserved) 
//#define     POS_PATH                          POS_SERIAL_NO + sizeof(ins_Packet.serial_no)

#define check_direction(arrptr) \
    (((arrptr)[POS_DIRECTION]  == 1) ? (Fwrd_Dirct = 1) : (Rvsr_Dirct = 1))


#define     MAX_NODES                         255
#define     MIN_NODES                         0  

#define     RX_QUEUE_SIZE                     7
#define     APP_BUF_SIZE                      500 


typedef enum 
{
    DATA_PACKET = 1,
    PING_PACKET,
    INS_PACKET

}DATA_TYPE;

typedef enum 
{    
    BACKWORD,
    FORWARD

}DIRECTION;




typedef struct
{
    uint16_t node_id;       // Neighbor ID
    int8_t   rssi;          // Signal strength
    //uint8_t  hop_count;     // Distance to sink
    //uint32_t last_seen;     // Timestamp
    //uint8_t  valid;         // Entry active
    //uint8_t dirtyflag;
} neighbor_t;
 
neighbor_t Nxt_neighbor_table[MAX_NEIGHBORS];
neighbor_t Prev_neighbor_table[MAX_NEIGHBORS];


typedef struct
{
    uint8_t node_id;
    uint8_t circle_no;
    //uint8_t  hop_count;
    //uint8_t  seq;
} adv_packet_t;

adv_packet_t advertisment_pcket;



typedef struct __attribute__((packed)) Packet_Header
{
      uint8_t packet_type; 
      uint8_t Direction;
      uint16_t length;
      uint8_t circle_array[MAX_CIRCLE];
      uint8_t dirtyflag;
      uint8_t reserved;
}PACKET_HEADER;

PACKET_HEADER Packet_Header;


typedef struct __attribute__((packed)) Ins_Packet
{
	uint8_t serial_no[8];
        uint8_t path[4];

}PACKET_INS;

PACKET_INS ins_Packet;

typedef struct
{
    uint8_t  data[APP_BUF_SIZE];
    uint16_t length;
} rx_packet_t;




uint8_t arrays[MAX_CIRCLE][ARRRAY_SIZE] =
{
    {0x0A, 0x0A, 0x0A, 0x0A},   // ARRAY_1
    {0x0B, 0x0B, 0x0B, 0x0B},   // ARRAY_2
    {0x0C, 0x0C, 0x0C, 0x0C},   // ARRAY_3
    {0x0D, 0x0D, 0x0D, 0x0D}    // ARRAY_4
};



uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
uint8_t base_addr_1[4] = {0xC1, 0xC1, 0xC1, 0xC1};
uint8_t addr_prefix[8] = {0x20};

uint8_t DCU_BASE[4] = { 0xDC, 0xDC, 0xDC, 0xDC };

uint8_t DCU_PREFIX[1]= { 0x01 };

volatile rx_packet_t rx_queue[RX_QUEUE_SIZE];
volatile uint8_t rx_head    =   0;
volatile uint8_t rx_tail    =   0;
volatile uint8_t buf_count  =   0;

uint8_t PACKET = 0;


uint8_t received_data = 0;

uint8_t selected_pipe = 0;
uint8_t volatile Mater_Data_received = 0;
uint8_t volatile Mater_Data_Sent = 0;
uint8_t volatile Slave_Data_received = 0;
uint8_t volatile Slave_Data_Sent = 0;
uint8_t itr ;
uint16_t length = 0;
uint8_t i = 0;
uint8_t sent_bytes_count = 0;
nrf_esb_payload_t rx_payload;
uint8_t Circle_No = 0;
uint8_t Current_Circle = 0;
uint8_t volatile Send_Data_To_Nxt_Slave = 0;

uint8_t uartbuff[2048];
uint8_t Ack_Txing = 0;
uint8_t Actd[15];
uint8_t *circlearr;
uint8_t Nxt_table_index;
uint8_t Prev_table_index;
uint8_t neighbour_no;

uint8_t RX_SUCESS_FLAG = 0;
uint8_t testSlave, doo, sendINS = 0; 

uint8_t circlearr1[500];


struct Packet_Header header;
neighbor_t *Nxt_neighbor = &Nxt_neighbor_table[0];
neighbor_t *Prev_neighbor = &Prev_neighbor_table[0];




void sort_neighbors_by_rssi(neighbor_t *table, int n)
{
    for (int i = 1; i < n; i++)
    {
        neighbor_t key = table[i];
        int j = i - 1;

        while (j >= 0 && table[j].rssi < key.rssi)
        {
            table[j + 1] = table[j];
            j--;
        }
        table[j + 1] = key;
    }
}


void Construct_DLMS_Packet(void)
{
   uint32_t err_code = 0;
  if(rx_queue[rx_tail].data[POS_CIRCLE_ARRAY+Current_Circle] == addr_prefix[0])
  {
   memcpy(&header, rx_payload.data,sizeof(header));
    if(rx_payload.data[START_BYTE] == 0x7E)
    {
            if((rx_payload.data[START_BYTE + 2]))
            {
                    length = (((rx_payload.data[START_BYTE +1] & 0x07) << 8) | rx_payload.data[START_BYTE +2]);
                    length += 2;
            }
            i = 0;

    }
    if(length  > NRF_ESB_MAX_PAYLOAD_LENGTH - START_BYTE)
    {
      //received = 0;

      memcpy(uartbuff+sent_bytes_count,rx_payload.data + START_BYTE,rx_payload.length - START_BYTE);
     // tx_payload.length = rx_payload.length;
      sent_bytes_count += NRF_ESB_MAX_PAYLOAD_LENGTH - START_BYTE;
      length -= NRF_ESB_MAX_PAYLOAD_LENGTH -START_BYTE;
    }
    else
    {
      //memset(tx_payload.data,'/0',strlen(tx_payload.data));
     // memcpy(tx_payload.data+sent_bytes_count,rx_payload.data,rx_payload.length);
      memcpy(uartbuff+sent_bytes_count,rx_payload.data + START_BYTE,rx_payload.length - START_BYTE);
    //  tx_payload.length = rx_payload.length;
      sent_bytes_count += rx_payload.length - START_BYTE;
      if(uartbuff[sent_bytes_count-1] == 0x7E)
      {
           //if(header.length == sent_bytes_count)
           //{
               //sent_bytes_count = 0;
               header.Direction = BACKWORD;// reverse direction;
              for (uint32_t i = 0; i < sent_bytes_count; i++)
              {
                  while (true)
                  {
                      err_code = app_uart_put(uartbuff[i]);

                      if (err_code == NRF_SUCCESS)
                      {
                          break;  // byte sent to FIFO
                      }
                      else if (err_code == NRF_ERROR_BUSY ||
                               err_code == NRF_ERROR_NO_MEM)
                      {
                          // wait until space available
                          // just retry
                      }
                      else
                      {
                          NRF_LOG_ERROR("UART error 0x%x", err_code);
                          break; // do NOT hard fault
                      }
                  }
              }
                sent_bytes_count = 0;
                if (uartbuff[sent_bytes_count - 1] == '\r')
                {
                    while (app_uart_put('\n') == NRF_ERROR_BUSY);
                }
           //}
      }   
      else
      {
        
         Mater_Data_received = 0;//when haft data come
      }
      
    }
  }
                    
  NRF_LOG_FLUSH();
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


void  nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:

                NRF_LOG_DEBUG("TX SUCCESS EVENT");
                NRF_LOG_FLUSH();
                Ack_Txing = 1;
                NRF_LOG_FLUSH();

        break;

        case NRF_ESB_EVENT_TX_FAILED:

                NRF_LOG_DEBUG("TX FAILED EVENT");
                NRF_LOG_FLUSH();
                if(tx_payload.data[POS_PACKET_TYPE] == PING_PACKET)
                {
                   neighbour_no++;
                }
                else
                {   // if rx packet is data packet
                    //reRouting(); // need to uncomment 
                }
                //NRF_LOG_FLUSH();

        break;

        case NRF_ESB_EVENT_RX_RECEIVED:

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

                       
                    }  

          break;
    }
}

void reRouting(void)
{
      tx_payload.data[POS_DIRTY_FLAG] = 1;
   
       if(tx_payload.data[POS_DIRECTION] == FORWARD)
       {
          if(Nxt_neighbor < &Nxt_neighbor_table[MAX_NEIGHBORS]) 
          {
             Nxt_neighbor++;
             if(Nxt_neighbor->node_id == 0)
             {
                Nxt_neighbor = &Nxt_neighbor_table[0];
             }
          }
          else
          {
             Nxt_neighbor = &Nxt_neighbor_table[0];
          }
          tx_payload.data[POS_CIRCLE_ARRAY+ Current_Circle + 1] = Nxt_neighbor->node_id;
 
          set_slave_adress(Nxt_neighbor->node_id, arrays[Current_Circle + 1]);
          sendDataToNextSlave();         
    
       }
 
       else
       {
          if(Prev_neighbor < &Prev_neighbor_table[MAX_NEIGHBORS]) 
          {
             Prev_neighbor++;
             if(Prev_neighbor->node_id == 0)
             {
                Prev_neighbor = &Prev_neighbor_table[0];
             }
          }
          else
          {
             Prev_neighbor = &Prev_neighbor_table[0];
          }
          tx_payload.data[POS_CIRCLE_ARRAY+ Current_Circle - 1] = Prev_neighbor->node_id;

          set_slave_adress(Prev_neighbor->node_id, arrays[Current_Circle - 1]);
          sendDataToNextSlave();
       }
}

void pingPacket(void)
{
      
          if ( rx_payload.data[POS_PACKET_TYPE] == PING_PACKET && rx_payload.data[POS_LENGTH] == 0 )
          {
                memcpy(&advertisment_pcket,rx_payload.data + sizeof(header), sizeof(advertisment_pcket));
                if(rx_payload.rssi > 0 && Nxt_table_index < MAX_NEIGHBORS && advertisment_pcket.circle_no == Current_Circle +1)
                {
                      Nxt_neighbor_table[Nxt_table_index].node_id = advertisment_pcket.node_id;
                      Nxt_neighbor_table[Nxt_table_index].rssi = rx_payload.rssi;
                      Nxt_table_index++;
                }
                if(rx_payload.rssi > 0 && Prev_table_index < MAX_NEIGHBORS && advertisment_pcket.circle_no == Current_Circle - 1)
                {
                      Prev_neighbor_table[Prev_table_index].node_id = advertisment_pcket.node_id;
                      Prev_neighbor_table[Prev_table_index].rssi = rx_payload.rssi;
                      Prev_table_index++;
                }
                neighbour_no++;
                Ack_Txing = 0;

          }
          
          if ( rx_payload.data[POS_PACKET_TYPE] == PING_PACKET && rx_payload.data[POS_LENGTH] == 1 )
          {
                advertisment_pcket.circle_no = Current_Circle;
                advertisment_pcket.node_id = addr_prefix[0];
                memset(&header,0,sizeof(header));
                header.packet_type = PING_PACKET;
                
                if(rx_payload.data[(POS_CIRCLE_ARRAY+ Current_Circle) - 1] != 0 && (POS_CIRCLE_ARRAY+ Current_Circle) != 0) 
                {
                      header.Direction = BACKWORD;
                }
                else
                {
                      header.Direction = FORWARD;
                }
                memcpy(tx_payload.data,&header,sizeof(header));
                memcpy(tx_payload.data + sizeof(header), &advertisment_pcket, sizeof(advertisment_pcket));
                tx_payload.length = sizeof(header) + sizeof(advertisment_pcket);
                //check_direction(tx_payload.data);
                //sendDataBidirectional( tx_payload.data[POS_DIRECTION] );

                nrf_delay_us(50000);
                nrf_esb_stop_rx();


                if ( tx_payload.data[POS_DIRECTION] == FORWARD )
                {
                    set_slave_adress(rx_payload.data[POS_CIRCLE_ARRAY + (Current_Circle + 1)], arrays[Current_Circle + 1]);
                }
                else
                {
                    set_slave_adress(rx_payload.data[POS_CIRCLE_ARRAY + (Current_Circle - 1)], arrays[Current_Circle - 1]);
                }

                sendDataToNextSlave();
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
    bsp_board_init(BSP_INIT_LEDS);
}

void nrf_set_txmode(void)
{
    uint32_t err_code;
     nrf_esb_config_t nrf_esb_config        = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.payload_length           = 8;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);
    //VERIFY_SUCCESS(err_code);
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



uint8_t Diagnostic_Test = 0;
#define SCAN_PREV   (1 << 0)
#define SCAN_NEXT   (1 << 1)


void doDiagnosticTest(void)
{       
      Nxt_table_index = 0;
      Prev_table_index = 0;
      uint8_t done_rx_start = 1;
      uint8_t scan_dir = 0;

    
      if (Current_Circle == MIN_CIRCLE)
      {
          scan_dir = SCAN_NEXT;
      }
      else if (Current_Circle == MAX_CIRCLE)
      {
          scan_dir = SCAN_PREV;
      }
      else
      {
          scan_dir = ( SCAN_PREV | SCAN_NEXT );
      }

      if (( scan_dir & SCAN_NEXT ) == 1  )
      {

            for(neighbour_no = MIN_NODES; neighbour_no < MAX_NODES ;)
            {

                if(Ack_Txing == 0)
                {
                      nrf_esb_stop_rx();
                      set_slave_adress( neighbour_no ,arrays[Current_Circle + 1]);
                      memset(&header,0,sizeof(header));
                      header.packet_type = 1;
                      header.length = 1;
                      header.circle_array[Current_Circle] = addr_prefix[0];
                      memcpy(tx_payload.data,&header,sizeof(header));
                      memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                      tx_payload.length = (sizeof(header) + sizeof("HAI"));
                      done_rx_start = 1;
                      if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                      {
                            nrf_delay_us(50000); //to send the data 
                            //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);

                      }
                }
                if(done_rx_start == 1 && Ack_Txing == 1)
                {
                      done_rx_start = 0;
                      nrf_esb_flush_tx();
                      nrf_esb_flush_rx();
                      set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                      nrf_esb_start_rx();

                }
                if ( RX_SUCESS_FLAG == 1 )
                {
                      RX_SUCESS_FLAG = 0;
                      pingPacket();
                }

            }
            sort_neighbors_by_rssi(Nxt_neighbor_table, Nxt_table_index - 1);
            Diagnostic_Test = 0;

      }
      if (( scan_dir & SCAN_PREV ) == 1 )
      {

            for(neighbour_no = MIN_NODES; neighbour_no < MAX_NODES ;)
            {
                  if(Ack_Txing == 0)
                  {
                        nrf_esb_stop_rx();
                        set_slave_adress( neighbour_no ,arrays[Current_Circle - 1]);
                        memset(&header,0,sizeof(header));
                        header.packet_type = 1;
                        header.length = 1;
                        header.circle_array[Current_Circle] = addr_prefix[0];
                        memcpy(tx_payload.data,&header,sizeof(header));
                        memcpy(tx_payload.data + sizeof(header),"HAI",sizeof("HAI"));
                        tx_payload.length = (sizeof(header) + sizeof("HAI"));
                        done_rx_start = 1;
                        if (nrf_esb_write_payload(&tx_payload) == NRF_SUCCESS)
                        {
                              nrf_delay_us(50000); //to send the data 
                              //  set_slave_adress(addr_prefix[0],arrays[Current_Circle]);
                              //   nrf_esb_start_rx();
                        }
                  }
                  if(done_rx_start == 1 && Ack_Txing == 1)
                  {
                        done_rx_start = 0;
                        nrf_esb_flush_tx();
                        nrf_esb_flush_rx();
                        set_slave_adress(addr_prefix[0], arrays[Current_Circle]); 
                        nrf_esb_start_rx();

                  }
                  if ( RX_SUCESS_FLAG == 1 )
                  {
                      RX_SUCESS_FLAG = 0;
                      pingPacket();
                  }

            }
            sort_neighbors_by_rssi(Prev_neighbor_table, Prev_table_index - 1);
            Diagnostic_Test = 0;
      }

      Nxt_neighbor  = &Nxt_neighbor_table[0];
      Prev_neighbor = &Prev_neighbor_table[0];

     // sendINS = 1;
}


void sendDataBidirectional(uint8_t direction)
{
        //if(Fwrd_Dirct == 1)
        if ( direction == FORWARD )
        {
              Fwrd_Dirct = 0;
              nrf_delay_us(50000);
              nrf_esb_stop_rx();
              if( rx_queue[rx_tail].data[POS_PACKET_TYPE] == DATA_PACKET )
              {
                  
                  memcpy(tx_payload.data, (void *)rx_queue[rx_tail].data, rx_queue[rx_tail].length);
                  
                  tx_payload.length  = rx_queue[rx_tail].length;
                  set_slave_adress(rx_queue[rx_tail].data[(POS_CIRCLE_ARRAY+ Current_Circle) + 1],arrays[Current_Circle + 1]);

              }
              else
              {
                  set_slave_adress(rx_payload.data[POS_CIRCLE_ARRAY + (Current_Circle + 1)], arrays[Current_Circle + 1]);
              }
              tx_payload.noack = false;
              
              
              sendDataToNextSlave();
              rx_queue_pop();
        }

        //if(Rvsr_Dirct == 1)
        else
        {
              Rvsr_Dirct = 0;
              nrf_delay_us(50000);
              nrf_esb_stop_rx();

              if( header.packet_type == DATA_PACKET )
              {
                  memcpy(tx_payload.data, (void *)rx_queue[rx_tail].data, rx_queue[rx_tail].length);
                  tx_payload.data[POS_CIRCLE_ARRAY + Current_Circle] = addr_prefix[0];
                  tx_payload.length  = rx_queue[rx_tail].length;

                  if (Current_Circle == 0)    
                  {
                      // send the response to DCU s
                      set_slave_adress(DCU_PREFIX[0], DCU_BASE); // sending to DCU
                      sendDataToNextSlave();
            
                  }

                  else  
                  {
                    set_slave_adress(rx_queue[rx_tail].data[(POS_CIRCLE_ARRAY+ Current_Circle) - 1],arrays[Current_Circle - 1]);
                  }
              }

              else
              {
                  set_slave_adress(rx_payload.data[POS_CIRCLE_ARRAY + (Current_Circle - 1)], arrays[Current_Circle - 1]);
              }


              tx_payload.noack = false;
              
              sendDataToNextSlave();
              rx_queue_pop();    
        }
}


void fillPacket(uint8_t direction, uint8_t *data, uint16_t length)
{
      nrf_delay_us(500);
      nrf_esb_stop_rx();
 
      if ( direction == FORWARD )
      {
          memcpy( tx_payload.data, data, length );
          tx_payload.length = length;
 
          set_slave_adress(tx_payload.data[POS_CIRCLE_ARRAY + Current_Circle + 1], arrays[Current_Circle + 1]);
          sendDataToNextSlave();
      }
 
      else
      {
          memcpy( tx_payload.data, data, length );
          tx_payload.length = length;
          
          if(Current_Circle == 0)
          {
              set_slave_adress(DCU_PREFIX[0], DCU_BASE);
          }
          else
          {
              set_slave_adress(tx_payload.data[POS_CIRCLE_ARRAY + Current_Circle - 1], arrays[Current_Circle - 1]);
          }
          sendDataToNextSlave();
      }
}

void sendDataToNextSlave(void)
{
        Send_Data_To_Nxt_Slave = 0; 
       
        //nrf_esb_stop_rx();
        
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

#define UART_TX_BUF_SIZE                2048                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                2048                                         /**< UART RX buffer size. */
uint8_t data_array1[UART_RX_BUF_SIZE];
uint16_t rx_index = 0;
uint8_t Uart_rx_flag = 0;
void uart_event_handle(app_uart_evt_t * p_event)
{
    uint32_t       err_code;
    switch (p_event->evt_type)
    {
       case APP_UART_DATA:
            UNUSED_VARIABLE(app_uart_get(&data_array1[rx_index]));
            Uart_rx_flag = 1;
            //memcpy(&rx_queue[rx_tail].data[PACKET_HEADER_SIZE+1], data_array1, size_t n)
            rx_index++;
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;
            
        default:
            break;
    }
}

static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UARTE_BAUDRATE_9600
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_9600
#endif
    };


    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);

}

#define MAX_TRANFERSIZE            252 
#define DATA_SIZE                  MAX_TRANFERSIZE- PACKET_HEADER_SIZE 
uint8_t sending_data[252];
void send_data_dcu(uint16_t length)
{
   uint32_t       err_code;

	uint16_t sent_bytes_count = 0;
        uint16_t length1 = length;

       NRF_LOG_INFO("first %d",length);
	while(length > 0)
	{
               
		if(length > DATA_SIZE)
		{
                        memset(sending_data , 0, MAX_TRANFERSIZE);
                        memcpy(sending_data,&header,PACKET_HEADER_SIZE);
                        memcpy(sending_data+PACKET_HEADER_SIZE,data_array1+sent_bytes_count,DATA_SIZE);
		        fillPacket(BACKWORD, sending_data, DATA_SIZE + PACKET_HEADER_SIZE);
                        for(int i =sent_bytes_count;i<= DATA_SIZE;i++)
                        {
                          NRF_LOG_RAW_INFO(" %x",data_array1[i]);
                        }
			sent_bytes_count += DATA_SIZE;
			length -= DATA_SIZE;


		}
		else
		{
                       length1  = length;
                        memset(sending_data , 0, MAX_TRANFERSIZE);
                        memcpy(sending_data,&header,PACKET_HEADER_SIZE);
                        memcpy(sending_data+PACKET_HEADER_SIZE,data_array1+sent_bytes_count,DATA_SIZE);
			fillPacket(BACKWORD, sending_data, (length1 + PACKET_HEADER_SIZE));
			
                        for(int i =sent_bytes_count;i< sent_bytes_count+length;i++)
                        {
                            NRF_LOG_RAW_INFO(" %x",data_array1[i]);
                        }
                        length = 0;
		}
	}
        rx_index = 0;
        rx_queue_pop();
}




void main(void)

  {
    uint32_t err_code; 
    uint8_t check_nerby[PACKET_HEADER_SIZE];

    gpio_init();
    nrf_gpio_cfg_output(POW_CNTRL_PIN);
    nrf_gpio_pin_set(POW_CNTRL_PIN);

    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    clocks_start();

     
    err_code = esb_init();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEBUG("NODE");

    
    uart_init();
    

    err_code = nrf_esb_start_rx();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_FLUSH();
  
    while(true)
    {

        switch ( PACKET )
        {
            case    DATA_PACKET  :

                       if ( buf_count != 0 )
                       {
                            if ( rx_queue[rx_tail].data[POS_DIRECTION] == BACKWORD )
                            {
                                /* Respone from meter */

                                fillPacket(BACKWORD, (void *)rx_queue[rx_tail].data, rx_queue[rx_tail].length);
                                rx_queue_pop();
                            }

                            else
                            {
                                /* Request from DCU */

                                if(rx_queue[rx_tail].data[(POS_CIRCLE_ARRAY+ Current_Circle ) + 1] != 0)
                                {
                                   fillPacket( FORWARD, (void *)rx_queue[rx_tail].data, rx_queue[rx_tail].length );
                                   rx_queue_pop();
                                }
                                else
                                {
                                  Construct_DLMS_Packet();

                                }
                            }
                       }
                       PACKET = rx_queue[rx_tail].data[POS_PACKET_TYPE];
            break;


            case    PING_PACKET : 
                        
                        pingPacket();
                        PACKET = rx_queue[rx_tail].data[POS_PACKET_TYPE];
            break;


            case    INS_PACKET :
                        
                        rx_payload.data[ POS_CIRCLE_ARRAY + Current_Circle ] = addr_prefix[0];
                        fillPacket( BACKWORD, rx_payload.data, rx_payload.length );
                        PACKET = rx_queue[rx_tail].data[POS_PACKET_TYPE];
            break;

        }

       
        if(Diagnostic_Test == 1)
        {
            doDiagnosticTest();
        }

        if ( sendINS == 1 )
        {

            sendINS = 0;

            memset(&header, 0, PACKET_HEADER_SIZE);
            
            header.packet_type      =     INS_PACKET;
            header.Direction        =     BACKWORD;
            header.circle_array[Current_Circle] = addr_prefix[0];

  
            memcpy(tx_payload.data, &header, sizeof(header));
            memcpy(tx_payload.data + sizeof(header), "01000001",sizeof("01005748"));
            tx_payload.length = ( sizeof(header) + sizeof("01005748") );
            

            if ( Current_Circle == 0 )
                  set_slave_adress(DCU_PREFIX[0], DCU_BASE); 
            else
                  set_slave_adress(Prev_neighbor_table[0].node_id, arrays[Current_Circle - 1]); 

            rx_queue_pop(); 

            sendDataToNextSlave();
        }

        if(Uart_rx_flag == 1)
        {
           nrf_delay_us(50000);
          //memcpy(&rx_queue[rx_tail].data[PACKET_HEADER_SIZE+1], data_array1, rx_index);
          send_data_dcu(rx_index);
          rx_index = 0;
        }
    }
  
    if (NRF_LOG_PROCESS() == false)
    {
        __WFE();
    }
        
}