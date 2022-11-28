/*
 * Copyright (C) ExpressLRS_relay
 *
 *
 * License GPLv3: http://www.gnu.org/licenses/gpl-3.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifdef RP2040_PLATFORM

//#include <Arduino.h>
#include <stdio.h>
#include "pico/stdlib.h"
//#include "pico/multicore.h"
#include "hardware/pio.h"
//#include "hardware/uart.h"
#include "RP2040_uart.pio.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"
//#include "sport.h"
//#include "tools.h"
//#include "config.h"
//#include "param.h"

// to do: 
// if we can reply to several device id, we should keep a table with the last field index used for this deviceid
// when we get a polling for one device id, we should use this table to search from this last field index 


// one pio and 2 state machines are used to manage the sport in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When a byte is received after a 7E, then we stop the sm that receive (it means frsky made a polling)
//    We fill a buffer with the data
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   


//#define SPORT_PIO_RX_PIN 10  // pin being used by the UART pio
#define SPORTSYNCREQUEST 0x7E
#define SPORTDEVICEID    0xE4

queue_t sportRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for Sport
PIO sportPio = pio0;
uint sportSmTx = 0; // to send the telemetry to sport
uint sportSmRx = 1; // to get the request from sport
uint sportOffsetTx ; 
uint sportOffsetRx ; 
uint sportRxMillis ;

//extern volatile boolean txIrqFired ;

#define SPORT_PIN 1 // pin used for sport

// PIO0_IRQ_0 is used for TX
// PIO0_IRQ_1 is used for RX

// dma channel is used to send Sport telemetry without blocking
int sport_dma_chan;


//uint8_t sportTxBuffer[50];
extern volatile uint8_t sTxData[];
extern volatile uint8_t sportTxCount ;

void sportPioTxHandlerIrq(void){    // when all bytes have been sent, disable the sm
    // clear the irq flag at the origin of the interrupt
    pio_interrupt_clear(pio0, 0);    
    // disable TX state machine
    sport_uart_tx_program_stop(sportPio, sportSmTx, SPORT_PIN); // stop transmitting
    // enable RX state machine
    sport_uart_rx_program_restart(sportPio, sportSmRx, SPORT_PIN , true);     
    //txIrqFired = true;    
    //printf("inirq\n");
}

void sportPioRxHandlerIrq(){    // when a byte is received on the Sport, read the pio Sport fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_1 );
  while (  ! pio_sm_is_rx_fifo_empty (sportPio ,sportSmRx)){ // when some data have been received
     uint8_t c = pio_sm_get (sportPio , sportSmRx) >> 24;         // read the data
     //printf("%x", c);
     queue_try_add (&sportRxQueue, &c);          // push to the queue
    sportStuffTime = micros();                    // save the timestamp of last received byte.
  }
}


void initSportUart() {  // setup dma, pio to manage sport bus (send and receive)
// set up the DMA but do not yet start it to send data to Sport
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
    sport_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config sportDmaConfig;
    sportDmaConfig = dma_channel_get_default_config(sport_dma_chan);
    channel_config_set_read_increment(&sportDmaConfig, true);
    channel_config_set_write_increment(&sportDmaConfig, false);
    channel_config_set_dreq(&sportDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&sportDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        sport_dma_chan,
        &sportDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &sTxData[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    sportOffsetTx = pio_add_program(sportPio, &sport_uart_tx_program);
    sport_uart_tx_program_init(sportPio, sportSmTx, sportOffsetTx, SPORT_PIN, 57600 , true); 
                    // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 
// set an irq on pio to handle when all Tx bytes have been sent; irq is activated by the pio
    irq_set_exclusive_handler( PIO0_IRQ_0 , sportPioTxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// configure the queue to get the data from Sport in the irq handle
    queue_init (&sportRxQueue, sizeof(uint8_t), 250);
// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_1 , sportPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_1 , true) ;
// Set up the state machine we're going to use to receive them.
    sportOffsetRx = pio_add_program(sportPio, &sport_uart_rx_program);
    sport_uart_rx_program_init(sportPio, sportSmRx, sportOffsetRx, SPORT_PIN, 57600 , true);  
}

void sendSTxData(){
    sport_uart_rx_program_stop(sportPio, sportSmRx, SPORT_PIN); // stop receiving
    sport_uart_tx_program_start(sportPio, sportSmTx, SPORT_PIN, true); // prepare to transmit
    sport_uart_tx_program_set_y(sportPio, sportSmTx, sportTxCount-1);  // load pio reg Y with nbr of bytes
    dma_channel_set_read_addr (sport_dma_chan, &sTxData[0], false);
    dma_channel_set_trans_count (sport_dma_chan, sportTxCount, true) ; // start the DMA to send n bytes to pio
}
        
void getSportFromQueue(){  // fill sportbuff[]
    uint8_t data; 
    if (! queue_is_empty(&sportRxQueue)) {
        queue_try_remove (&sportRxQueue,&data);
        if (sportindex < 16) sportbuff[sportindex++] = data ;    //store received bytes in a buffer
    }    
}

#endif // End RP2040_PLATFORM