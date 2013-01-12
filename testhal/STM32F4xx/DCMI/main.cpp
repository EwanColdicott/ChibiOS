/*
   Original File license:
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
    
   Code below is adapted from the STM32F4 Discovery board demo for Chibi/OS which has the above license.
*/

#include "ch.h"
#include "hal.h"
#include "camera.h"
#include "bmp.h"
#include "chprintf.h"

using namespace std;

#define JPEG_OUTPUT_MAX 32768          //Up to 32KiB allocated for each output JPEG file
#define COMMS_READY 0xAC
 
static uint8_t jpegFrame1[JPEG_OUTPUT_MAX];   
//static uint32_t frame1Size = 0;
//static uint8_t jpegFrame2[JPEG_OUTPUT_MAX];
//static uint32_t frame2Size = 0;

Thread* txThread;

/*
 * PWM configuration structure, for Timer3 (IR LED).
 * Cyclic callback disabled.
 * Channel 2 enabled without callback.
 * The active state is a logic one.
 */
static PWMConfig pwm3cfg = {
  156000,                                   /* 156kHz PWM clock frequency.   */
  4,                                        /* PWM period every 4 ticks (ie new period @ 39kHz - D = 0,0.25,0.50,0.75 possible) */
  NULL,                                     /* No cyclic callback */
  {
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},         /* Channel 2 enabled, active high output. No callback. */
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL}
  },
  /* HW dependent part.*/
  0
};


/*
 * Serial driver 3 configuration structure.
 * Uses USART3, provides buffers, nice abstraction etc.
 * 115200bps, 8N1
 */
static const SerialConfig sd3cfg = {
  57600,                    /* Bitrate */
  0,                        /* USART3 config register 1: defines parity settings etc */
  USART_CR2_STOP1_BITS,     /* Config register 2: defines stop bits */
  0                         /* Config register 3 */
};


/*
 * This is a periodic thread that does absolutely nothing except flashing
 * an LED.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (TRUE) {
    palSetPad(GPIOA, GPIOA_LED2);       /* User LED */
    chThdSleepMilliseconds(10);
    palClearPad(GPIOA, GPIOA_LED2); 
    chThdSleepMilliseconds(5000);
  }
  
  return (msg_t) 0;
}

/*static WORKING_AREA(waThread2, 1024);
static msg_t Thread2(void *arg) {
  (void) arg;
  bool_t buffer = FALSE;      //false=buf1, true=buf2
  uint8_t* outBuf;
  uint32_t* outBytes;
  chRegSetThreadName("encoder");
  while(TRUE) {
    if( buffer == FALSE ) {
      outBuf = jpegFrame1;
      outBytes = &frame1Size;
    } else {
      outBuf = jpegFrame2;
      outBytes = &frame2Size;
    }

    *outBytes = cameraJpegSnapshot(outBuf, JPEG_OUTPUT_MAX);
    chMsgSend(txThread, buffer==FALSE ? 1 : 2);
    buffer = !buffer;
  }
  
  return 0;   //will never execute
}*/

static WORKING_AREA(waThread4, 2048);
static msg_t Thread4(void *arg) {
  (void) arg;
  chRegSetThreadName("encoder-transmitter");
  uint16_t outBytes;
  uint8_t response;
  while(TRUE) {
    chThdSleepMilliseconds(5000);
    outBytes = cameraJpegSnapshot(jpegFrame1, JPEG_OUTPUT_MAX);
    chIOWriteTimeout((BaseChannel*)&SD3, (uint8_t*)&outBytes, 2, MS2ST(20) );
    response = chIOGetTimeout( (BaseChannel*) &SD3, MS2ST(20) );
    if( response != COMMS_READY ) continue;   //didn't get ACK back, don't transmit frame
    palSetPad(GPIOA, GPIOA_LED2);
    chIOWriteTimeout((BaseChannel*)&SD3, jpegFrame1, outBytes, MS2ST(1500));   //SERIAL driver method
    palClearPad(GPIOA, GPIOA_LED2);
  }
  
  return 0;   //will never execute
}

/*static WORKING_AREA(waThread3, 512);
static msg_t Thread3(void *arg) {
  (void) arg;
  msg_t bufferNumber;   //buffer number to transmit to comms board
  msg_t response;
  uint8_t* buf;         //pointer to JPEG buffer to transmit
  uint16_t size;        //number of bytes to transmit
  Thread* sender;       //thread performing JPEG conversions
  chRegSetThreadName("transmitter");
  while(TRUE) {
    //sleep until message from encoder received, telling which buffer to send
    sender = chMsgWait();
    bufferNumber = chMsgGet( sender );
    chMsgRelease( sender, 0 );

    //send message to comms board to tell it how large the JPEG image is
    buf = (bufferNumber == 1) ? jpegFrame1 : jpegFrame2;
    size = (bufferNumber ==  1) ? frame1Size : frame2Size;
    chIOWriteTimeout((BaseChannel*)&SD3, (uint8_t*)&size, 2, MS2ST(20) );
    response = chIOGetTimeout( (BaseChannel*) &SD3, MS2ST(20) );
    if( response != COMMS_READY ) continue;   //didn't get ACK back, don't transmit over SPI

    //transmit frame over UART
    chIOWriteTimeout((BaseChannel*)&SD3, buf, size, MS2ST(1500));   //SERIAL driver method
//    sdStop(&SD3);
//    uartStart(&UARTD3, &uart3cfg);
//    uartStartSend(&UARTD3, size, buf);
//    if(dmaStreamGetTransactionSize(UARTD3.dmatx) > 0) chThdSleepMilliseconds(3);
//    uartStop(&UARTD3);
//    sdStart(&SD3, &sd3cfg);
    
  }
  
  return 0;   //will never execute
}*/

/*
 * Application entry point.
 */
int main(void) {
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /* Activates the serial driver 3 using the config structure defined above.
   * PC10(TX) and PC11(RX) are routed to USART3 in board.h */
  sdStart(&SD3, &sd3cfg);
  
  /* Initializes PWM driver 3 */
  pwmStart(&PWMD3, &pwm3cfg);
  pwmEnableChannel(&PWMD3, 1, 2);     /* Enable PWM output on channel 2, width 2 clock ticks -> D=0.5, 39kHz square wave */
  
  /* Initialises the I2C2 peripheral, tests for connectivity, enables camera
   * Defined in camera.h */
  cameraInit();
  cameraI2CTest();
  cameraConfigure();

  //perform UART test
  uint8_t rx = chIOGet((BaseChannel*)&SD3);    //blocks until byte received
  if( rx != 0xFA ) { chSysHalt(); }
  chIOPut((BaseChannel*)&SD3, 0xFD );            //send acknowledgement byte
  rx = chIOGet((BaseChannel*)&SD3);
  if( rx != 0x7A ) { chSysHalt(); }
  
  /* Creates the blinker thread. */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);
  /* Creates the encoder thread */
//  chThdCreateStatic(waThread2, sizeof(waThread2), NORMALPRIO, Thread2, NULL);
  /* Creates the transmitter thread */
//  txThread = chThdCreateStatic(waThread3, sizeof(waThread3), NORMALPRIO+1, Thread3, NULL);
  /* Creates the encoder and transmitter thread (low fps)*/
  chThdCreateStatic(waThread4, sizeof(waThread4), NORMALPRIO, Thread4, NULL);

  /*
   * Normal main() thread activity. Send an integer via UART to host PC.
   */
//  chprintf((BaseChannel*) &SD3, "Ready for commands:\n\r");
//  chprintf((BaseChannel*) &SD3, " - T/t: Test camera I2C connectivity\n\r");
//  chprintf((BaseChannel*) &SD3, " - F/f: Get a frame from camera and send over UART\n\r"); 
  while (TRUE) {
    chThdSleepMilliseconds(500);
  }
}
