/*
 * camera.cpp - contains implementation of functions used with
 * Toshiba TCM8230MD camera module on the Camera board.
 * Author: Ewan Coldicott
 * Date: 11 June 2012
 */
 
#include "ch.h"
#include "hal.h"
#include "camera.h"
#include "chprintf.h"
#include "stm32f4xx.h"
#include "stm32_dma.h"
#include "jpge.h"
 
using namespace std;
using namespace jpge;
 
#define JPEG_IMAGE_WIDTH 320
#define JPEG_IMAGE_HEIGHT 240          //Must be divisible by 16
#define BYTES_PER_PIXEL 2
#define COLOUR_CHANNELS 3              //YUV422 camera output
#define OUT_STREAM_DMA_THRESHOLD 64    //For transfers >=, DMA will be used, else software.
 
static uint32_t jpegFileIndex = 0;
static uint32_t jpegFileMax = 0;
static uint8_t* jpegOutputBuffer = NULL;

//we'll use DMA double buffer mode so that one block of 16 lines can be read (from DCMI) while the other is being encoded (passed to jpge).
//we read 16 lines at a time as the JPEG macrocells are 16x16 pixels when using YUV4
static uint8_t rawImageBuffer0[JPEG_IMAGE_WIDTH*BYTES_PER_PIXEL*16];
static uint8_t rawImageBuffer1[JPEG_IMAGE_WIDTH*BYTES_PER_PIXEL*16];
static uint8_t jpgeOutBuffer[JPGE_OUT_BUF_SIZE];                      //JPGE_OUT_BUF_SIZE defined in jpge.h

//linker will place the encoder object in core-coupled memory, since DMA access not required to most of it.
//pointer to jpgeOutBuffer passed to jpge - linker will place this buffer in ethram, so DMA can access it.
//if the jpgeOutBuffer was a statically-allocated field of the jpge object, it'd be placed in CCM by the linker - then we can't use DMA to read from it.
jpeg_encoder encoder;  
camBoard_jpge_output_stream out;   

 
 /***********************************************
  *      JPEG Encoder config and I/O section    *
  ***********************************************/
  //Destructor for application-specific output stream
  //Called upon de-init of the JPEG encoder object
camBoard_jpge_output_stream::~camBoard_jpge_output_stream() {
  //Memory we're using for output stream is statically allocated, no need to release etc.
  jpegFileIndex = 0;
}

  // Called by jpge in order to provide an application-specific output method.
  // Header and image data (of length len) will be provided in the buffer pointed to by Pbuf.
  // put_buf() is generally called with len==JPGE_OUT_BUF_SIZE bytes (2048), but for headers it'll be called with smaller amounts.
  // Returns TRUE upon successful output, FALSE otherwise.
  // Note that this function is NOT called to indicate when the frame has ended - user must reset jpegFileIndex
bool camBoard_jpge_output_stream::put_buf(const void* Pbuf, int len) {
  //Check enough space left in statically allocated jpegFile array
  if( (jpegFileIndex + len) >= jpegFileMax ) {
    //out of space
    return FALSE;
  }
  
  //If only a small amount of data to copy, use software
  if( len < OUT_STREAM_DMA_THRESHOLD ) {
    for( int16_t i = 0; i < len; i ++ ) {
      jpegOutputBuffer[jpegFileIndex + i] = ((uint8_t*)Pbuf)[i];
    }
    jpegFileIndex += len;
    return TRUE;
  }
    
  //Else, use a mem-mem DMA stream to copy data
  if (dmaStreamAllocate( STM32_DMA2_STREAM4,
                     15,
                     NULL,
                     NULL))
  { return FALSE; }
  dmaStreamSetFIFO( STM32_DMA2_STREAM4, 
    STM32_DMA_FCR_FTH_HALF | 
    STM32_DMA_FCR_DMDIS );
  dmaStartMemCopy( STM32_DMA2_STREAM4,
    STM32_DMA_CR_PL(0) |                  //Low DMA priority
    STM32_DMA_CR_PSIZE_WORD |             //Word-sized transfers
    STM32_DMA_CR_MSIZE_WORD |
    STM32_DMA_CR_MBURST_SINGLE |
    STM32_DMA_CR_PBURST_SINGLE,
    Pbuf,                                 //source memory address
    &(jpegOutputBuffer[jpegFileIndex]) ,  //destination memory address
    len/4 );                              //number of transfers (words)
  
  //If the buffer length is not a multiple of one word, copy the remaining bytes in software
  if( len % 4 != 0 ) {
    for( int i = len ; i % 4 != 0 ; i-- ) {
      jpegOutputBuffer[jpegFileIndex + i - 1] = ((uint8_t*)Pbuf)[i-1];
    }
  }
    
  dmaWaitCompletion( STM32_DMA2_STREAM4 );    //Blocks until DMA transfer complete.
  dmaStreamRelease( STM32_DMA2_STREAM4 );
  //Would likely get away with returning after some number of transfers (32 words?) and releasing the stream in a callback.
  //Since the jpge class allocates the buffer (Pbuf) statically and reuses it from index 0 after calling put_buf, it is unlikely to clobber the
  //  higher elements while the DMA transfer is ongoing. Would allow jpge some more clocks, but is somewhat unsafe!
  
  jpegFileIndex += len;
  return TRUE;
}

 /***********************************************
  * End of JPEG Encoder config and I/O section  *
  ***********************************************/

 
 /*
 * PWM configuration structure, for Timer4 (camera EXTCLK).
 * Cyclic callback disabled.
 * Channel 3 enabled without callback.
 * The active state is a logic one.
 */
static PWMConfig pwm4cfg = {
  24000000,                                 /* 24MHz PWM clock frequency. Max clock is 84MHz. */
  2,                                        /* PWM period every 2 ticks (ie new period @ 12MHz, D = 0, 0.5 possible) */
  NULL,                                     /* No cyclic callback */
  {
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_DISABLED, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},         /* Channel 3 enabled, active high output. No callback. */
    {PWM_OUTPUT_DISABLED, NULL}
  },
  /* HW dependent part.*/
  0
};


 /*
 * I2C2 driver configuration structure.
 */
static const I2CConfig i2c2cfg = {
  OPMODE_I2C,
  400000,
  STD_DUTY_CYCLE,
};

 
 /*
 * Camera start and test code.
 */
void cameraInit(void) {
  /* Start up required drivers */
  pwmStart(&PWMD4, &pwm4cfg);
  i2cStart(&I2CD2, &i2c2cfg);
  RCC->AHB2ENR |= RCC_AHB2ENR_DCMIEN;   //enable DCMI clock
  
  pwmEnableChannel(&PWMD4, 2, 1);     /* Enable PWM output on channel 3, width 1 clock tick -> D=0.5, 12MHz square wave */
  chThdSleepMilliseconds(1);          /* Wait for greater than 100 clock ticks */
  palSetPad(GPIOD, GPIOD_RESET);      /* Pull RESET line high in order to enable camera */  
  chThdSleepMilliseconds(1);          /* Wait for camera to start */
}

/*
 *	Camera I2C communication test code. Prints result on Serial Driver 3
 */
void cameraI2CTest(void) {
  msg_t status = RDY_OK;
  systime_t tmo = MS2ST(10);

  /* Camera should be ready to respond to requests: test it. */
  /* The camera should acknowledge the I2C transaction */
  uint8_t txbuf[2];
  txbuf[0] = CAM_CONFIG1;
  txbuf[1] = CAM_CONFIG1_DEFAULT;
  i2cAcquireBus( &I2CD2 );
  status = i2cMasterTransmitTimeout( &I2CD2, CAM_ADDR, txbuf, 2, NULL, 0, tmo );
  i2cReleaseBus( &I2CD2 );
  
  if( status == RDY_RESET ) {
    chprintf((BaseChannel*)&SD3, "Camera I2C test: Error %X\n\r", i2cGetErrors(&I2CD2));
  }
  else if( status == RDY_TIMEOUT ) {
    chprintf((BaseChannel*)&SD3, "Camera I2C test: Timeout\n\r");
  }
  else {
 //   chprintf((BaseChannel*)&SD3, "Camera I2C test: Success\n\r");
  }
}


/*
 *  Configures camera to have desired image settings:
 *  - 320x240, YUV422
 *  - Test pattern (colour bars) ENABLED
 *  - Embedded synchronisation codes DISABLED
 *  - Output ENABLED
 */
void cameraConfigure(void) {
  systime_t tmo = MS2ST(20);
  uint8_t txbuf[2];

  txbuf[0] = CAM_CONFIG1;
  txbuf[1] =
    CAM_CONFIG1_FPS;        /* FPS bit high = 15fps for 'normal' EXTCLK */
  i2cAcquireBus( &I2CD2 );
  i2cMasterTransmitTimeout( &I2CD2, CAM_ADDR, txbuf, 2, NULL, 0, tmo );
  i2cReleaseBus( &I2CD2 );
  chThdSleepMilliseconds(1);

  txbuf[0] = CAM_CONFIG3;
  txbuf[1] =
//    CAM_CONFIG3_TESPIC |
    CAM_CONFIG3_HSYNCSEL |  /* HD blanking */ 
    CAM_CONFIG3_D_MASK_0 ;
  i2cAcquireBus( &I2CD2 );
  i2cMasterTransmitTimeout( &I2CD2, CAM_ADDR, txbuf, 2, NULL, 0, tmo );
  i2cReleaseBus( &I2CD2 );
  chThdSleepMilliseconds(1);

  txbuf[0] = CAM_CONFIG2;
  txbuf[1] =
 //   CAM_CONFIG2_PICFMT |    /* PICFMT = 1 -> RGB565 output. PICFMT = 0 -> YUV422 */
    CAM_CONFIG2_PICSIZ_0 ;  /* 320x240 */
  i2cAcquireBus( &I2CD2 );
  i2cMasterTransmitTimeout( &I2CD2, CAM_ADDR, txbuf, 2, NULL, 0, tmo );
  i2cReleaseBus( &I2CD2 );
}


//gets a frame from the camera, stores it UNCOMPRESSED in the memory of length size words.
//frame must be at least size words long. size must be the exact size of the image, in words.
//if size is too large, this function will never return. if size is too small, some image content will be lost.
bool_t cameraSnapshot(uint32_t* frame, uint32_t size)
{
  DCMI->CR = DCMI_CR_CM ;
  if (dmaStreamAllocate( STM32_DMA2_STREAM1,
                           15,
                           NULL,
                           NULL))
  {
    return FALSE;
  }
  dmaStreamSetPeripheral(STM32_DMA2_STREAM1, &(DCMI->DR));
  dmaStreamSetMemory0(STM32_DMA2_STREAM1, frame);
  dmaStreamSetTransactionSize(STM32_DMA2_STREAM1, size);
  dmaStreamSetMode(STM32_DMA2_STREAM1, 
    STM32_DMA_CR_PL(2) |            /* High DMA priority */
    STM32_DMA_CR_PSIZE_WORD |       /* Transfers from peripheral to DMA FIFO are 1 word long */
    STM32_DMA_CR_MSIZE_WORD |       /* Transfers from DMA FIFO to SRAM are 1 word long */
    STM32_DMA_CR_PBURST_SINGLE |
    STM32_DMA_CR_MBURST_SINGLE |
    STM32_DMA_CR_MINC |             /* Increment memory address by MSIZE after each FIFO->memory transfer */
    STM32_DMA_CR_CHSEL(1) );        /* DMA2, Stream 1, Channel 1 = DCMI requests (pg 165 reference manual) */
  dmaStreamSetFIFO(STM32_DMA2_STREAM1,
    STM32_DMA_FCR_FTH_FULL |
    STM32_DMA_FCR_DMDIS );
  dmaStreamEnable(STM32_DMA2_STREAM1);
  DCMI->CR |= DCMI_CR_ENABLE;
  DCMI->CR |= DCMI_CR_CAPTURE;
  dmaWaitCompletion(STM32_DMA2_STREAM1);
  dmaStreamRelease(STM32_DMA2_STREAM1);
  return TRUE;
}


//Captures a single image from the camera and encodes it to JPEG format. Assumes camera is already configured.
//frame is the output buffer
//size is the maximum number of bytes which can be written to the output buffer
//If successful, returns the number of bytes in the JPEG file.
//If the provided buffer is too small for the image, returns size+1
//If any other error occurs, returns 0
uint32_t cameraJpegSnapshot(uint8_t* frame, uint32_t size) {
  const uint8_t* rawImgBufs[2] = { rawImageBuffer0, rawImageBuffer1 };
  uint32_t linesRemaining = JPEG_IMAGE_HEIGHT;
  uint32_t numMacroBlocksProcessed = 0;
  jpegFileMax = size;
  jpegFileIndex = 0;
  jpegOutputBuffer = frame;
  if( encoder.init(&out, JPEG_IMAGE_WIDTH, JPEG_IMAGE_HEIGHT, COLOUR_CHANNELS, jpgeOutBuffer) != TRUE ) { return 0; }
  
  //get DMA stream, configure for double buffer transfer from DCMI
  DCMI->CR = DCMI_CR_CM ;
  DCMI->CR |= DCMI_CR_ENABLE ;
  if( dmaStreamAllocate(STM32_DMA2_STREAM1, 15, NULL, NULL) == TRUE) { return 0; } 
  dmaStreamSetPeripheral(STM32_DMA2_STREAM1, &(DCMI->DR));
  dmaStreamSetTransactionSize(STM32_DMA2_STREAM1, JPEG_IMAGE_WIDTH*BYTES_PER_PIXEL*16/4);     //transfer 16 lines at a time, in transactions of 1 word
  dmaStreamSetMemory0(STM32_DMA2_STREAM1, rawImgBufs[0]);
  dmaStreamSetMemory1(STM32_DMA2_STREAM1, rawImgBufs[1]);
  dmaStreamSetMode(STM32_DMA2_STREAM1, 
    STM32_DMA_CR_PL(2) |            /* High DMA priority*/
    STM32_DMA_CR_PSIZE_WORD |       /* Transfers from peripheral to DMA FIFO are 1 word long */
    STM32_DMA_CR_MSIZE_WORD |       /* Transfers from DMA FIFO to SRAM are 1 word long */
    STM32_DMA_CR_PBURST_SINGLE |
    STM32_DMA_CR_MBURST_SINGLE |
    STM32_DMA_CR_MINC |             /* Increment memory address by MSIZE after each FIFO->memory transfer */
    STM32_DMA_CR_CHSEL(1) |         /* DMA2, Stream 1, Channel 1 = DCMI requests (pg 165 reference manual) */
    STM32_DMA_CR_DBM );             /* DMA Double Buffer mode ENABLED */
  dmaStreamSetFIFO(STM32_DMA2_STREAM1,
    STM32_DMA_FCR_FTH_HALF |        /* FIFO flushed to SRAM when half full (two words) */
    STM32_DMA_FCR_DMDIS );
  
  dmaStreamEnable(STM32_DMA2_STREAM1);
  DCMI->CR |= DCMI_CR_CAPTURE ;
  
  while( linesRemaining > 0 ) {
    uint32_t currentBuffer = numMacroBlocksProcessed % 2 ;
    
    while( ((STM32_DMA2_STREAM1->stream->CR & STM32_DMA_CR_CT)>>19) == currentBuffer ) { ; }    //wait for buffer target to change (in double buffer mode)
    for( int i = 0; i < 16; i++ ) {
      if( encoder.process_scanline( &((rawImgBufs[currentBuffer])[i*JPEG_IMAGE_WIDTH*BYTES_PER_PIXEL]) ) == FALSE ) { return 0; }
    }
    
    linesRemaining -= 16;
    numMacroBlocksProcessed++;
  }
  
  //all lines processed
  dmaStreamDisable(STM32_DMA2_STREAM1);
  dmaStreamRelease(STM32_DMA2_STREAM1);
  if( encoder.process_scanline( NULL ) == FALSE ) { return 0; }
  
  return jpegFileIndex;  
}
