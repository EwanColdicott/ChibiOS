/*
 * camera.h - contains definitions and declarations useful for the use of the
 * Toshiba TCM8230MD camera module on the Camera board.
 * Author: Ewan Coldicott
 * Date: 11 June 2012
 */

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "jpge.h"

/*
 * I2C slave address of TCM8230MD camera module
 */
#define CAM_ADDR 0x3C

/*
 * I2C register addresses
 */
#define CAM_CONFIG1               0x02      /* Framerate, AC flicker filter, clock polarity settings */
#define CAM_CONFIG2               0x03      /* Output enable, image size, colour format, colour/B&W settings*/
#define CAM_CONFIG3               0x1E      /* Test mode bits*/

/*
 * Register bit definitions
 */
#define CAM_CONFIG1_DEFAULT       0x40
#define CAM_CONFIG1_FPS           0x80
#define CAM_CONFIG1_ACF           0x40
#define CAM_CONFIG1_DCLKP         0x02
#define CAM_CONFIG1_ACFDET        0x01

#define CAM_CONFIG2_DEFAULT       0x80
#define CAM_CONFIG2_DOUTSW        0x80
#define CAM_CONFIG2_DATAHZ        0x40
#define CAM_CONFIG2_PICSIZ_3      0x20
#define CAM_CONFIG2_PICSIZ_2      0x10
#define CAM_CONFIG2_PICSIZ_1      0x08
#define CAM_CONFIG2_PICSIZ_0      0x04
#define CAM_CONFIG2_PICFMT        0x02
#define CAM_CONFIG2_CM            0x01

#define CAM_CONFIG3_DEFAULT       0x68
#define CAM_CONFIG3_D_MASK_1      0x80
#define CAM_CONFIG3_D_MASK_0      0x40
#define CAM_CONFIG3_CODESW        0x20
#define CAM_CONFIG3_CODESEL       0x10
#define CAM_CONFIG3_HSYNCSEL      0x08
#define CAM_CONFIG3_TESPIC        0x04
#define CAM_CONFIG3_PICSEL_1      0x02
#define CAM_CONFIG3_PICSEL_0      0x01


//Output stream class, used by jpge in order to write the encoder output.
class camBoard_jpge_output_stream : public jpge::output_stream {
  public:
    ~camBoard_jpge_output_stream();
    bool put_buf(const void* Pbuf, int len);
};

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void cameraInit(void);
  
  void cameraI2CTest(void);
  
  void cameraConfigure(void);
  
  bool_t cameraSnapshot(uint32_t* frame, uint32_t size);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

//Captures an image from the camera and encodes it to JPEG format.
//frame is the output buffer
//size is the maximum number of bytes which can be written to the output buffer
//If successful, returns the number of bytes in the JPEG file.
//If the provided buffer is too small for the image, returns size+1
//If any other error occurs, returns 0
uint32_t cameraJpegSnapshot(uint8_t* frame, uint32_t size);

#endif /* _CAMERA_H_ */
