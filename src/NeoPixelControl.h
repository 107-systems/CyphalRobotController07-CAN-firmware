/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

#ifndef AUX_CTRL_NEO_PIXEL_CONTROL_H_
#define AUX_CTRL_NEO_PIXEL_CONTROL_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Adafruit_NeoPixel.h>

/**************************************************************************************
 * CLASS DEFINITION
 **************************************************************************************/

class NeoPixelControl
{
public:

  NeoPixelControl(int const pin, int const num_pixels);


  void begin();


  void light_off();
  void light_green();
  void light_red();
  void light_blue();
  void light_white();
  void light_amber();

  inline Adafruit_NeoPixel & pixels() { return _pixels; }


private:
  Adafruit_NeoPixel _pixels;
};

#endif /* AUX_CTRL_NEO_PIXEL_CONTROL_H_ */
