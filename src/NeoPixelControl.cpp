/*
 * Software for the auxiliary controller for the L3X-Z Hexapod
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include "NeoPixelControl.h"

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

NeoPixelControl::NeoPixelControl(int const pin, int const num_pixels)
: _pixels(num_pixels, pin, NEO_GRB)
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void NeoPixelControl::begin()
{
  _pixels.begin();
}

void NeoPixelControl::light_off()
{
  _pixels.clear();
  _pixels.show();
}

void NeoPixelControl::light_green()
{
  _pixels.fill(_pixels.Color(0, 55, 0));
  _pixels.show();
}

void NeoPixelControl::light_red()
{
  _pixels.fill(_pixels.Color(55, 0, 0));
  _pixels.show();
}

void NeoPixelControl::light_blue()
{
  _pixels.fill(_pixels.Color(0, 0, 55));
  _pixels.show();
}

void NeoPixelControl::light_white()
{
  _pixels.fill(_pixels.Color(55, 55, 55));
  _pixels.show();
}

void NeoPixelControl::light_amber()
{
  _pixels.fill(_pixels.Color(55, 40, 0));
  _pixels.show();
}
