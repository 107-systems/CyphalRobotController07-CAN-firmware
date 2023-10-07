#include <Arduino.h>

#include "ifx007t.h"
//----------------------------------------------------public Functions----------------------------------------------------
//Please use these functions in your sketch

/*-----------------------------
constructor for class, not needed by Arduino but for complete class. does not do anything.
*/
Ifx007t::Ifx007t()
{
}

/*-----------------------------
destructor for class, not needed by Arduino but for complete class. Calls Arduino end function
*/
Ifx007t::~Ifx007t()
{
  end();
}

/*-----------------------------
Arduino begin function. Forward data to initialize function
*/
void Ifx007t::begin(byte bp_pwm1, byte bp_pwm2, byte bp_inh)
{
  Ifx007t::p_pwm1=bp_pwm1;
  Ifx007t::p_pwm2=bp_pwm2;
  Ifx007t::p_inh=bp_inh;
  pinMode(p_pwm1, OUTPUT);
  pinMode(p_pwm2, OUTPUT);
  pinMode(p_inh, OUTPUT);
}

/*-----------------------------
Arduino end function. stop SPI if enabled
*/
void Ifx007t::end()
{
}

void Ifx007t::disable()
{
  digitalWrite(p_pwm1,LOW);
  digitalWrite(p_pwm2,LOW);
  digitalWrite(p_inh,LOW);
}
void Ifx007t::stop()
{
  analogWrite(p_pwm1,0);
  analogWrite(p_pwm2,0);
  digitalWrite(p_pwm1,LOW);
  digitalWrite(p_pwm2,LOW);
  digitalWrite(p_inh,HIGH);
}
void Ifx007t::fullforward()
{
  digitalWrite(p_pwm1,HIGH);
  digitalWrite(p_pwm2,LOW);
  digitalWrite(p_inh,HIGH);
}
void Ifx007t::fullbackward()
{
  digitalWrite(p_pwm1,LOW);
  digitalWrite(p_pwm2,HIGH);
  digitalWrite(p_inh,HIGH);
}
void Ifx007t::pwm(int pwm)
{
  digitalWrite(p_inh,HIGH);
  if(pwm==0)
  {
    analogWrite(p_pwm1,0);
    analogWrite(p_pwm2,0);
    digitalWrite(p_pwm1,LOW);
    digitalWrite(p_pwm2,LOW);
  }
  else if(pwm>0)
  {
    if(pwm>255) pwm=255;
    analogWrite(p_pwm1,pwm);
    analogWrite(p_pwm2,0);
    digitalWrite(p_pwm2,LOW);
  }
  else
  {
    pwm=-pwm;
    if(pwm>255) pwm=255;
    analogWrite(p_pwm2,pwm);
    analogWrite(p_pwm1,0);
    digitalWrite(p_pwm1,LOW);
  }

}
