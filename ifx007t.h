#ifndef IFX007T_H
#define IFX007T_H


class Ifx007t
{
  public:
    Ifx007t ();
    ~Ifx007t ();
    void begin(byte bp_pwm1, byte bp_pwm2, byte bp_inh);
    void end ();
    void disable();
    void stop();
    void fullforward();
    void fullbackward();
    void pwm(int pwm);


  private:
    byte p_pwm1;
    byte p_pwm2;
    byte p_inh;
};

#endif /* DOGGRAPHICDISPLAY_H */
