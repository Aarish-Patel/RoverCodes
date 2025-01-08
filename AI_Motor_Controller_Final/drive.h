class Drive
{
  private:
    int Rdir, Ldir, Rpwm, Lpwm;  //drive output pins
    int freq, Lchannel, Rchannel, resolution;
    int L, R;

  public:
    Drive()
    {
      freq = 8000;
      Lchannel = 0;
      Rchannel = 1;
      resolution = 8;
      L = 0;
      R = 0;
    }
    
    void setDrivePins(int dir_right, int dir_left, int pwm_right, int pwm_left)
    {
      Ldir = dir_left;
      Lpwm = pwm_left;
      Rdir = dir_right;
      Rpwm = pwm_right;

      ledcSetup(Lchannel, freq, resolution);
      ledcSetup(Rchannel, freq, resolution);
      ledcAttachPin(Lpwm, Lchannel);
      ledcAttachPin(Rpwm, Rchannel);
      pinMode(Ldir, OUTPUT);
      pinMode(Rdir, OUTPUT);

    }
    
    void Stop(void)
    {
      ledcWrite(Lchannel, 0);
      ledcWrite(Rchannel, 0);
    }

    void MotorCode(int L, int R)
    {
      ledcWrite(Lchannel, abs(L)*2.55);
      ledcWrite(Rchannel, abs(R)*2.55);
      if(L > 0)
      {
        digitalWrite(Ldir, HIGH);
      }
      else if(L < 0)
      {
        digitalWrite(Ldir, LOW);
      }
      else
      {
        digitalWrite(Ldir, LOW);
        ledcWrite(Lchannel, 0);
      }
      if(R > 0)
      {
        digitalWrite(Rdir, HIGH);
      }
      else if(R < 0)
      {
        digitalWrite(Rdir, LOW);
      }
      else
      {
        digitalWrite(Rdir, LOW);
        ledcWrite(Rchannel, 0);
      }
    }
};
