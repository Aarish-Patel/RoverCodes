class Drive
{
  private:
    int Rdir, Ldir, Rpwm, Lpwm;  //drive output pins
    int freq, Lchannel, Rchannel, resolution;
    int x, y, M;

  public:
    Drive()
    {
      freq = 8000;
      Lchannel = 0;
      Rchannel = 1;
      resolution = 8;
      x = 0;
      y = 0;
      M = 0;
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
      //      Serial.println("done pins");

    }
    
    void Stop(void)
    {
      ledcWrite(Lchannel, 0);
      ledcWrite(Rchannel, 0);
    }

    void MotorCode(int x, int y, int M)
    {
      // STOP
      if (abs(x) < 170 && abs(y) < 170)
      {
//        Serial.println("Stop");

        digitalWrite(Ldir, LOW);
        digitalWrite(Rdir, LOW);
        ledcWrite(Lchannel, 0);
        ledcWrite(Rchannel, 0);
      }

      // FORWARD MAX
      else if (abs(x) < 170 && y > 170)
      {
//        Serial.println("FM");

        digitalWrite(Ldir, HIGH);
        digitalWrite(Rdir, HIGH);
        int i = map(abs(y) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(y) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // BACKWARD MAX
      else if (abs(x) < 170 && y < 170)
      {
//        Serial.println("BM");

        digitalWrite(Ldir, LOW);
        digitalWrite(Rdir, LOW);
        int i = map(abs(y) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(y) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // SPOT LEFT
      else if (x < 170 && abs(y) <= 170)
      {
//        Serial.println("SL");

        digitalWrite(Ldir, LOW);
        digitalWrite(Rdir, HIGH);
        int i = map(abs(x) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(x) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // SPOT RIGHT
      else if (x > 170 && abs(y) <= 170)
      {
//        Serial.println("SR");

        digitalWrite(Ldir, HIGH);
        digitalWrite(Rdir, LOW);
        int i = map(abs(x) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(x) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // OCTET 1
      else if (x > 170 && y > 170 && x > y)
      {
//        Serial.println("O1");

        digitalWrite(Ldir, HIGH);
        digitalWrite(Rdir, LOW);
        int i = map(abs(abs(x) - abs(y)) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(x) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // OCTET 2
      else if (x > 170 && y > 170 && x < y)
      {
//        Serial.println("O2");

        digitalWrite(Ldir, HIGH);
        digitalWrite(Rdir, HIGH);
        int i = map(abs(abs(x) - abs(y)) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(y) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // OCTET 3
      else if (x < 170 && y > 170 && abs(x) < y)
      {
//        Serial.println("O3");

        digitalWrite(Ldir, HIGH);
        digitalWrite(Rdir, HIGH);
        int i = map(abs(y) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(abs(x) - abs(y)) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // OCTET 4
      else if (x < 170 && y > 170 && abs(x) >= y)
      {
//        Serial.println("O4");

        digitalWrite(Ldir, LOW);
        digitalWrite(Rdir, HIGH);
        int i = map(abs(x) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(abs(x) - abs(y)) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }
      // OCTET 5
      else if (x < 170 && y < 170 && abs(x) > abs(y))
      {
//        Serial.println("O5");

        digitalWrite(Ldir, LOW);
        digitalWrite(Rdir, HIGH);
        int i = map(abs(x) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(abs(x) - abs(y)) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // OCTET 6
      else if (x < 170 && y < 170 && abs(x) < abs(y))
      {
//        Serial.println("O6");

        digitalWrite(Ldir, LOW);
        digitalWrite(Rdir, LOW);
        int i = map(abs(y) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(abs(x) - abs(y)) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // OCTET 7
      else if (x > 170 && y < 170 && abs(x) < abs(y))
      {
//        Serial.println("O7");

        digitalWrite(Ldir, LOW);
        digitalWrite(Rdir, LOW);
        int i = map(abs(abs(x) - abs(y)) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(y) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }

      // OCTET 8
      else if (x > 170 && y < 170 && abs(x) > abs(y))
      {
//        Serial.println("O8");

        digitalWrite(Ldir, HIGH);
        digitalWrite(Rdir, LOW);
        int i = map(abs(abs(x) - abs(y)) * (M * 0.1), 170, 1023, 0, 255);
        int j = map(abs(x) * (M * 0.1), 170, 1023, 0, 255);
        ledcWrite(Lchannel, i);
        ledcWrite(Rchannel, j);
      }
    }
};
