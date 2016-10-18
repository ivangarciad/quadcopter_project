#include "TimerOne.h"
 
//Timer t;
int pin1 = 9;
int pin2 = 10;

unsigned long int period = 1000000;
unsigned long int prev_period = 0;
int fps = 1;

void input_commands(void);

 
void setup()
{
  Serial.begin(9600);

  pinMode(pin1, OUTPUT);
  pinMode(pin2, OUTPUT);

  Timer1.initialize(period); 
  Timer1.pwm(9, 256);
  Timer1.pwm(10, 256);
}
 
 
void loop()
{
  input_commands();

  if (period != prev_period)
  {
    Timer1.setPeriod(period); 
    Timer1.setPwmDuty(9, 256);
    Timer1.setPwmDuty(10, 256);

    prev_period = period;

    char tbs_analog[100];
    sprintf(tbs_analog, "fps:%d; period:%lu",
      fps,
      period);

    Serial.print(tbs_analog);
    Serial.println();
  }
}

void input_commands(void)
{
  if(Serial.available() > 0)
  {
    char aux = Serial.read();
    switch (aux)
    {
      case '+':
        fps += 1;
        break;
      case '-':
        fps -= 1;
        break;
    }
    period = 1000000/fps;
  }
}

