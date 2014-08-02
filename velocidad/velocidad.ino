#include <Servo.h> 

Servo myservo; 
int pwm_value = 32;

void setup() 
{ 
	Serial.begin(9600);
  Serial.println("initializing");

  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  delay(4000);
	
  myservo.write(0);
  Serial.println("listo");
} 

void loop() 
{ 
	if(Serial.available() > 0)
  {
    // read the value
    char ch = Serial.read();

    if (ch == 'u' || ch == 'U')
    {
      pwm_value++;
      Serial.println(pwm_value);
			myservo.write(pwm_value);
    }
    if (ch == 'd' || ch == 'D')
    {
      pwm_value--;
      Serial.println(pwm_value);
			myservo.write(pwm_value);
    }
  }

}
