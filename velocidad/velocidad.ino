#include <Servo.h> 

Servo motor_1; 
Servo motor_2; 
Servo motor_3; 
Servo motor_4; 
int pwm_value = 1;

void setup() 
{ 
	Serial.begin(9600);
  Serial.println("initializing");

  motor_1.attach(9);  // attaches the servo on pin 9 to the servo object 
  motor_2.attach(10);  // attaches the servo on pin 9 to the servo object 
  motor_3.attach(4);  // attaches the servo on pin 9 to the servo object 
  motor_4.attach(5);  // attaches the servo on pin 9 to the servo object 
  motor_1.write(0);
  motor_2.write(0);
  motor_3.write(0);
  motor_4.write(0);

  delay(400);

  motor_1.write(0);
  motor_2.write(0);
  motor_3.write(0);
  motor_4.write(0);

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
			motor_1.write(pwm_value);
			motor_2.write(pwm_value);
			motor_3.write(pwm_value);
			motor_4.write(pwm_value);
    }
    if (ch == 'd' || ch == 'D')
    {
      pwm_value--;
      Serial.println(pwm_value);
			motor_1.write(pwm_value);
			motor_2.write(pwm_value);
			motor_3.write(pwm_value);
			motor_4.write(pwm_value);
    }
  }

}
