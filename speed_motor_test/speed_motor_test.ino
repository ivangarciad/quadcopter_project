#include <Servo.h> 

Servo motor_1; 
Servo motor_2; 
Servo motor_3; 
Servo motor_4; 
int pwm_value = 635;
int pwm_value_max = 1500;

void setup() 
{ 
  motor_1.attach(5);  // attaches the servo on pin 5 to the servo object 
  motor_2.attach(10); // attaches the servo on pin 10 to the servo object 
  motor_3.attach(4);  // attaches the servo on pin 4 to the servo object 
  motor_4.attach(9);  // attaches the servo on pin 9 to the servo object 

	Serial.begin(9600);
  Serial.println("Start power motor");
  Serial.println("Initialization with:");
  Serial.println(pwm_value, DEC);
  //motor_1.writeMicroseconds(pwm_value);
  motor_2.writeMicroseconds(pwm_value);
  //motor_3.writeMicroseconds(pwm_value);
  motor_4.writeMicroseconds(pwm_value);
  delay(10000);
  Serial.println("Finished initialization");
} 

void loop() 
{ 
	if(Serial.available() > 0)
  {
    // Read the keyboard value
    char ch = Serial.read();

    if (ch == 'u' || ch == 'U')
    {
      pwm_value += 1;
      Serial.println(pwm_value);
			motor_2.writeMicroseconds(pwm_value); 
			motor_4.writeMicroseconds(pwm_value);
    }
    if (ch == 'd' || ch == 'D')
    {
      pwm_value -= 1;
      Serial.println(pwm_value);
			motor_2.writeMicroseconds(pwm_value);
			motor_4.writeMicroseconds(pwm_value);
    }
  }
}
