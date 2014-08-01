// this uses the Arduino servo library included with version 0012 

// caution, this code sweeps the motor up to maximum speed !
// make sure the motor is mounted securily before running.

#include <Servo.h> 

Servo myservo;
byte recibiendoByte;

int time_1 = 0;
int time_2 = 0;
int T = 100;
int delay_value = 0;
int angle = 0;
static int speed=335;

void arm(){
  // arm the speed controller, modify as necessary for your ESC  
  setSpeed(0); 
  delay(1000); //delay 1 second,  some speed controllers may need longer
}


void setup()
{
  myservo.attach(9);
  Serial.begin(9600);
  //arm();  
}

void setSpeed(int speed){
  // speed is from 0 to 100 where 0 is off and 100 is maximum speed
  // the following maps speed values of 0-100 to angles from 0-180,
  // some speed controllers may need different values, see the ESC instructions
  angle = map(speed, 0, 1023, 0, 179);
  Serial.println (speed);
  Serial.println (angle);
  myservo.write(angle);    
}

void loop()
{
  time_1 = millis();
  setSpeed(OrdenSubirBajar());
  time_2 = millis();

  
  delay_value = T - (time_2 - time_1);
  delay(delay_value);
} 

int OrdenSubirBajar()
{

  if (Serial.available() > 0) 
  {
     recibiendoByte = Serial.read(); // Leemos el Byte recibido
     //Serial.println(recibiendoByte);
     
     if (recibiendoByte == 65 || recibiendoByte == 97)
     {  // A o a
        Serial.println("SUBIR");
        speed += 1;
     }
    if (recibiendoByte == 90 || recibiendoByte ==122) 
    { // Z o z
      Serial.println("BAJAR");
      speed -= 1;
    }
    if (recibiendoByte == 83  || recibiendoByte == 115)
    { // T o t
      Serial.println("Stop!!");
      speed = 35;
    }
  }
  return (speed);
}

