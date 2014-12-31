#include <Wire.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <L3G.h>
#include <LSM303.h>
#include <LPS331.h>
#include "main_control.h"

// L3G4200D gyro: 250 dps full scale
#define Gyro_Gain_FS1 0.00875 // dps/digit
#define Gyro_Gain_FS2 0.0175 // dps/digit
#define Gyro_Gain_FS3 0.07 // dps/digit

// LSM303 accelerometer: 2 g sensitivity
#define Accel_Gain_FS1 0.001 // g/digit
#define Compas_Gain_1 0.00094 // G/digit
#define Compas_Gain_2 0.00105 // G/digit

#define T_CONTROL 100.0 // milliseconds

// RF Communications
#define STATUS_LED_PIN 13 
#define rxPin 2     //D2(arduino) --> ACP220(TX)
#define txPin 3     //D3(arduino) --> ACP220(RX)

L3G gyro;
LSM303 compass;
LPS331 ps;

int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer1=0;   //general purpuse timer
long timer2=0;   //general purpuse timer
long timer_start=0;   //general purpuse timer

// Euler angles
float roll;
float pitch;
float yaw;

t_message message;
char message_aux[24];

SoftwareSerial apc220(rxPin, txPin);
t_motors motors;
t_control_acction acction;

t_angular_speed angular_speed;
t_angular_speed angular_speed_1;
t_acceleration acceleration;
t_acceleration north;
t_angular_position angular_position;
t_angular_position angular_position_1;
float altitude;

 
void setup()
{ 
  #if SHOW_WITH_IDE_ARDUINO == 1
  Serial.begin(9600);
  #endif
  apc220.begin(9600);
  pinMode (STATUS_LED_PIN,OUTPUT);  // Status LED
  digitalWrite(STATUS_LED_PIN,LOW);

  init_message();
  #if SHOW_WITH_IDE_ARDUINO == 1
  Serial.println("Pololu MinIMU-9 + Arduino AHRS");
  #endif
  apc220.println("Pololu MinIMU-9 + Arduino AHRS");

  init_motors(motors);
  init_control_acction(acction);

  init_angular_speed(angular_speed);
  init_angular_speed(angular_speed_1);
  init_acceleration(acceleration);
  init_acceleration(north);
  init_angular_position(angular_position);
  init_angular_position(angular_position_1);

  Wire.begin();

  init_gyro();
  init_accelerometers();
  init_altimeter();
 
  digitalWrite(STATUS_LED_PIN,HIGH);

  delay(20);
  timer_start = millis();
}

void loop()
{
  timer1 = millis();

  rf_module();

  imu_measurement();

  motor_reference(motors, 0);

  print_messages();

  timer2 = millis();

  if (timer2-timer1 > int(T_CONTROL))
  {
    #if SHOW_WITH_IDE_ARDUINO == 1
    Serial.println("The T_CONTROL time has been overflow. Review the control loop");
    #endif
    apc220.println("The T_CONTROL time has been overflow. Review the control loop");
  }
  else
    delay(int(T_CONTROL) - (timer2-timer1));
}

/** 
 * Evaluate the absolute angular position
 * @param void void 
 *
 * @return Void value is returned. 
**/
int imu_measurement(void)
{
  int ret = 0;

  gyro.read();
  compass.read();
  altitude = ps.pressureToAltitudeMeters(ps.readPressureMillibars());

  // dps 
  angular_speed.x = gyro.g.x*Gyro_Gain_FS2;
  angular_speed.y = gyro.g.y*Gyro_Gain_FS2;
  angular_speed.z = gyro.g.z*Gyro_Gain_FS2;

  // g
  acceleration.x = (compass.a.x >> 4)*Accel_Gain_FS1;
  acceleration.y = (compass.a.y >> 4)*Accel_Gain_FS1;
  acceleration.z = (compass.a.z >> 4)*Accel_Gain_FS1;

  // G
  north.x = compass.m.x*Compas_Gain_1;
  north.y = compass.m.y*Compas_Gain_1;
  north.z = compass.m.z*Compas_Gain_2;

  if(acceleration.x > 1.0) acceleration.x = 1.0;
  if(acceleration.y > 1.0) acceleration.y = 1.0;
  if(acceleration.z > 1.0) acceleration.z = 1.0;
  if(acceleration.x < -1.0) acceleration.x = -1.0;
  if(acceleration.y < -1.0) acceleration.y = -1.0;
  if(acceleration.z < -1.0) acceleration.z = -1.0;

  // The complementary filter
  float gain = 0;

    angular_position.x = gain*(angular_position_1.x + angular_speed.x*T_CONTROL/1000) + (1-gain)*acos(acceleration.z)*180/3.14;

  // Temporar memory system
  angular_speed_1.x = angular_speed.x;
  angular_speed_1.y = angular_speed.y;
  angular_speed_1.z = angular_speed.z;

  angular_position_1.x = angular_position.x;
  angular_position_1.y = angular_position.y;
  angular_position_1.z = angular_position.z;

  return (ret);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int init_message(void)
{
  message.id_window = 0;
  message.command = 0;
  message.time = 0;
  message.status = 0;

  return 0;
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/

int init_motors(t_motors motors)
{
  motors.one.attach(9);  //attaches the servo on pin 9 to the servo object
  motors.two.attach(10);  //attaches the servo on pin 10 to the servo object
  motors.three.attach(4);  // attaches the servo on pin 4 to the servo object
  motors.four.attach(5);  // attaches the servo on pin 5 to the servo object

  motors.one.write(0);
  motors.two.write(0);
  motors.three.write(0);
  motors.four.write(0);
  
  delay(4);

  motors.one.write(0);
  motors.two.write(0);
  motors.three.write(0);
  motors.four.write(0);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int init_control_acction(t_control_acction acction)
{
  int ret = 0;

  acction.pwm_value_1 = 0;
  acction.pwm_value_2 = 0;
  acction.pwm_value_3 = 0;
  acction.pwm_value_4 = 0;

  return (ret);
}

int init_acceleration(t_acceleration acceleration)
{
  int ret = 0;

  acceleration.x = 0.0;
  acceleration.y = 0.0;
  acceleration.z = 0.0;

  return (ret);
}
/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int init_angular_speed(t_angular_speed angular_speed)
{
  int ret = 0;

  angular_speed.x = 0.0;
  angular_speed.y = 0.0;
  angular_speed.z = 0.0;

  return (ret);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int init_angular_position(t_angular_position angular_position)
{
  int ret = 0;

  angular_position.x = 0.0;
  angular_position.y = 0.0;
  angular_position.z = 0.0;

  return (ret);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int init_gyro(void)
{
  int ret = 0;

  if (!gyro.init())
  {
    #if SHOW_WITH_IDE_ARDUINO == 1
    Serial.println("Failed to autodetect gyro type!");
    #endif
    apc220.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
  gyro.writeReg(L3G_CTRL_REG4, 0x10);

  return (ret);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int init_accelerometers(void)
{
  int ret = 0;

  compass.init();
  compass.enableDefault();

  compass.getDeviceType();
  #if SHOW_WITH_IDE_ARDUINO == 1
  Serial.println("Magnetometer type: ");
  Serial.println(compass.getDeviceType());
  #endif

  return (ret);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int init_brujula(void)
{
  int ret = 0;

  return (ret);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int init_altimeter(void)
{
  int ret = 0;
  
  altitude = 0.0;

  if (!ps.init())
  {
    #if SHOW_WITH_IDE_ARDUINO == 1
    Serial.println("Failed to autodetect pressure sensor!");
    #endif
    apc220.println("Failed to autodetect pressure sensor!");
    ret = -1;
  }

  ps.enableDefault();

  return (ret);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int motor_reference(t_motors motors, int value)
{
  int ret = 0;

  motors.one.write(value);
  motors.two.write(value);
  motors.three.write(value);
  motors.four.write(value);

  return (ret);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int rf_module(void)
{
  int n = apc220.available();

  if(n > 0)
  {
    apc220.readBytes(message_aux, n);
    n = 0;
  }

  return (n);
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int print_messages(void)
{
  int ret = 0;
  static char tbs_analog[77];

  //sprintf(tbs_analog, "S:%07.2f:%07.2f:%07.2f:A:%07.2f:%07.2f:%07.2f:C:%07.2f:%07.2f:%07.2f", 
  sprintf(tbs_analog, "S:%.2f:%.2f:%.2f:A:%.2f:%.2f:%.2f:C:%.2f:%.2f:%.2f", 
      angular_speed.x, 
      angular_speed.y, 
      angular_speed.z,
      acceleration.x, 
      acceleration.y, 
      acceleration.z,
      north.x, 
      north.y, 
      north.z);

  apc220.print(tbs_analog);
  apc220.println();

  #if ANGULAR_POSITION == 1
  sprintf(tbs_analog, "P:%07.2f:%07.2f:%07.2f:%07.2f:", 
      angular_position.x, 
      angular_position.y, 
      angular_position.z,
      altitude);

  #if SHOW_WITH_IDE_ARDUINO == 1
  Serial.print(tbs_analog);
  Serial.println();
  #endif
  apc220.print(tbs_analog);
  apc220.println();
  #endif

  #if ANGULAR_SPEED == 1
  sprintf(tbs_analog, "S: %07.2f ; %07.2f ; %07.2f ; %07.2f ;", 
      angular_speed.x, 
      angular_speed.y, 
      angular_speed.z,
      0.0);

  #if SHOW_WITH_IDE_ARDUINO == 1
  Serial.print(tbs_analog);
  Serial.println();
  #endif
  apc220.print(tbs_analog);
  apc220.println();
  #endif

  #if ACCELERATION == 1
  sprintf(tbs_analog, "A: %07.2f ; %07.2f ; %07.2f ; %07.2f ;", 
      acceleration.x, 
      acceleration.y, 
      acceleration.z,
      0.0);

  #if SHOW_WITH_IDE_ARDUINO == 1
  Serial.print(tbs_analog);
  Serial.println();
  #endif
  apc220.print(tbs_analog);
  apc220.println();
  #endif

  #if COMPASS == 1
  sprintf(tbs_analog, "N: %07.2f ; %07.2f ; %07.2f ; %07.2f ;", 
      north.x, 
      north.y, 
      north.z,
      0.0);

  #if SHOW_WITH_IDE_ARDUINO == 1
  Serial.print(tbs_analog);
  Serial.println();
  #endif
  apc220.print(tbs_analog);
  apc220.println();
  #endif

  return(ret);
}
