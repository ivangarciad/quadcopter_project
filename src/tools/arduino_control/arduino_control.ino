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

#define T_CONTROL 10.0 // milliseconds
#define PWM_OFFSET 0

// RF Communications
#define STATUS_LED_PIN 13 
#define rxPin 2     //D2(arduino) --> ACP220(TX)
#define txPin 3     //D3(arduino) --> ACP220(RX)

typedef struct t_datalog 
{
    char id;
    char id_2;
    int timestamp;
    int reference_pos;
    int angulo_Accx;
    int angulo_Accy;
    int angulo_Accz;                              
    int angulo_Gyrox;
    int angulo_Gyroy;
    int angulo_Gyroz;
    int angulo_Filtrox;
    int angulo_Filtroy;
    int angulo_Filtroz;   
} t_datalog;//definimos tipo struct t_datalog


int print_raw_imu_data(void);
void controller(int ref);
void init_datalog(t_datalog* datalog);
void send_datalog(t_datalog datalog);

L3G gyro;
LSM303 compass;
LPS331 ps;

int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long timer1=0;   //general purpuse timer
long timer2=0;   //general purpuse timer
long timer_printf=0;   //general purpuse timer

// Euler angles
float roll;
float pitch;
float yaw;

SoftwareSerial apc220(rxPin, txPin);
t_control_acction acction;

t_angular_speed angular_speed;
t_angular_speed angular_speed_kalman;
t_angular_speed angular_speed_1;
t_acceleration acceleration;
t_acceleration north;
t_angular_position angular_position;
t_angular_position angular_position_1;
t_kalman kalman;

float altitude;
float angle_pitch;

float kp, ki;

Servo one;
Servo two;
Servo three;
Servo four;

int stop = 0;

int value_1 = 0;  //Pos width us in PWM signal
int value_2 = 0;
int value_3 = 0;
int value_4 = 0;

int pwm_value = 0;
float action = 0;
int min_pwm_action = 700;

char tbs_analog[30];
 
void setup()
{ 
  Serial.begin(115200);
  //apc220.begin(9600);
  pinMode (STATUS_LED_PIN,OUTPUT);  // Status LED
  digitalWrite(STATUS_LED_PIN,LOW);

  #if SHOW_WITH_IDE_ARDUINO == 1
  Serial.println("Pololu MinIMU-9 + Arduino AHRS");
  #endif
  //apc220.println("Pololu MinIMU-9 + Arduino AHRS");
  Serial.println("Start Init process");

  // Init variables
  //init_control_action(acction);
  init_angular_speed(angular_speed);
  init_angular_speed(angular_speed_kalman);
  init_acceleration(acceleration);
  init_acceleration(north);
  init_angular_position(angular_position);

  // Init used interface for sensors
  Wire.begin();

  // Init sensores
  init_gyro();
  init_accelerometers();
  init_altimeter();
  init_motors();

  angle_pitch = 0.0;
  kp = 0.2663;
  ki = 0.032;
 
  digitalWrite(STATUS_LED_PIN,HIGH);

  Serial.println("Init process finshed");
  int size = sizeof(t_datalog);
  Serial.println("sizeof(t_datalog");
  Serial.print(size);
  t_datalog datalog;
  init_datalog(&datalog);
  while(1)
    send_datalog(datalog);
}

void loop()
{
  static int ref = 0;

  timer1 = millis();

  input_references_and_commands();
  imu_measurement();


  //if(Serial.available()) 
  if(0) 
  {
    Serial.println("UART data received");
    value_4 = Serial.parseInt();    // Parse an Integer from Serial
    Serial.println(value_4);

    one.writeMicroseconds(value_4);
    three.writeMicroseconds(value_4);
    two.writeMicroseconds(value_4);
    four.writeMicroseconds(value_4);
  }
  else
  {
     controller(0);
     two.writeMicroseconds(min_pwm_action - action);
     four.writeMicroseconds(min_pwm_action + action);
  }
  if ((millis() - timer_printf) > 1000)
  {
    //Serial.println(angular_speed.y*10, DEC);
    print_raw_imu_data();
  //  timer_printf = millis();
  }

  timer2 = millis();

  if (timer2-timer1 > int(T_CONTROL))
  {
    //Serial.println("The T_CONTROL time has been overflow. Review the control loop");
  }
  else
    delay(int(T_CONTROL) - (timer2-timer1));
}

/** 
 * Controller pid
 * @param void void 
 *
 * @return Void value is returned. 
**/
void controller(int ref)
{
  float error = ref - angle_pitch;

  float proporcional = kp*error;

  float integral_error =+ error*T_CONTROL;
  float integral = ki*integral_error;

  action =  proporcional + integral;
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

  // g(m/ss)
  acceleration.x = (compass.a.x >> 4)*Accel_Gain_FS1;
  acceleration.y = (compass.a.y >> 4)*Accel_Gain_FS1;
  acceleration.z = (compass.a.z >> 4)*Accel_Gain_FS1;

  // G(gauss)
  north.x = compass.m.x*Compas_Gain_1;
  north.y = compass.m.y*Compas_Gain_1;
  north.z = compass.m.z*Compas_Gain_2;

  // The complementary filter
  //angular_position.x = complementary_filter(angular_position_1.x, angular_speed.x, acceleration.z);

  // The Kalman filter
  kalman = kalman_filter(angular_speed.x);

  // Temporar memory system
  angular_speed_1.x = angular_speed.x;
  angular_speed_1.y = angular_speed.y;
  angular_speed_1.z = angular_speed.z;

  angular_position_1.x = angular_position.x;
  angular_position_1.y = angular_position.y;
  angular_position_1.z = angular_position.z;

  return (ret);
}

float complementary_filter(float angular_position_prev, float angular_speed, float acceleration)
{
  float ret = 0;
  float gain = 0;
  
  ret = gain*(angular_position_prev + angular_speed*T_CONTROL/1000) + (1-gain)*acos(acceleration)*180/3.14;

  return ret;
}

t_kalman kalman_filter(float measurement)
{

  t_kalman kalman_ret;

  float desv_sensor = 0.2;
  float desv_system = 0.0;
  float A = 1.05;

  float var_sensor = desv_sensor * desv_sensor;
  float var_system = desv_system * desv_system;
  
  static float x_k_1 = 0.0;
  static float P_k_1 = 1.0;

  // Predicción
  float x__k = x_k_1;
  float P__k = A*P_k_1 + var_system;

  // Corrección
  float K_k = P__k / (P__k + var_sensor);
  float x_k = x__k + K_k * (measurement - x__k);
  float P_k = (1 - K_k)*P__k;

  x_k_1 = x_k;
  P_k_1 = P_k;

  kalman_ret.x_k = x_k;
  kalman_ret.K_k = K_k;
  kalman_ret.P_k = P_k;

  return kalman_ret;
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/

int init_motors(void)
{
  one.attach(5);  //attaches the servo on pin 9 to the servo object
  two.attach(10);  //attaches the servo on pin 10 to the servo object
  three.attach(4);  // attaches the servo on pin 4 to the servo object
  four.attach(9);  // attaches the servo on pin 5 to the servo object

  Serial.println("Conect ESC power");

  one.writeMicroseconds(pwm_value);
  two.writeMicroseconds(pwm_value);
  three.writeMicroseconds(pwm_value);
  four.writeMicroseconds(pwm_value);
  delay(10000);
  Serial.println("Finished motor initialization");
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
    //apc220.println("Failed to autodetect gyro type!");
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
    //apc220.println("Failed to autodetect pressure sensor!");
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
void motor_reference(int value)
{
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
void input_references_and_commands(void)
{
  if(Serial.available() > 0)
  {
    char aux = Serial.read();

    switch (aux)
    {
      case '1':
        /* Incrementar velocidad motor 1 */
        value_1 += 1;
        one.writeMicroseconds(value_1);
        Serial.print("Motor one ");
        Serial.println(value_1);
        break;
      case 'q':
        /* Decrementar velocidad motor 1 */
        value_1 -= 1;
        one.writeMicroseconds(value_1);
        Serial.print("Motor one ");
        Serial.println(value_1);
        break;
      case '2':
        /* Incrementar velocidad motor 2 */
        value_2 += 1;
        two.writeMicroseconds(value_2);
        Serial.print("Motor two ");
        Serial.println(value_2);
        break;
      case 'w':
        /* Decrementar velocidad motor 2 */
        value_2 -= 1;
        two.writeMicroseconds(value_2);
        Serial.print("Motor two ");
        Serial.println(value_2);
        break;
      case '3':
        /* Incrementar velocidad motor 3 */
        value_3 += 1;
        three.writeMicroseconds(value_3);
        Serial.print("Motor three ");
        Serial.println(value_3);
        break;
      case 'e':
        /* Decrementar velocidad motor 3 */
        value_3 -= 1;
        three.writeMicroseconds(value_3);
        Serial.print("Motor three ");
        Serial.println(value_3);
        break;
      case '4':
        /* Incrementar velocidad motor 4 */
        value_4 += 1;
        four.writeMicroseconds(value_4);
        Serial.print("Motor four ");
        Serial.println(value_4);
        break;
      case 'r':
        /* Decrementar velocidad motor 4 */
        value_4 -= 1;
        four.writeMicroseconds(value_4);
        Serial.print("Motor four ");
        Serial.println(value_4);
        break;
      case '0':
        /* Stop */
        kp = 0;
        ki = 0;
        action = 0;
        min_pwm_action = 0;
        Serial.println("Parada emergencia");
        break;
      case 'P':
        kp += 0.001;
        break;
      case 'p':
        kp -= 0.001;
        break;
      case 'I':
        ki += 0.001;
        break;
      case 'i':
        ki -= 0.001;
        break;
    }
  }
}

/** 
 * Brieft coment about the function. 
 * @param x x coment
 * @param y y coment
 * @param st st coment
 *
 * @return Void value is returned. 
**/
int print_raw_imu_data(void)
{
  int ret = 0;

 //sprintf(tbs_analog, "S:%.2f:%.2f:%.2f:A:%.2f:%.2f:%.2f:C:%.2f:%.2f:%.2f:Al:%.2f:Ac:%.2f", 
 //    angular_speed.x, 
 //    angular_speed.y, 
 //    angular_speed.z,
 //    acceleration.x, 
 //    acceleration.y, 
 //    acceleration.z,
 //    north.x, 
 //    north.y, 
 //    north.z,
 //    altitude,
 //    action);

// sprintf(tbs_analog, "S:%7.2f:%7.2f:%7.3f:%7.3f:%7.3f", 
//     acceleration.y,
//     angle_pitch,
//     kp,
//     ki,
//     action);

  sprintf(tbs_analog, "S:%7.2f:%7.5f:%7.5f:%7.5f:%7.2f", 
      angular_speed.x,
      kalman.x_k,
      kalman.K_k,
      kalman.P_k,
      action);

  //#if SHOW_WITH_IDE_ARDUINO == 1
  Serial.print(tbs_analog);
  Serial.println();
  //#endif

  return(ret);
}
void send_datalog(t_datalog datalog)
{
  Serial.write((uint8_t*) &datalog, sizeof(t_datalog));
}

void init_datalog(t_datalog* datalog)
{
    datalog->id = 0x21;
    datalog->id_2 = 0x22;
    datalog->timestamp = 0;
    datalog->reference_pos = 1;
    datalog->angulo_Accx = 2;
    datalog->angulo_Accy = 3;
    datalog->angulo_Accz = 4;
    datalog->angulo_Gyrox = 0;
    datalog->angulo_Gyroy = 0;
    datalog->angulo_Gyroz = 0;
    datalog->angulo_Filtrox = 0;
    datalog->angulo_Filtroy = 0;
    datalog->angulo_Filtroz = 0;

}
