#ifndef MAIN_CONTROL_H
#define MAIN_CONTROL_H

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define Gyro_Scaled(x) ((x)*ToRad(Gyro_Gain)) //Return the scaled ADC raw data of the gyro in radians for second

#define ANGULAR_POSITION 0 
#define ANGULAR_SPEED 0 
#define ACCELERATION 0 
#define COMPASS 0 
#define SHOW_WITH_IDE_ARDUINO 0 

typedef struct t_motors
{
  Servo one;
  Servo two;
  Servo three;
  Servo four;
};

typedef struct t_control_acction
{
  unsigned int pwm_value_1;
  unsigned int pwm_value_2;
  unsigned int pwm_value_3;
  unsigned int pwm_value_4;
};

typedef struct t_acceleration
{
  float x;
  float y;
  float z;
};

typedef struct t_angular_speed
{
  float x;
  float y;
  float z;
};

typedef struct t_angular_position
{
  float x;
  float y;
  float z;
};

typedef struct t_message
{
   long int id_home;
   long int id_window;
   long int command;
   long int time;
   long int status;
   long int ack;
};

int init_motors(t_motors motors);
int init_control_acction(t_control_acction acction);
int init_acceleration(t_acceleration acceleration);
int init_angular_speed(t_angular_speed angular_speed);
int init_angular_position(t_angular_position angular_position);
int init_gyro(void);
int init_accelerometers(void);
int init_brujula(void);
int init_altimeter(void);

int motor_reference(t_motors motors, int value);
int rf_module(void);

int imu_measurement(void);
int init_message(void);

#endif
