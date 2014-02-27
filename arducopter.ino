#include <SoftwareSerial.h>
#include <Timer.h>
#include "/home/igdaza/domotica/src_python/message_def.h"

#define ID_MOTOR ID_HABITACION_7

#define setPin 2
#define openPin 3   //D3(arduino) --> IN1(Relay)
#define closePin 4  //D4(arduino) --> IN2(Relay)
#define enablePin 5
#define rxPin 6
#define txPin 7
#define defAudioPin A5

#define ledPin 12


typedef struct t_message
{
  long int id_home;
  long int id_window;
  long int command;
  long int time;
  long int status;
  long int ack;
};

t_message message;
t_message* p_message;

char message_aux[24];
bool to_do;
int status;
int count_bytes;
//int audioPin = defAudioPin;
//int audioDatos = 0;
//int index = 0;

void init_message();
void open();
void close();

SoftwareSerial apc220(rxPin, txPin);
Timer timer_riego;

void setup()
{  
  pinMode(ledPin, OUTPUT);
  //timer_riego.oscillate(ledPin, 10*1000, LOW); // 1 minute

  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);

  pinMode(setPin, OUTPUT);
  digitalWrite(setPin, HIGH);

  init_message();
  apc220.begin(9600);  
  Serial.begin(9600);  
  status = NULL_STATUS;
  to_do = false;
  count_bytes = 0;
  pinMode(openPin, OUTPUT);
  pinMode(closePin, OUTPUT);


}

void loop()
{   
  //audioDatos = analogRead(audioPin);
  //if (index < length)
  //{
  //   buffer_audio[index] = audioDatos;
  //Serial.println(buffer_audio[index]);
  //Serial.println(index);
  //   index++;
  //}
  //else
  //   index = 0;



  timer_riego.update();
  int n = apc220.available();

  if(n > 0)
  {
    apc220.readBytes((message_aux+count_bytes), n);

    count_bytes = count_bytes + n;
    n = 0;
  }

  if(count_bytes == sizeof(t_message))
  {
    count_bytes = 0;
    memcpy((char*)p_message, message_aux, sizeof(t_message));

    if(message.id_home == ID_HOME && 
      message.id_window == ID_MOTOR)
    {
      if (message.command == POWER_UP)
      {
        to_do = true;
        status = OPEN_STATUS;
        message.status = status;
        message.ack = OK;
      }
      else if(message.command == POWER_DOWN)
      {
        to_do = true;
        status = CLOSE_STATUS;
        message.status = status;
        message.ack = OK;
      }
      else if(message.command == GET_STATUS)
      {
        message.command = message.command;
        message.status = status;
        message.ack = OK;
      }
      else
        message.ack = NO_OK;


      apc220.write((uint8_t*) &message, sizeof(t_message));
    }

    Serial.println("Message information: ");
    Serial.println(message.id_home);
    Serial.println(message.id_window);
    Serial.println(message.command);
    Serial.println(message.time);
    Serial.println(message.status);
    Serial.println(message.ack);
    Serial.println();
  }

  if(to_do)
  {
    if (message.command == POWER_UP)
      open();
    else if (message.command == POWER_DOWN)
      close();

    to_do = false;
  }
}

void init_message()
{
  p_message = &message;
  message.id_home = 0;
  message.id_window = 0;
  message.command = 0;
  message.time = 0;
  message.status = 0;
  memset(&message_aux, '0', sizeof(message_aux));
}

void open()
{
  digitalWrite(openPin, HIGH);
  digitalWrite(ledPin, HIGH);
  delay(message.time);
  digitalWrite(ledPin, LOW);
  digitalWrite(openPin, LOW);
}

void close()
{
  digitalWrite(closePin, HIGH);
  digitalWrite(ledPin, HIGH);
  delay(message.time);
  digitalWrite(ledPin, LOW);
  digitalWrite(closePin, LOW);
}

