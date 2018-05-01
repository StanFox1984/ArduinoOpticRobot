/*
  Arduino Starter Kit example
 Project 6  - Light Theremin
 
 This sketch is written to accompany Project 6 in the
 Arduino Starter Kit
 
 Parts required:
 photoresistor
 10 kilohm resistor 
 piezo
 
 Created 13 September 2012
 by Scott Fitzgerald
 
 http://arduino.cc/starterKit
 
 This example code is part of the public domain 
*/

#include <EEPROM.h>

#define NUM_DIRS 4
#define NUM_SENSORS 4
#define CLOCK_DELAY 50
#define SLEEP_DURATION 1000
#define MAX_CMD_LEN 20
#define MOTOR_ON 1

// variable to hold sensor value
unsigned long _sensorValue[NUM_SENSORS];
unsigned long _sensorThreshold[NUM_SENSORS];
int self_control = 2;
int sensorValue=0;
unsigned long last_light[NUM_SENSORS];
// variable to calibrate low value
int sensorLow = 1023;
// variable to calibrate high value
int sensorHigh = 0;
// LED pin
unsigned long per_millis = 0;
const int ledPin = 13;
unsigned long lastMillis = 0;
unsigned long lastSleep = 0;
int MotorPin = 9;
int MotorPin2 = 10;
int last_motor_state = LOW;
int first_s = 1;
int enabled[NUM_SENSORS];
int last_pitch = 0;
int is_diff = 0;
char s = ' ';
int pitch = 0;
unsigned long last_mi;
int last_motor_state2 = LOW;

int sensorValue2=0;
unsigned long _last_value[NUM_SENSORS];
char cmd[20];
int cnt = 0;
unsigned long cmd_timeout = 0;
String cmd_string = "";
unsigned long onduration = 0;
unsigned long defduration = 100;
unsigned long duration90 = 0;
unsigned long duration180 = 0;
float duration360=290;
float duration = 0;
int current_direction = 0;
int desired_direction = 0;
unsigned long sensorValueLeft[NUM_DIRS];
unsigned long sensorValueRight[NUM_DIRS];
const int SCAN_STATE = 0;
const int WAIT_STATE = 1;
const int TURN_STATE = 2;
const int PERFORMING_STATE = 3;
int  ffirst =0;
int max_direction = 0;
unsigned long last_check = 0;
unsigned long self_period = 20000;
int self_state = SCAN_STATE;
int self_last_state = 0;
int o1 = 0;
int o2 = 0;
unsigned long threshold = 150;
#define LEFT90 0x01
#define RIGHT90 0x02
#define RIGHT180 0x03
#define FORWARD 0x04
#define OBSTACLE 0x05
unsigned long eeprom_wr_ptr=255;
unsigned long eeprom_rd_ptr=255;
unsigned long _eeprom_rd_ptr=255;
int write_event(unsigned char type, unsigned long light)
{
  int i=0;
  char _light[4];
  switch(type)
  {
        case LEFT90:
        Serial.write((String("type: ")+String("LEFT90")+String("light: ")+String(light)).c_str());
        break;
        case RIGHT90:
        Serial.write((String("type: ")+String("RIGHT90")+String("light: ")+String(light)).c_str());
        break;
        case RIGHT180:
        Serial.write((String("type: ")+String("RIGHT180")+String("light: ")+String(light)).c_str());
        break;
        case FORWARD:
        Serial.write((String("type: ")+String("FORWARD")+String("light: ")+String(light)).c_str());
        break;
        case OBSTACLE:
        Serial.write((String("type: ")+String("OBSTACLE")+String("light: ")+String(light)).c_str());
        break;
  }
  _light[0] = light & 0xff;
  _light[1] = (light & 0xff00)>>8;
  _light[2] = (light & 0xff0000)>>16;
  _light[3] = (light & 0xff000000)>>24;
  eeprom_wr_ptr = EEPROM.read(0);
  eeprom_rd_ptr = EEPROM.read(1);
  if((eeprom_wr_ptr < 2)||(eeprom_wr_ptr >= 201))
  {
      eeprom_wr_ptr = 2;
  }
  if((eeprom_rd_ptr < 2)||(eeprom_rd_ptr >= 201))
  {
      eeprom_rd_ptr = 2;
  }
  EEPROM.write(eeprom_wr_ptr, type);
  eeprom_wr_ptr = (eeprom_wr_ptr + 1) % 201;
  if(eeprom_wr_ptr == 0)
    eeprom_wr_ptr = 2;
  for(i=0;i<4;i++)
  {
      EEPROM.write(eeprom_wr_ptr, _light[i]);  
      eeprom_wr_ptr = (eeprom_wr_ptr + 1) % 201;
      if(eeprom_wr_ptr == 0)
        eeprom_wr_ptr = 2;
  }
  Serial.write((String("Wrote event type:")+String(type)).c_str());
  Serial.write(String("\n").c_str());
  Serial.write((String("Wrote event light:")+String(light)).c_str());
  Serial.write(String("\n").c_str());
  Serial.write((String("Wrote event eeprom_ptr:")+String(eeprom_wr_ptr)).c_str());
  Serial.write(String("\n").c_str());
  EEPROM.write(0, (unsigned char)eeprom_wr_ptr);
  return 1;
}

void clear_events(void)
{
  EEPROM.write(1, (unsigned char)eeprom_rd_ptr);
  EEPROM.write(0, (unsigned char)eeprom_wr_ptr);
  Serial.write((String("Cleared events")).c_str());
  Serial.write(String("\n").c_str());
}

int read_event(unsigned char *type, unsigned long *light)
{
  int i=0;
  unsigned char _light[4];
  unsigned long t = 0;
  eeprom_rd_ptr = EEPROM.read(1);
  eeprom_wr_ptr = EEPROM.read(0);
  if((eeprom_rd_ptr < 2)||(eeprom_rd_ptr >= 201))
  {
      eeprom_rd_ptr = 2;
  }
  if((eeprom_wr_ptr < 2)||(eeprom_wr_ptr >= 201))
  {
      eeprom_wr_ptr = 2;
  }
  if(eeprom_rd_ptr == eeprom_wr_ptr)
  {
     return 0;
  }
  *type=EEPROM.read(eeprom_rd_ptr);
  eeprom_rd_ptr = (eeprom_rd_ptr + 1) % 201;
  if(eeprom_rd_ptr == 0)
    eeprom_rd_ptr = 2;
  for(i = 0;i<4;i++)
  {
      _light[i] = EEPROM.read(eeprom_rd_ptr);  
      eeprom_rd_ptr = (eeprom_rd_ptr + 1) % 201;
      if(eeprom_rd_ptr == 0)
        eeprom_rd_ptr = 2;
  }
  *light = 0;
  t |= _light[0];
  *light = t;
  t = 0;
  t |= _light[1];
  t = t <<8;
  *light |= t;
  t = 0;
  t |= _light[2];
  t = t <<16;
  *light |= t;
  t = 0;
  t |= _light[3];
  t = t <<24;
  *light |= t;
  //Serial.write((String("Read event eeprom_ptr:")+String(eeprom_rd_ptr)).c_str());
  //Serial.write(String("\n").c_str());
  EEPROM.write(1, (unsigned char)eeprom_rd_ptr);
  return 1;
}


void TurnLeft90(void)
{
  //Serial.write(cmd);
    if(self_state == PERFORMING_STATE)
      return;
    delay(500);
    onduration = duration360/4;
    Serial.write(String(onduration).c_str());
    last_motor_state = HIGH;
    last_mi = millis();
    if(MOTOR_ON)
        digitalWrite(MotorPin, last_motor_state);
    self_last_state = self_state;
    self_state = PERFORMING_STATE;
    Serial.write(String("PERFORMING_STATE").c_str());
    Serial.write(String("\n").c_str());
    
    s = 'k';
}
void TurnRight90(void)
{
  //Serial.write(cmd);
    if(self_state == PERFORMING_STATE)
      return;
    delay(500);
    onduration = duration360/4;
    
    last_motor_state2 = HIGH;
    last_mi = millis();
    if(MOTOR_ON)
    digitalWrite(MotorPin2, last_motor_state2);
    self_last_state = self_state;
    self_state = PERFORMING_STATE;
    Serial.write(String("PERFORMING_STATE").c_str());
    Serial.write(String("\n").c_str());
    
    s = 'k';
}
void TurnRight45(void)
{
  //Serial.write(cmd);
    if(self_state == PERFORMING_STATE)
      return;
    delay(500);
    onduration = (duration - duration/3)/2;
    last_motor_state2 = HIGH;
    last_mi = millis();
    if(MOTOR_ON)
      digitalWrite(MotorPin2, last_motor_state2);
    self_last_state = self_state;
    self_state = PERFORMING_STATE;
    Serial.write(String("PERFORMING_STATE").c_str());
    Serial.write(String("\n").c_str());
    
    s = 'k';
}
void TurnRight180(void)
{
  //Serial.write(cmd);
    if(self_state == PERFORMING_STATE)
      return;
    delay(500);
    onduration = (duration - duration/3)*8;
    last_motor_state2 = HIGH;
    last_mi = millis();
    if(MOTOR_ON)
    digitalWrite(MotorPin2, last_motor_state2);
    self_last_state = self_state;
    self_state = PERFORMING_STATE;
    Serial.write(String("PERFORMING_STATE").c_str());
    Serial.write(String("\n").c_str());
    
    s = 'k';
}
void TurnRight270(void)
{
  //Serial.write(cmd);
    //if(self_state == PERFORMING_STATE)
     // return;
    delay(500);
    onduration = (duration - duration/3)*3;
    last_motor_state2 = HIGH;
    last_mi = millis();
    if(MOTOR_ON)
    digitalWrite(MotorPin2, last_motor_state2);
    self_last_state = self_state;
    self_state = PERFORMING_STATE;
    Serial.write(String("PERFORMING_STATE").c_str());
    Serial.write(String("\n").c_str());
    
    s = 'k';
}
void Forward(void)
{
    //Serial.write(cmd);
    if(self_state == PERFORMING_STATE)
      return;
    delay(500);
    onduration = duration+duration;
    last_mi = millis();
    last_motor_state2 = HIGH;
    if(MOTOR_ON)
    digitalWrite(MotorPin2, last_motor_state2);
    last_motor_state = HIGH;
    if(MOTOR_ON)
    digitalWrite(MotorPin, last_motor_state);
    self_last_state = self_state;
    self_state = PERFORMING_STATE;
    Serial.write(String("PERFORMING_STATE").c_str());
    Serial.write(String("\n").c_str());
    
    s = 'k';
}

void setup() {
  // Make the LED pin an output and turn it on
  pinMode(ledPin, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(ledPin, HIGH);
  digitalWrite(MotorPin, LOW);
  digitalWrite(10, LOW);
  // calibrate for the first five seconds after program runs
  while (millis() < 1000) {
    // record the maximum sensor value
    sensorValue = analogRead(A0);
    if (sensorValue > sensorHigh) {
      sensorHigh = sensorValue;
    }
    // record the minimum sensor value
    if (sensorValue < sensorLow) {
      sensorLow = sensorValue;
    }
    lastMillis = millis();
    
  }
  digitalWrite(MotorPin, LOW);
  digitalWrite(10, LOW);
  // turn the LED off, signaling the end of the calibration period
  digitalWrite(ledPin, LOW);
  Serial.begin(9600);
  per_millis = millis();
  onduration = defduration;
    //read the input from A0 and store it in a variable
  duration90 = duration360/4.00;
  duration180 = duration360/2.00;
  //Serial.write(String(duration90).c_str());
  //Serial.write(String(duration180).c_str());
  duration = (unsigned long)(duration360/(float)NUM_DIRS);
  //Serial.write(String(duration).c_str());
  Serial.write(String((int)duration).c_str());
  Serial.write(String("\n").c_str());
  for(int i=0;i<NUM_SENSORS;i++)
  { 
    enabled[i] = 0;
  }
  enabled[0] = 1;
  enabled[1] = 1;
  enabled[2] = 1;
  enabled[3] = 1;
  lastSleep = millis();
  eeprom_rd_ptr = EEPROM.read(0);
  if((eeprom_rd_ptr < 1)||(eeprom_rd_ptr >= 201))
  {
    eeprom_rd_ptr = 1;
  }
  clear_events();
}
unsigned long max_light = 0;
int receive_command(char cmd_buf[MAX_CMD_LEN], int time_out)
{
  int last_millis = millis();
  cmd[0] = 0;
  while(Serial.available() && (cnt<19))
  {
     char ch = (char)Serial.read();
     s = ch;
     if(s == '\t' || s == '\r' || s == '\n')
       break;
     cmd[cnt]=s;
     //Serial.write(cmd[cnt]);
     cnt++;
     delay(10);
     
     if((millis() - last_millis)>time_out)
     {
        break;
     }
     delay(10);
  }
  if(cnt>(MAX_CMD_LEN-1))
  {
     return -1;
  }
  cmd[cnt] = '\0';
  return 0;
}
unsigned long sensor_ports[NUM_DIRS];

#define MAX_PROG_LEN 8

unsigned char program_batch[MAX_PROG_LEN];
unsigned char command_read_ptr = 0;
unsigned char command_write_ptr = 0;


void do_program(unsigned long *sensor_values, int num_sensors)
{
    while (command_read_ptr != command_write_otr)
    {
	switch(program_batch[command_ptr])
	{
	    case LEFT90:
			TurnLeft90();
	                write_event(LEFT90, max_light);
			break;
		case RIGHT90:
			TurnRight90();
	                write_event(RIGHT90, max_light);
			break;
		case FORWARD:
			Forward();
	                write_event(FORWARD, max_light);
			break;
	}
	command_read_ptr=(command_read_ptr + 1) % MAX_PROG_LEN;
    }
}

void add_command(unsigned char cmd)
{
    program_batch[command_write_ptr] = cmd;
    command_write_ptr=(command_write_ptr + 1) % MAX_PROG_LEN;
}

void refresh_sensors(unsigned long *sensor_values, unsigned long *sensor_ports, int num_sensors)
{
    int i;int t;
    sensor_ports[0] = A0;
    sensor_ports[1] = A1;
    sensor_ports[2] = A2;
    sensor_ports[3] = A4;

    for (t=0;t<3;t++) {
    for (i=0;i<num_sensors;i++)
    {
         if (t == 0) sensor_values[i] = 0;
        sensor_values[i] += analogRead(sensor_ports[i]);
        Serial.write((String("SensorVal:")+String(sensor_values[i])).c_str());
        Serial.write(String("\n").c_str());
    }
    delay(100);
    }
}

unsigned long get_light_sector(unsigned long start, unsigned long stop,
                               unsigned long *sensor_values,
                               int num_sensors)
{
    unsigned long native_sector_size = 360/num_sensors;
    unsigned long res = 0;
    int i;
    for (i=start;i<stop;i+=native_sector_size)
    {
        res += sensor_values[i/native_sector_size];
    }
    return res;
}

unsigned long get_best_light_sector(unsigned long *sensor_values,
                                    unsigned long sector_size,
                                    int num_sensors)
{
    unsigned long max_light = 0;
    unsigned long native_sector_size = 360/num_sensors;
    int i;
    unsigned long best_sector = 0;
    int sensors_per_sector = sector_size / native_sector_size;
    if (sector_size < native_sector_size)
      sector_size = native_sector_size;
    for(i=0;i<360;i+=sector_size)
    {
        unsigned long light_this_sector = 0;
        light_this_sector = get_light_sector(i, i+sector_size, sensor_values, num_sensors);
        if (max_light < light_this_sector) {
            max_light = light_this_sector;
            best_sector = i/sector_size;
        }
    }
    Serial.write((String("Best Light - Sector ")+String(best_sector)).c_str());
    Serial.write(String("\n").c_str());
    return best_sector;
}

void make_decision(unsigned long *sensor_values, int num_sensors)
{
        int obstacle = 0;
        for(int u=0;u<NUM_SENSORS;u++)
        {
            _last_value[u] = _sensorValue[u];
        }
        refresh_sensors(_sensorValue, sensor_ports, NUM_SENSORS);
        if(_last_value[1] > _sensorValue[1])
        if(abs(_last_value[1] - _sensorValue[1]) > 50)
        {
           o1 = 1;
        }
        if(_last_value[2] > _sensorValue[2])
        if(abs(_last_value[2] - _sensorValue[2]) > 50)
        {
           if((o1 == 1)&&(o2==0))
           {
             tone(8, 2000, 10);
             tone(8, 3000, 10);
             delay(1000);
             Serial.write((String("ObstacleDetected").c_str()));
             Serial.write(String("\n").c_str());
             obstacle = 1;
             o1 = 0;
             o2 = 1;
           }
        }
        current_direction = 0;
        n = get_best_light_sector(_sensorValue, 360/NUM_DIRS, NUM_SENSORS);
        last_check = millis();
	if (n != 1)
	    add_command(LEFT90);
	else
	    add_command(FORWARD);
	
        if(current_direction != desired_direction)
        {
           self_state = TURN_STATE;
           Serial.write("TURN_STATE");
           Serial.write("\n"); 
        }
        else
        {
           self_state = WAIT_STATE;
           Serial.write("WAIT_STATE");
           Serial.write("\n");
        }
        Serial.write((String("Current direction ")+String(current_direction)).c_str());
        Serial.write(String("\n").c_str());
        Serial.write((String("Desired direction ")+String(desired_direction)).c_str());
        Serial.write(String("\n").c_str());
        if(obstacle == 1)
        {
          TurnRight180();
          unsigned int cur_light0 = analogRead(A1)+analogRead(A2);
          write_event(OBSTACLE, cur_light0);
        }
}


void loop() {
  
  sensorValue = analogRead(A0);
  cnt = 0;
  receive_command(cmd, 1000);
  cmd_string = String(cmd);
  delay(100);
  if(!strcmp(cmd,"Ping"))
  {
    //Serial.write(cmd);
    delay(50);
    //read the input from A0 and store it in a variable
    Serial.write("Pong");
    s = 'k';
  }
  if(String(cmd) == "Sensor0")
  {
    //Serial.write(cmd);
    delay(50);
    //read the input from A0 and store it in a variable
    sensorValue = analogRead(A0);
    Serial.write((String("REPLY")+String(sensorValue)).c_str());
    s = 'k';
  }
  if(String(cmd) == "Sensor1")
  {
    //Serial.write(cmd);
    delay(50);
    //read the input from A0 and store it in a variable
    sensorValue = analogRead(A1);
    Serial.write(String(sensorValue).c_str());
    s = 'k';
  }
  if(String(cmd).startsWith("LeftMotorOn"))
  {
    //Serial.write(cmd);
    delay(50);
    last_motor_state = HIGH;
    digitalWrite(MotorPin, last_motor_state);
    last_mi = millis();
    s = 'k';
  }
  if(String(cmd).startsWith("LeftMotor90"))
  {
    //Serial.write(cmd);
    TurnLeft90();
  }
  if(String(cmd).startsWith("LeftMotor180"))
  {
    //Serial.write(cmd);
    delay(50);
    onduration = duration180;
    last_motor_state = HIGH;
    digitalWrite(MotorPin, last_motor_state);
    last_mi = millis();
    s = 'k';
  }
  if(String(cmd) == "LeftMotorOff")
  {
    //Serial.write(cmd);
    delay(50);
    last_motor_state = LOW;
    digitalWrite(MotorPin, last_motor_state);
    s = 'k';
  }
  if(String(cmd) == "RightMotorOn")
  {
    //Serial.write(cmd);
    delay(50);
    last_motor_state2 = HIGH;
    digitalWrite(MotorPin2, last_motor_state2);
    last_mi = millis();
    s = 'k';
  }
  if((self_control == 2)&&(self_state!=PERFORMING_STATE))
  {
    int n=0;
    int i=0;
    int right_left = 0;

    if(self_state == SCAN_STATE)
    {
	make_decision(_sensorValue, NUM_SENSORS);
    }
    else
    {
        do_program(_sensorValue, NUM_SENSORS);
        self_state = SCAN_STATE;
    }
  }
  if(String(cmd) == "RightMotorOff")
  {
    //Serial.write(cmd);
    delay(50);
    last_motor_state2 = LOW;
    digitalWrite(MotorPin2, last_motor_state2);
    s = 'k';
  }
  if(String(cmd) == "SelfControl")
  {
    //Serial.write(cmd);
    delay(50);
    self_state = SCAN_STATE;
    current_direction = 0;
    self_control = (self_control + 1) % 4;
    Serial.write(String(self_control).c_str());
    s = 'k';
  }
  if(String(cmd) == "ShowEvents")
  {
    int i=0;

    Serial.write((String("ShowEvents").c_str()));
    Serial.write(String("\n").c_str());
    unsigned long light=0;
    unsigned char type=0;
    unsigned char last_type = -1;
    unsigned long last_type_times = 1;
    while(read_event(&type,&light)==1)
    {
      if(last_type == -1)
        last_type = type;
      else
      {
        if(last_type != type)
        {
          Serial.write((String(" times: ")+String(last_type_times)+String("light: ")+String(light)).c_str());
          Serial.write(String("\n").c_str());
          last_type_times = 1;
          last_type = type;
        }
        else
        {
          last_type_times++;
          continue;
        }
      }
      switch(type)
      {
        case LEFT90:
        Serial.write((String("type: ")+String("LEFT90")+String("light: ")+String(light)).c_str());
        break;
        case RIGHT90:
        Serial.write((String("type: ")+String("RIGHT90")+String("light: ")+String(light)).c_str());
        break;
        case RIGHT180:
        Serial.write((String("type: ")+String("RIGHT180")+String("light: ")+String(light)).c_str());
        break;
        case FORWARD:
        Serial.write((String("type: ")+String("FORWARD")+String("light: ")+String(light)).c_str());
        break;
        case OBSTACLE:
        Serial.write((String("type: ")+String("OBSTACLE")+String("light: ")+String(light)).c_str());
        break;
      }
    }
    Serial.write((String(" times: ")+String(last_type_times)+String("light: ")+String(light)).c_str());
    Serial.write(String("\n").c_str());
    Serial.write((String("ShowEventsEND").c_str()));
    Serial.write(String("\n").c_str());
    delay(5000);
    s = 'k';
  }
  if(String(cmd) == "Beep")
  {
    //Serial.write(cmd);
    delay(50);
    tone(8, 2000, 10);
    s = 'k'; 
  }
  if((last_motor_state == HIGH) ||(last_motor_state2 == HIGH))
  {
     if((millis() - last_mi) > onduration)
     {
       last_motor_state = LOW;
       last_motor_state2 = LOW;
       if(self_state == PERFORMING_STATE)
       {
         self_state = self_last_state;
         Serial.write(String("PERFORMING_STATE to PREV").c_str());
         Serial.write(String("\n").c_str());
       }
       digitalWrite(MotorPin2, last_motor_state2);
       digitalWrite(MotorPin, last_motor_state);
       delay(100);
     }
  }
  if((millis() - lastSleep)<SLEEP_DURATION)
    delay(CLOCK_DELAY);
  else
  {
    if((last_motor_state != HIGH) &&(last_motor_state2 != HIGH))
    {
      delay(SLEEP_DURATION);
      lastSleep = millis();
    }
  }
}

