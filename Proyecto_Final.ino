
#include "DHT.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>




#define TRIGGER_PIN  7
#define ECHO_PIN     6
#define LED1         4
#define LED2         5
#define MAX_DISTANCE 200
#define LIGHT_TRESHOLD 200
#define DISTANCE_TRESHOLD 50
#define TEMP_TRESHOLD 30


#define DHTIN 2     // what pin we're connected to
#define DHTOUT 3
#define DHTTYPE DHT11   // DHT 11 


int left_m1 = 12;
int left_m2 = 13;
int left_ms = 10;
int left_speed=80;

int right_m1 = 8;
int right_m2 = 11;
int right_ms = 9;
int right_speed=100;

long  US_distance;
int light_inten; 
volatile float t=0;
volatile float x_data=0;
volatile float y_data=0;
volatile float z_data=0;

// Connect pin 1 (on the left) of the sensor to +5V
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

DHT dht(DHTIN,DHTOUT, DHTTYPE);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);


void init_wifi(void);
char msg_in[32] = {0};
int idx = 0;

typedef enum {
  no_message,
  car_go_forward,
  car_go_backward,
  car_stop,
  car_left,
  car_right,
  max_message  
}messages_from_app_t;
messages_from_app_t parse_messages_from_app(void);


void setup()
{
  pinMode(left_m1,OUTPUT);
  pinMode(left_m2,OUTPUT);
  pinMode(left_ms,OUTPUT);
  
  pinMode(right_m1,OUTPUT);
  pinMode(right_m2,OUTPUT);
  pinMode(right_ms,OUTPUT);
  
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  digitalWrite(LED1,LOW);

  
  Serial.begin(115200);

  
   /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  
 
/* WIFI setup*/
  Serial1.begin(115200);
  init_wifi();
  
}
 
void loop()
{
    messages_from_app_t new_message;
  // Read temperature as Celsius
  t = dht.readTemperature();
  if(t<TEMP_TRESHOLD)
  {
      digitalWrite(4,LOW);
  }
  else
  {
     digitalWrite(4,HIGH);

  }
  // Monitonr ambient light
  light_inten = analogRead(0); //the light sensor is attached to analog 0
  if(light_inten<LIGHT_TRESHOLD)
  {
        digitalWrite(5,HIGH);
  }
  else
  {
    digitalWrite(5,LOW);

  }
  
  
  
  
  //Monitor distance
  US_distance =GetDistance();
  Serial1.print("D");
  Serial1.print( US_distance);
  Serial1.print("\r");
  Serial1.print("T");
  Serial1.print( t);
  Serial1.print("\r");
  //Serial.print("Distance: ");
  //Serial.print( US_distance);
  //Serial.println("cm");
   // Serial.print("Light: ");
  //Serial.println( light_inten);
  delay(10);
  
  //Read acelerometer
  sensors_event_t event; 
  accel.getEvent(&event);
  x_data= event.acceleration.x;
  y_data=event.acceleration.y;
  z_data=event.acceleration.z;
  
  /*
  Serial.print("Distance: ");
  Serial.print( US_distance);
  Serial.println("cm");
  Serial.print("Light: ");
  Serial.println( light_inten);
  Serial.print("Temp: ");
  Serial.println(t);
  Serial.print("X: ");
  Serial.println(x_data);
  Serial.print("Y: ");
  Serial.println(y_data);
  Serial.print("Z: ");
  Serial.println(z_data);
  */

  


  
   new_message = parse_messages_from_app();
  if(new_message == car_go_forward)   {  MotorFWD(100); }
  else if(new_message == car_go_backward)  {  MotorBWD(100); }
  else if(new_message == car_left)         {  MotorLeft(); }
  else if(new_message == car_right)        {  MotorRight(); }
  else if(new_message == car_stop)         {  MotorSTOP(); }
  //else {Serial.println("Default case");}
  
//Serial.print(new_message);
  /*
  if(US_distance<DISTANCE_TRESHOLD)
  {
        MotorSTOP();
        delay(3000);
        MotorBWD(100);
        delay(3000);
        MotorLeft();
        delay(250);

  }
  else
  {
      MotorFWD(100);

  }
*/

 /*
  MotorFWD(100);
  delay(5000);
  MotorBWD(100);
  delay(5000);
  MotorLeft();
  delay(5000);
  MotorRight();
  delay(5000);
*/
}

long GetDistance (void)
{
  long distance=0;
  long duration=0;
  digitalWrite(TRIGGER_PIN, LOW);  // Added this line
  delayMicroseconds(2); // Added this line
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10); // Added this line
  digitalWrite(TRIGGER_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration/2) / 29.1;
  return distance;

}
void MotorLeft()
{
  analogWrite(left_ms,40);
  digitalWrite(left_m2,HIGH);
  digitalWrite(left_m1,LOW);
  analogWrite(right_ms,100);
  digitalWrite(right_m2,LOW);
  digitalWrite(right_m1,HIGH);

}
void MotorRight()
{
  analogWrite(left_ms,100);
  digitalWrite(left_m2,HIGH);
  digitalWrite(left_m1,LOW);
  analogWrite(right_ms,40);
  digitalWrite(right_m2,LOW);
  digitalWrite(right_m1,HIGH);

}

void MotorBWD(int motor_speed)
{
  //FWD
  analogWrite(left_ms,motor_speed);
  digitalWrite(left_m2,HIGH);
  digitalWrite(left_m1,LOW);
  analogWrite(right_ms,motor_speed);
  digitalWrite(right_m2,LOW);
  digitalWrite(right_m1,HIGH);

}

void MotorFWD(int motor_speed)
{
  //BACKWARD
  analogWrite(left_ms,motor_speed);
  digitalWrite(left_m2,LOW);
  digitalWrite(left_m1,HIGH);
  analogWrite(right_ms,motor_speed);
  digitalWrite(right_m2,HIGH);
  digitalWrite(right_m1,LOW);


}
void MotorSTOP()
{
  analogWrite(left_ms,0);
  analogWrite(right_ms,0);

}
messages_from_app_t parse_messages_from_app(void)
{
  char in_byte;
  messages_from_app_t ret = no_message;
      Serial.print(">");

  while (Serial1.available() > 0) {
    // read the incoming byte:
    in_byte = Serial1.read();
    Serial.print(in_byte);
    msg_in[idx++] = in_byte;

    if(in_byte == '\n') {
      msg_in[idx] = 0;
      if (strcmp(msg_in, "FORWARD\r\n")  == 0){
        idx = 0;
        ret = car_go_forward;
        break;

      }
      else if (strcmp(msg_in, "BACKWARD\r\n")  == 0){
        idx = 0;
        ret = car_go_backward;
        break;

      }
      else if (strcmp(msg_in, "PARA\r\n")  == 0){
        idx = 0;
        ret = car_stop;
        break;

      }
      else if (strcmp(msg_in, "LEFT\r\n")  == 0){
        idx = 0;
        ret = car_left;
        break;

      }
      else if (strcmp(msg_in, "RIGHT\r\n")  == 0){
        idx = 0;
        ret = car_right;
        break;
      }
      idx = 0;
    }
  }
  return ret;
}



void init_wifi(void)
{
  char in_byte;
  
  //Serial.print("\r\nRunning the esp_01_test project.\n");

  //Serial.print(">+++");
  Serial1.print("+++");
  delay(2000);
  
  //Serial.print(">AT\r\n");  
  Serial1.print("AT\r\n");
  do {
    in_byte = Serial1.read();
    //Serial.write(in_byte);
  }while(in_byte != '\n');
  delay(1000);  
  
  //Serial.print(">AT+RST\r\n");
  Serial1.print("AT+RST\r\n");
  do {
    in_byte = Serial1.read();
    //Serial.write(in_byte);
  }while(in_byte != '\n');
  delay(4000);  
  
  //Serial.print(">ATE0\r\n");
  Serial1.print("ATE0\r\n");
  do {
    in_byte = Serial1.read();
    //Serial.write(in_byte);
  }while(in_byte != '\n');
  delay(1000);  

  //Serial.print(">AT+CWMODE=1\r\n");
  Serial1.print("AT+CWMODE=1\r\n");
  do {
    in_byte = Serial1.read();
    //Serial.write(in_byte);
  }while(in_byte != '\n');
  delay(1000);  
  
  //Serial.print(">AT+CWJAP=\"instru\",\"12345678\"\r\n"); 
  Serial1.print("AT+CWJAP=\"instru\",\"12345678\"\r\n");
  do {
    in_byte = Serial1.read();
    //Serial.write(in_byte);
  }while(in_byte != '\n');
  delay(8000);  
  
  //Serial.print(">AT+CIPSTART=\"TCP\",\"192.168.43.1\",5000\r\n");  
  Serial1.print("AT+CIPSTART=\"TCP\",\"192.168.43.1\",5000\r\n");
  do {
    in_byte = Serial1.read();
    //Serial.write(in_byte);
  }while(in_byte != '\n');
  delay(3000);  
  
  //Serial.print(">AT+CIPMODE=1\r\n");
  Serial1.print("AT+CIPMODE=1\r\n");
  do {
    in_byte = Serial1.read();
    //Serial.write(in_byte);
  }while(in_byte != '\n');
  delay(2000);  

  //Serial.print(">AT+CIPSEND\r\n");  
  Serial1.print("AT+CIPSEND\r\n");
  delay(2000);  

  //Serial.print(">Wifi Connected!\r\n");
  //Serial1.print("DEVICE_ON\r\n");
 
}
