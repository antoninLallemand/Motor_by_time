#include <Arduino.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <string.h>

#define GMT_OFFSET 1  //France : GMT+1
#define ASCII_OFFSET 0x30 //link ascii digits to integer digits

String formattedTime; //gmt time in string format
uint8_t integerTime[3]; //hour, minute, second integer array
enum time {hour, minute, second}; //names previous array cells

const char *SSID = "Redmi 10C"; //Redmi 10C
const char *PWD = "ogre3187"; //ogre3187
unsigned long lastMillis = 0; //wifi retry variable

WiFiUDP ntpUDP; //define WifiUDP Class instance
NTPClient timeClient(ntpUDP, "pool.ntp.org");//define NTPClient Class instance

#define FCLK 80000000UL //basic clock frquency
#define PRESCALER 8000  //divides the clock frquency
#define Te 2 //checking period in s
const uint16_t nbOfTicks = FCLK/PRESCALER*Te; //definition of number of ticks with previous parameters

hw_timer_t *timer = NULL; //creates an object "timer" which contain settings to apply as a pointer on a hardware timer

bool timeCheck = false;

/* Function executed once each interrupt */
void Timer_ISR(){
  timeCheck = true;  //let active time collection
}

String get_wifi_status(int status){
  switch(status){
    case WL_IDLE_STATUS:
      return "WL_IDLE_STATUS";
    case WL_SCAN_COMPLETED:
      return "WL_SCAN_COMPLETED";
    case WL_NO_SSID_AVAIL:
      return "WL_NO_SSID_AVAIL";
    case WL_CONNECT_FAILED:
      return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:
      return "WL_CONNECTION_LOST";
    case WL_CONNECTED:
      return "WL_CONNECTED";
    case WL_DISCONNECTED:
      return "WL_DISCONNECTED";
    default :
      return "";
    break;
  }
}

void connectToWiFi(){
  int status = WL_IDLE_STATUS;
  Serial.print("Connecting to WiFi : ");
  Serial.println(get_wifi_status(status));
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(SSID, PWD);
  Serial.println(SSID);
  lastMillis = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
    status = WiFi.status();
    Serial.println(get_wifi_status(status));
    if (millis() - lastMillis > 20000)
      ESP.restart();
  }

  Serial.println("Connection success.");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());  
}

void getTime(uint8_t *intTime){
  formattedTime = timeClient.getFormattedTime();  //return gmt time in string format
  char gmtTime[formattedTime.length()+1];
  strcpy(gmtTime, formattedTime.c_str()); //convert string formated time to a char array
  /*fill in an integer array with time from previous char array*/
  intTime[hour] = (gmtTime[0]-ASCII_OFFSET)*10 + gmtTime[1]-ASCII_OFFSET + GMT_OFFSET;
  intTime[minute] = (gmtTime[3]-ASCII_OFFSET)*10 + gmtTime[4]-ASCII_OFFSET;
  intTime[second] = (gmtTime[6]-ASCII_OFFSET)*10 + gmtTime[7]-ASCII_OFFSET;
}

/*Definition of 3 deadlines activating the system*/
const uint8_t deadLines[3][3] = {
  {9, 30, 0},
  {14, 0, 0},
  {18, 30, 0}
};

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Motion (l393d) ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

#define INPUT1 3 //connected to l293 pin 2 
#define INPUT2 4 //connected to l293 pin 7
#define ENABLE 5 //connected to l293 pin 1
#define PWM_FREQUENCY 1000
#define PWM_RESOLUTION 10 //10 bits resolution (0-1023)
#define PWM_CHANNEL 0 //use of Ledc channel 0


enum direction {clockwise, counterclockwise, stop}; //links 0,1,2 to different directions

/*function to be implemented in infinite loop*/
void motorMotion (uint8_t dir, uint16_t _10BitDutyCycle){

  /*set motor speed*/
  if (_10BitDutyCycle >= pow(2,PWM_RESOLUTION))
    _10BitDutyCycle = pow(2,PWM_RESOLUTION)-1;

  ledcWrite(ENABLE, _10BitDutyCycle);

  /*set motor rotation direction*/
  switch(dir){
    case clockwise :
      digitalWrite(INPUT1, 0);
      digitalWrite(INPUT2, 1);
    break;
    case counterclockwise :
      digitalWrite(INPUT1, 1);
      digitalWrite(INPUT2, 0);
    break;
    case stop :
      digitalWrite(INPUT1, 0);
      digitalWrite(INPUT2, 0);
    break;
    default :
      Serial.println("direction parameter you defined does not exixt !");
    break;
  }
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Execution ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void setup(){
  /*Serial monitor setup*/
  Serial.begin(115200);

  /*Wifi and NTP client setup*/
  connectToWiFi();
  timeClient.begin();

  /*timer setup*/
  timer = timerBegin(3, PRESCALER, true); //timer 3, prescaler : 8000, count up
  timerAttachInterrupt(timer, &Timer_ISR, true); //pointer on timer object, pointer on our interrupt function, append on rising edge (timer overflow in our case)
  timerAlarmWrite(timer, nbOfTicks, true); //pointer on timer object, overflow value on the timer, autoreload
  timerAlarmEnable(timer);

  /*Pinout setup*/
  pinMode(INPUT1, OUTPUT);
  pinMode(INPUT2, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  /*PWM settings*/
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(ENABLE, PWM_CHANNEL);
}

void loop(){
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection lost. Reconnecting...");
    connectToWiFi();
  }
  else if(timeCheck){ //execute only if the device is connected to wifi
    timeCheck = false;
    timeClient.update();
    getTime(integerTime);

    Serial.print(integerTime[hour]);
    Serial.print(':');
    Serial.print(integerTime[minute]);
    Serial.print(':');
    Serial.print(integerTime[second]);
    Serial.println();
  }
}

