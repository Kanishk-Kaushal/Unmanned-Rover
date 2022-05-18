#include "WiFi.h"

// NOTE: CURRENTLY IN TESTING AND PROTOTYPING PHASE 

/*

We require the rover to traverse autonomously when it gets the appropriate commands from the Nvidia Jetson TX2
and hence perform the desired tasks.
If and when the AI's algorithm messes up and the rover is in urgent need of manual maneuver during the mission, the rover should
switch to the Teleoperation Mode.

PROBLEM : When we run the "Obstacle Avoidance" programme, the data sent by the LIDAR interferes with the data being sent to the ESP32
          as both are running on a co-dependant communication environment.

SOLUTION : Implement Independant Communication Protocols for both of the type of datas ie data sent for Teleoperation and the data 
           sent for Autonomous Traversal.
           
FUTURE SCOPE: Run the whole progroamme on FreeRTOS and use unique threads for each of the mode

*/ 

#define RXD2 16 // USART 2nd Serial Port Receiver
#define TXD2 17 // USART 2nd Serial Port Transmitter

// Defines WiFi Client
WiFiClient client;

// Stores trash data
char trash; 

// Motor Mode Parameters
int gear_mm, x_mm, y_mm;

// Internet Parameters
const char* ssid = "wifi";
const char* password = "password";

// Time to implelement Fail Safe
unsigned long previousMillis = 0;
unsigned long interval = 15;

String request;

// Right Motor
int rpwm = 23;    // D23
int rdir = 22;    // D22
int dr = 0;       // dr -> Right Direction Scraped Value
int pwmr = 0;     // pwmr -> Right PWM
int mpwmr = 0;    // mpwmr -> Mapped Right PWM

// Left Motor
int lpwm = 33;    // D33
int ldir = 32;    // D32
int dl = 0;       // dl -> Left Direction Scraped Value
int pwml = 0;     // pwml -> Left PWM
int mpwml = 0;    // mpwml -> Mapped Left PWM

// Setting Right PWM properties
const int RightFreq = 30000;
const int RightPWMChannel = 0;
const int RightResolution = 16;
const int RIGHT_MAX_DUTY_CYCLE = (int)(pow(2, RightResolution) - 1);


// Setting Left PWM properties
const int LeftFreq = 30000;
const int LeftPWMChannel = 2;
const int LeftResolution = 16;
const int LEFT_MAX_DUTY_CYCLE = (int)(pow(2, LeftResolution) - 1);

// Scarping iteration variable
int i = 0;

// Last PWM Values
int Last_pwmr;
int Last_pwml;
int pr;
int pl;

// Defines Server | Port Number -> 5005
WiFiServer wifiServer(5005);

// UNCOMMENT THE FOLLOWING WHEN YOU REQUIRE STATIC IP ---- (1)
/*
// Set your Static IP address
IPAddress local_IP(192, 168, 1, 134);

// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

// Set your Subnet
IPAddress subnet(255, 255, 255, 0);

// Set your Primary DNS
IPAddress primaryDNS(8, 8, 8, 8);   //optional

// Set your Secondary DNS
IPAddress secondaryDNS(8, 8, 4, 4); //optional
*/

void setup() 
{

  Serial.begin(115200);

  delay(1000);  

  // Start USART Connectivity
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  
  // UNCOMMENT THE FOLLOWING WHEN YOU REQUIRE STATIC IP ---- (2)
  /*
  // Configures static IP Address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) 
  {
    Serial.println("STA Failed to configure");
  }
  */

  // Start WiFi Connectivity
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");
  Serial.println(WiFi.localIP());

  wifiServer.begin();

  // Sets the pins as outputs:
  pinMode(rpwm, OUTPUT);
  pinMode(rdir, OUTPUT);
  pinMode(lpwm, OUTPUT);
  pinMode(ldir, OUTPUT);
   
  // Configure Right LED PWM functionalitites
  ledcSetup(RightPWMChannel, RightFreq, RightResolution);

  // Configure Left LED PWM functionalitites
  ledcSetup(LeftPWMChannel, LeftFreq, LeftResolution);
  
  // Attach the channel to the GPIO to be controlled
  ledcAttachPin(rpwm, RightPWMChannel);
  ledcAttachPin(lpwm, LeftPWMChannel);
  
}

// Reads data over TCP/IP
char readchar_tcp(void)
{
  char req = client.read();
  return req;
}

// Reads data serially over USART
char readchar_usart(void)
{
  char req = Serial2.read();
  return req;
}

void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y, float gear) {
  if (DL == 1)
    digitalWrite(ldir, HIGH);   //Turn on LEFT LED
  else
    digitalWrite(ldir, LOW);   //Turn off LEFT LED

  if (DR == 1)
    digitalWrite(rdir, HIGH);   //Turn on RIGHT LED
  else
    digitalWrite(rdir, LOW);   //Turn off RIGHT LED

  ledcWrite(LeftPWMChannel, (uint32_t) abs(abs(a * X) - abs(b * Y)) * (gear * 0.1));
  ledcWrite(RightPWMChannel, (uint32_t) abs(abs(p * X) - abs(q * Y)) * (gear * 0.1));

  Serial.println("PWMR");
  Serial.println((uint32_t) abs(abs(p * X) - abs(q * Y)) * (gear * 0.1));
  Serial.println("PWML");
  Serial.println((uint32_t) abs(abs(a * X) - abs(b * Y)) * (gear * 0.1));
  Serial.println("DIRR");
  Serial.println(DR);
  Serial.println("DIRL");
  Serial.println(DL);
}

void MotorCode(int x, int y, float g) 
{
//void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y, int g)

  if (abs(x) < 20 && abs(y) < 20)   //No Motion
    Drive(0, 0, 0, 0, 0, 0, 0, 0, 0);

  else if (abs(x) < 10 && y < 0)   //Full Backward
    Drive(0, 0, 0, 1, 0, 1, x, y, g);

  else if (abs(x) < 10 && y > 0)   //Full Forward
    Drive(1, 1, 0, 1, 0, 1, x, y, g);

  else if (x < 0 && abs(y) <= 10)   //Spot Turn Left
    Drive(0, 1, 1, 0, 1, 0, x, y, g);

  else if (x > 0 && abs(y) <= 10)   //Spot Turn Right
    Drive(1, 0, 1, 0, 1, 0, x, y, g);

  else if (x > 0 && y > 0 && x > y)   //Octet 1
    Drive(1, 0, 1, 0, 1, 1, x, y, g);

  else if (x > 0 && y > 0 && x < y)   //Octet 2
    Drive(1, 1, 0, 1, 1, 1, x, y, g);

  else if (x < 0 && y > 0 && abs(x) < y)   //Octet 3
    Drive(1, 1, 1, 1, 0, 1, x, y, g);

  else if (x < 0 && y > 0 && abs(x) >= y)   //Octet 4
    Drive(0, 1, 1, 1, 1, 0, x, y, g);

  else if (x < 0 && y < 0 && abs(x) > abs(y))   //Octet 5
    Drive(0, 1, 1, 0, 1, 1, x, y, g);

  else if (x < 0 && y < 0 && abs(x) < abs(y))   //Octet 6
    Drive(0, 0, 0, 1, 1, 1, x, y, g);

  else if (x > 0 && y < 0 && abs(x) < abs(y))   //Octet 7
    Drive(0, 0, 1, 1, 0, 1, x, y, g);

  else if (x > 0 && y < 0 && abs(x) > abs(y))   //Octet 8
    Drive(1, 0, 1, 1, 1, 0, x, y, g);

  //Test Drive:
  //Drive(1,1,1,0,0,1,x,y,g);

}


void loop() 
{
  unsigned long currentMillis = millis();
   
  // If WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) 
  {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    previousMillis = currentMillis;
  }

  client = wifiServer.available();

  while(Serial2.available())
  {
    if (readchar_usart() == 'a')
    {
      pwmr = (readchar_usart()-'0')*100 + (readchar_usart()-'0')*10 + (readchar_usart()-'0');
      mpwmr = map(pwmr, 0, 255, 0, 65535);
      ledcWrite(RightPWMChannel, mpwmr);
      
      dr = readchar_usart() -'0';
      
      if(dr == 1)
      {
        digitalWrite(rdir, HIGH);
      }
      else
      {
        digitalWrite(rdir, LOW);
      }
      

      pwml = (readchar_usart() - '0')*100 + (readchar_usart()-'0')*10 + (readchar_usart()-'0');
      mpwml = map(pwml, 0, 255, 0, 65535);
      Last_pwml = mpwml;
      ledcWrite(LeftPWMChannel, mpwml);

      dl = readchar_usart()-'0';
                 
      if(dl == 1)
      {
        digitalWrite(ldir, HIGH);
      }
      else
      {
        digitalWrite(ldir, LOW);
      }
      
      Serial.println("PWMR");
      Serial.println(pwmr);
      Serial.println("PWML");
      Serial.println(pwml);
      Serial.println("DIRR");
      Serial.println(dr);
      Serial.println("DIRL");
      Serial.println(dl);
 
     }

     if(readchar_usart() == 'e')
     {
      if (client) 
      {

        while (client.connected()) 
        {
    
          while (client.available()>0) 
          {
            
           if (readchar_tcp() == 'm')
           {
            gear_mm = readchar_tcp();
    
            if(readchar_tcp() == 's')
            {
              x_mm = (readchar_tcp() - '0')*10000 + (readchar_tcp()-'0')*1000 + (readchar_tcp()-'0')*100 + (readchar_tcp()-'0')*10 + (readchar_tcp()-'0');
            }
    
            if(readchar_tcp() == 'f')
            {
              y_mm = (readchar_tcp() - '0')*10000 + (readchar_tcp()-'0')*1000 + (readchar_tcp()-'0')*100 + (readchar_tcp()-'0')*10 + (readchar_tcp()-'0');
            }
    
            trash = readchar_tcp();
    
            x_mm = x_mm - 8000;
            y_mm = y_mm - 8000;
    
           if (abs(x_mm) < 500)
            x_mm = 0;
           if (abs(y_mm) < 500)
            y_mm = 0;
    
            MotorCode(x_mm, y_mm, gear_mm);   //Run MotorCode 
            
           }
            
         }
          
        }
    
        client.stop();
    
        digitalWrite(rdir, LOW);
        digitalWrite(ldir, LOW);
        ledcWrite(LeftPWMChannel, 0);
        ledcWrite(RightPWMChannel, 0);
        
        Serial.println("Client disconnected");
     
      }              
     }
  }
}
