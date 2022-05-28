/*
Date : 15 May 2022

We require the rover to traverse autonomously when it gets the appropriate commands from the Nvidia Jetson TX2
and hence perform the desired tasks.
If and when the AI's algorithm messes up and the rover is in urgent need of manual maneuver during the mission, the rover should
switch to the Teleoperation Mode.

PROBLEM-1 : When we run the "Obstacle Avoidance" programme, the data sent by the LIDAR interferes with the data being sent to the ESP32
          as both are running on a co-dependant communication environment.
SOLUTION-1 : Implement Independant Communication Protocols for both of the type of datas ie data sent for Teleoperation and the data 
           sent for Autonomous Traversal.
           
PROBLEM-2 : The new PCB is yet to be made and hence "Pin-Remapping" mightbe an issue.
           
FUTURE UPDATE : The "LED Panel - Mode Detector" is yet to be interfaced. The Panel will showcase Blue while Teleoperation, Red while
                Autonomous Traversal and Flashing Green when the desired location is acheived.
           
FUTURE SCOPE: Run the whole progroamme on FreeRTOS and use unique threads for each of the mode

Date : 21 May 2022

1. The "LED Panel - Mode Detector" has been interfaced and runs on a core different than on which the driving programme runs using FreeRTOS.
2. PROBLEM-1 has been solved using a weird approach. Approach being converting the LIDAR programme from Python to C language
3. No more requirement for USART to drive the rover while in Teleoperation Mode.

Date : 28 May 2022

1. PROBLEM-2 has been solved.

*/ 

#include "WiFi.h"
#include <Adafruit_NeoPixel.h>

// Assinging LED Panel programme to the Core 0 of ESP32
TaskHandle_t LedPanel;

// Assinging Rover programme to the Core 1 of ESP32
TaskHandle_t UGVcontrol;

// Queues are required to communicate between Tasks in FreeRTOS
QueueHandle_t queue;
int queueSize = 1;

// Led Color Indicator
int LedFlag;

// Stores Incoming Data
char ch;

// Which pin on the ESP32 is connected to the NeoPixels -> D5
#define PIN          5

// How many NeoPixels LEDs are attached to the ESP32 -> 256
#define NUMPIXELS   256

// We define birghtness of NeoPixel LEDs
#define BRIGHTNESS  20

// Initialises the LED Matrix
Adafruit_NeoPixel matrix = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// Decalred as client
WiFiClient client;

// Reads Incoming Data
char readchar(void)
{
  char req = client.read();
  return req;
}

// Reads the Trash Value
char trash; 

// Internet Parameters
const char* ssid = "wifi";
const char* password = "password";

// Time to implelement Fail Safe
unsigned long previousMillis = 0;
unsigned long interval = 15;

String request;

// Motor - Teleoperation Mode Parameters
int gear_mm, x_mm, y_mm;

// Right Motor
int rpwm = 2;     // D2
int rdir = 15;    // D15
int dr = 0;       // dr -> Right Direction Scraped Value
int pwmr = 0;     // pwmr -> Right PWM
int mpwmr = 0;    // mpwmr -> Mapped Right PWM

// Left Motor
int lpwm = 16;    // D16 | Also Rx
int ldir = 4;     // D4
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

// Uncomment the following if you want to set up a static IP for the ESP ----(1)

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

  // Set NeoPixel configuration 
  matrix.setBrightness(BRIGHTNESS);

  // Start NeoPixel library with all LEDs off
  matrix.begin();

  // Show settings of LEDs in NeoPixel array
  matrix.show();

  // Declares the Queue
  queue = xQueueCreate( queueSize, sizeof( int ) );

  // FreeRTOS Task for the 0th Core
  xTaskCreatePinnedToCore(
                    UGVcontrolCode,   /* Task function. */
                    "UGVcontrol",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &UGVcontrol,      /* Task handle to keep track of created task */
                    1);      /* pin task to core 1 */  
                    
  // FreeRTOS Task for the 1st Core                    
  xTaskCreatePinnedToCore(
                    LedPanelCode,   /* Task function. */
                    "LedPanel",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &LedPanel,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */  

       

  delay(1000);  
  
  // Uncomment the following if you want to set up a static IP for the ESP ----(2)
  /*
  // Configures static IP Address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) 
  {
    Serial.println("STA Failed to configure");
  }
  */

  // Initialise the connectivity
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

// Function to Drive the Motors
void Drive(int DL, int DR, int a, int b, int p, int q, int X, int Y, float gear) 
{
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

// Function to declare the "Drive" function's paraameters
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

// Set the LED Panel to desired Color
void colorWipe(uint32_t c, uint8_t wait)
{
  for(uint16_t i=0; i<matrix.numPixels(); i++)
  {
    matrix.setPixelColor(i, c);
    matrix.show();
    delay(wait);
  }
}

// Blinks Green Color
void blink_green_x10(void) 
{
 
    colorWipe(matrix.Color(0, 255, 0), 0);
    delay(500);
    colorWipe(matrix.Color(0, 0, 0), 0);
    delay(500);
  
  
}

void UGVcontrolCode( void * pvParameters )
{
  for(;;)
  {
    /*
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

    WiFi.begin(ssid, password);
    */
  
    client = wifiServer.available();
  
    if (client) 
    {
  
      while (client.connected()) 
      {
  
        while (client.available()>0) 
        { 
          ch = readchar();
          switch(ch)
          {
            case 'a':
            {
              int ledRed = 0;
              xQueueSend(queue,&ledRed, 0);
              Serial.println("Enterred A");
              pwmr = (readchar()-'0')*100 + (readchar()-'0')*10 + (readchar()-'0');
              mpwmr = map(pwmr, 0, 255, 0, 65535);
              Last_pwmr = mpwmr;
              /*
              Serial.print("mpwmr = ");
              Serial.println(mpwmr);
              */
              ledcWrite(RightPWMChannel, mpwmr);
              
              dr = readchar() -'0';
              
              if(dr == 1)
              {
                digitalWrite(rdir, HIGH);
              }
              else
              {
                digitalWrite(rdir, LOW);
              }
              
  
              pwml = (readchar() - '0')*100 + (readchar()-'0')*10 + (readchar()-'0');
              mpwml = map(pwml, 0, 255, 0, 65535);
              Last_pwml = mpwml;
              ledcWrite(LeftPWMChannel, mpwml);
      
              dl = readchar()-'0';
                         
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
  
              break;
            }
            
            case 'm':
            {
              int ledBlue = 1;
              xQueueSend(queue, &ledBlue, 0);

              gear_mm = readchar();

              if(readchar() == 's')
              {
                x_mm = (readchar() - '0')*10000 + (readchar()-'0')*1000 + (readchar()-'0')*100 + (readchar()-'0')*10 + (readchar()-'0');
              }
      
              if(readchar() == 'f')
              {
                y_mm = (readchar() - '0')*10000 + (readchar()-'0')*1000 + (readchar()-'0')*100 + (readchar()-'0')*10 + (readchar()-'0');
              }
      
              trash = readchar();
      
              x_mm = x_mm - 8000;
              y_mm = y_mm - 8000;
      
             if (abs(x_mm) < 500)
              x_mm = 0;
             if (abs(y_mm) < 500)
              y_mm = 0;
      
             MotorCode(x_mm, y_mm, gear_mm);   //Run MotorCode 
              
             
             break;
           }
           
          case 'e':
          {
            int greenflash = 2;
            xQueueSend(queue,&greenflash, 0);
          }
          
          default:
          int def = 3;
          xQueueSend(queue,&def, 0);
          printf("Entered Default");
        }
          
        }
      }

    client.stop();

    int ledstop = 3;
    xQueueSend(queue,&ledstop, 0);

    digitalWrite(rdir, LOW);
    digitalWrite(ldir, LOW);
    ledcWrite(LeftPWMChannel, 0);
    ledcWrite(RightPWMChannel, 0);
    
    Serial.println("Client disconnected");
    }
  }
} 

void LedPanelCode( void * pvParameters )
{
  int Flag;
  
  for(;;)
  {
    
    xQueueReceive(queue, &Flag, portMAX_DELAY);
    
        
    if(Flag == 0)
    {
      colorWipe(matrix.Color(255, 0, 0), 0);
    } 
    else if(Flag == 1)
    {
      colorWipe(matrix.Color(0, 0, 255), 0);
    }
    else if(Flag == 2)
    {
      blink_green_x10();
    }
    else if(Flag == 3)
    {
      colorWipe(matrix.Color(0, 0, 0), 0);
      //matrix.begin();
    } 
  }
}


void loop() 
{

   
}
