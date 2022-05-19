#include "WiFi.h"

// Internet Parameters
const char* ssid = "mrm@M2";
const char* password = "";

// Time to implelement Fail Safe
unsigned long previousMillis = 0;
unsigned long interval = 15;

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

void setup() 
{

  Serial.begin(115200);

  delay(1000);  
  
  // Configures static IP Address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) 
  {
    Serial.println("STA Failed to configure");
  }

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

  WiFiClient client = wifiServer.available();

  if (client) 
  {

    while (client.connected()) 
    {

      while (client.available()>0) 
      {
        String request = client.readStringUntil('e');
        
         if (request[i] == 'm')
         {
            pwmr = (request[i+1]-'0')*100 + (request[i+2]-'0')*10 + (request[i+3]-'0');
            mpwmr = map(pwmr, 0, 255, 0, 65535);
            Last_pwmr = mpwmr;
            /*
            Serial.print("mpwmr = ");
            Serial.println(mpwmr);
            */
            ledcWrite(RightPWMChannel, mpwmr);
            
            dr = request[i+4]-'0';
            
            if(dr == 1)
            {
              digitalWrite(rdir, HIGH);
            }
            else
            {
              digitalWrite(rdir, LOW);
            }
            

            pwml = (request[i+5]-'0')*100 + (request[i+6]-'0')*10 + (request[i+7]-'0');
            mpwml = map(pwml, 0, 255, 0, 65535);
            Last_pwml = mpwml;
            ledcWrite(LeftPWMChannel, mpwml);
    
            dl = request[i+8]-'0';
                       
            if(dl == 1)
            {
              digitalWrite(ldir, HIGH);
            }
            else
            {
              digitalWrite(ldir, LOW);
            }

            /*
            
            Serial.println("PWMR");
            Serial.println(pwmr);
            Serial.println("PWML");
            Serial.println(pwml);
            Serial.println("DIRR");
            Serial.println(dr);
            Serial.println("DIRL");
            Serial.println(dl);

            */
    
            i = 0;
       
       }
        
     }

     // delay(10);
      
    }

    client.stop();


    digitalWrite(rdir, LOW);
    digitalWrite(ldir, LOW);
    ledcWrite(LeftPWMChannel, 0);
    ledcWrite(RightPWMChannel, 0);
    
    Serial.println("Client disconnected");
    

  }
}
