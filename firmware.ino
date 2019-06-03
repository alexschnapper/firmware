#define USE_GPS     1
#define USE_LORA    1
#define USE_SDS011  1

#include <TimerTCC0.h>
int sec = 0;
int secSinceStart = 0;


#if USE_SDS011
#include "SDS011.h"
SDS011 sdsSensor;
// https://www.arduino.cc/en/Reference/SoftwareSerialConstructor
// rxPin: the pin on which to receive serial data 
// txPin: the pin on which to transmit serial data 
#define SDS_PIN_RX 5  // RX = red cable
#define SDS_PIN_TX 6  // TX = orange cable
// later in the code it sends:
// ssSDS = new SoftwareSerial(SDS_PIN_RX, SDS_PIN_TX); 
SoftwareSerial *ssSDS; 
boolean readSDS;
float pm25;
float pm10;
int sdsErrorCode;
#endif

#if USE_GPS
#include "TinyGPS++.h"
TinyGPSPlus gps;
boolean readGPS = false;
#endif

double lati;
double longi;

#if USE_LORA
#include <LoRaWan.h>


// The Things Network configuration -- Start -------
#define DEV_ADDR  "26011163"
#define DEV_EUI   "00144B18B55FA651"
#define APP_EUI   "70B3D57ED00189D3"
#define NWK_S_KEY "A2E44769CB28AD0418C05955C4A0D7FA"
#define APP_S_KEY "62E062150BD78CFDD6D5648C314EC13B"
#define APP_KEY   "9302B0831DB58F8A6DB0A9E7A1183CCD"

const float EU_channels[8] =    {868.1, 868.3, 868.5, 867.1, 867.3, 867.5, 867.7, 867.9}; //rx 869.525
#define UPLINK_DATA_RATE_MIN    DR0 //The min uplink data rate for all countries / plans is DR0
#define UPLINK_DATA_RATE_MAX_EU DR5
#define MAX_EIRP_NDX_EU         5
#define DOWNLINK_DATA_RATE_EU   DR3
#define FREQ_RX_WNDW_SCND_EU    869.525
#define DEFAULT_RESPONSE_TIMEOUT 5
// The Things Network configuration -- End -------

unsigned int frame_counter = 1;
char buffer[256];
boolean sendPacket = false;
#endif

const int ledFalse = 13;  // rot, definiere Variable für Pin 13
const int ledSDS = 12;     // gelb, definiere Variable für Pin 12
const int ledLORA = 11;     // gruen, definiere Variable für Pin 11
const int ledGPS = 10;      // blau, definiere Variable für Pin 10
const int ledWhite = 9;     // white LED

int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

int ledState = LOW;   // ledState used to set the LED
unsigned long previousMillis = 0;   // will store last time LED was updated
const long interval = 250; // interval at which to blink (ms)

/**
 * blinkwithoutDelayLED
 * bwodLED
 */
void bwodLED(const int led) {
  // check to see if it's time to blink the LED, that is, if the difference
  // between the current time and last time you blinked the LED is bigger than
  // the interval at which you want to blink the LED
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    // if the LED is off turn it on and vice-versa
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // set the LED with the ledSTate of the variable.
    digitalWrite(led, ledState);
  }
}

void checkwire() {
  // if no gps detected, check wiring, show red and blue leds
  digitalWrite(ledFalse, HIGH);
  digitalWrite(ledGPS, HIGH);
}

void nogpsfixintime() {
  // Not able to get a fix in alloted time
  digitalWrite(ledWhite, HIGH);
  digitalWrite(ledGPS, HIGH);
}

// +++++++++++++++++++++++++++++++++ S E T U P +++++++++++++++++++++++++++++++++
void setup() {
  SerialUSB.begin(115200);
  //while (!SerialUSB);
  
  pinMode(ledFalse, OUTPUT);
  pinMode(ledSDS, OUTPUT); 
  pinMode(ledLORA, OUTPUT);
  pinMode(ledGPS, OUTPUT);
  pinMode(ledWhite, OUTPUT); 

  // initialize digital pin LED_BUILTIN as an output.
  // pinMode(LED_BUILTIN, OUTPUT);

  #if USE_SDS011
    ssSDS = new SoftwareSerial(SDS_PIN_RX, SDS_PIN_TX); 
    sdsSensor.begin (ssSDS);
  #endif

  #if USE_GPS
    char c;
    bool locked;
    Serial.begin(9600);     // open the GPS
    locked = false;
  
    // For S&G, let's get the GPS fix now, before we start running arbitary
    // delays for the LoRa section
  
    while (!gps.location.isValid()) {
      while (Serial.available() > 0) {
        if (gps.encode(c = Serial.read())) {
          displayGPSInfo();
          if (gps.location.isValid()) {
            //            locked = true;
            break;
          }
        }
        //        SerialUSB.print(c);
      }
      //      if (locked)
      //        break;
      if (millis() > 15000 && gps.charsProcessed() < 10){
        SerialUSB.println(F("No GPS detected: check wiring."));
        SerialUSB.println(gps.charsProcessed());
        checkwire(); // on
        while (true);
      }  else if (millis() > 999000) {
        SerialUSB.println(F("Not able to get a fix in alloted time."));
        break;
      }
    } // while (!gps.location.isValid()) - Schleife
  #endif

  #if USE_LORA
    
    digitalWrite(ledWhite, HIGH);
    
    lora.init();
    lora.setDeviceDefault();
  
    memset(buffer, 0, 256);
    lora.getVersion(buffer, 256, 1);
    SerialUSB.print(buffer);
    SerialUSB.println(("buffer: ", buffer));
  
    memset(buffer, 0, 256);
    lora.getId(buffer, 256, 1);
    SerialUSB.print(buffer);
  
    // void setId(char *DevAddr, char *DevEUI, char *AppEUI);
    lora.setId(DEV_ADDR, DEV_EUI, APP_EUI);
    // setKey(char *NwkSKey, char *AppSKey, char *AppKey);
    lora.setKey(NWK_S_KEY, APP_S_KEY, APP_KEY);
  
    lora.setDeciveMode(LWABP);
    lora.setDataRate(DR0, EU868);   // 0 = SF12 / 125 kHz / 440bps
    // 1 = SF11 / 125 kHz / 440bps
    // 2 = Max EIRP –  4dB
    // 5 = Max EIRP - 10dB - MAX_EIRP_NDX_EU_433
    // 7 = Max EIRP - 14dB - MAX_EIRP_NDX_EU_863
  
    // power is signal strength as well, might override above data rate setting
    lora.setPower(15); // works

    setChannelsForTTN(EU_channels);
    lora.setReceiceWindowFirst(7, 867.9);
    lora.setReceiceWindowSecond(FREQ_RX_WNDW_SCND_EU, DOWNLINK_DATA_RATE_EU);
    
    digitalWrite(ledWhite, LOW);
    
    //    lora.setDutyCycle(false);
    //    lora.setJoinDutyCycle(false);
  #endif // USE_LORA

  //TimerTcc0.initialize(60000000); 1 Minute
  TimerTcc0.initialize(1000000); //1 second
  TimerTcc0.attachInterrupt(timerIsr);
} 
// +++++++++++++++++++++++++++++++++ S E T U P +++++++++++++++++++++++++++++++++

// void(* resetFunc) (void) = 0; //declare reset function @ address 0

//interrupt routine
void timerIsr(void) {
  secSinceStart += 1;

  #if USE_LORA
    // every minute
    if (secSinceStart % 60 == 0) {
      SerialUSB.println("LORA sendPacket=true");
      sendPacket = true;
      // digitalWrite(ledLORA, HIGH);
      bwodLED(ledLORA);

    } else {
      sendPacket = false;
      digitalWrite(ledLORA, LOW);

    }
  #endif

  // sec is forced to a range of 0-5, so we can decide to do something in either of those seconds
  sec = (sec + 1) % 6;
  SerialUSB.println(sec);
  bwodLED(ledWhite); // blink white LED every second = program running

  #if USE_GPS
    if (sec == 1) {
      Serial.write("h"); //Turn on GPS
    }
  #endif
  #if USE_SDS011
    if (sec == 2) {
      readSDS = true;
    }
  #endif
  #if USE_GPS
    if (sec == 3) {
      readGPS = true;
    }
  #endif
  if (sec == 5) {
    #if USE_SDS011
        readSDS = false;

    #endif
    #if USE_GPS
        displayGPSInfo();
        readGPS = false;
        Serial.write("$PMTK161,0*28\r\n");
    #endif
  }
}  // void timerIsr(void) Schleife ---


void loop(void) {

//  Serial.println("resetting");
//  resetFunc();  //call reset

  #if USE_SDS011
    if (readSDS) {
      // bwodLED(ledSDS);   // flashes really fast
      digitalWrite(ledSDS, HIGH);    // works
      delay(500);
      // return code is 0, if new values were read, and 1 if there were no new values.
      sdsErrorCode = sdsSensor.read(&pm25, &pm10);
      // SerialUSB.println("PM 2.5: " + String(pm25) + ", PM 10: " + String(pm10)); // works
      
      if (!sdsErrorCode) {
        
        SerialUSB.println("PM 2.5: " + String(pm25));
        SerialUSB.println("PM 10:  " + String(pm10));

      } else {
        SerialUSB.print(F("SDS Error Code: "));
        SerialUSB.println(sdsErrorCode);
        
      }
      readSDS = false;
      
    } else {
      // if readSDS = false, means not reading sds data
      digitalWrite(ledSDS, LOW); // works
    }
  #endif
  
  #if USE_GPS
    if (readGPS) {
      digitalWrite(ledGPS, HIGH);
      while (Serial.available() > 0) {
        char currChar = Serial.read();
        gps.encode(currChar);
      }
    } else {
      // readGPS = false, when gps data should not be read
      digitalWrite(ledGPS, LOW);
    }
  #endif
  
  #if USE_LORA
    if (sendPacket) {
      digitalWrite(ledLORA, HIGH);
      SerialUSB.print("sendAndReceiveLoRaPacket() frame_counter: ");
      SerialUSB.println(frame_counter);
      sendAndReceiveLoRaPacket();
      frame_counter += 1;

    } else {
      // sendPacket = false, when no packet should be sent
      digitalWrite(ledLORA, LOW);
    }
  #endif
} // loop(void) Schleife 

#if USE_GPS
  // ''' GPS Info Start ''' 
  void displayGPSInfo() {
    SerialUSB.print(F("Location (Latitude, Longtitude): "));
    if (gps.location.isValid()) {
      SerialUSB.print(gps.location.lat(), 6);
      SerialUSB.print(F(","));
      SerialUSB.print(gps.location.lng(), 6);  
       lati = (gps.location.lat());
       longi = (gps.location.lng());
    
      digitalWrite(ledGPS, HIGH);
      digitalWrite(ledFalse, LOW);   
    } else  {
      SerialUSB.print(F("INVALID"));
      digitalWrite(ledFalse, HIGH);
      digitalWrite(ledGPS, LOW);
    }
  
    SerialUSB.print(F("  Date/Time: "));
    if (gps.date.isValid())  {
      SerialUSB.print(gps.date.day());
      SerialUSB.print(F("."));
      SerialUSB.print(gps.date.month());
      SerialUSB.print(F("."));
      SerialUSB.print(gps.date.year());

    } else {
      SerialUSB.print(F("INVALID"));
    }
  
    SerialUSB.print(F(" "));
    if (gps.time.isValid()) {
      if (gps.time.hour() < 10) SerialUSB.print(F("0"));
      SerialUSB.print(gps.time.hour());
      SerialUSB.print(F(":"));
      if (gps.time.minute() < 10) SerialUSB.print(F("0"));
      SerialUSB.print(gps.time.minute());
      SerialUSB.print(F(":"));
      if (gps.time.second() < 10) SerialUSB.print(F("0"));
      SerialUSB.print(gps.time.second());
      SerialUSB.print(F("."));
      if (gps.time.centisecond() < 10) SerialUSB.print(F("0"));
      SerialUSB.print(gps.time.centisecond());
    }  else  {
      SerialUSB.print(F("INVALID"));
    }
  
    SerialUSB.println();
}
#endif
// ''' GPS Info End ''' 

#if USE_LORA
  void setChannelsForTTN(const float* channels) {
    for (int i = 0; i < 8; i++) {
      // DR0 is the min data rate
      // UPLINK_DATA_RATE_MAX_EU = DR5 is the max data rate for the EU
      if (channels[i] != 0) {
        lora.setChannel(i, channels[i], UPLINK_DATA_RATE_MIN, UPLINK_DATA_RATE_MAX_EU);
      }
    }
  }

  void sendAndReceiveLoRaPacket() {
    bool result = false;
    char payload[256];
    memset(payload, 0, 256);
    // String(frame_counter).toCharArray(payload, 256);  // original
    // String("PM2.5: " + String(pm25) + ", PM10: " + String(pm10)).toCharArray(payload,256); // generates: PM2.5: 30.50, PM10: 52.70
    
    // WORKS:
    // Output: 50.12,8.69;11.30,13.90
    /* double lati = (gps.location.lat());
    double longi = (gps.location.lng());
    */
    
    // WORKS:
    // Output: 50.12,8.69;11.30,13.90
    //
    // 0.000000,0.000000;14.500000,19.299999
    // when ,6 is used... Needed for lati and long = better GPS Position. 
  
    // TODO what when no GPS or no SDS data?? 
    String(String(lati,6) + "," + String(longi,6) + ";" + String(pm25) + "," + String(pm10)).toCharArray(payload,256); // works

    // LoRa radio data structure
    // 1. UNIX TIMESTAMP
    // 2. PM2.5
    // 3. PM10
    // 4. lat
    // 5. lon

    //String(String(lati,6) + "," + String(longi,6) + ";" + String(pm25) + "," + String(pm10)).toCharArray(payload,256); // works
    // String(String(pm25) + "," String(pm10) + "," + String(lati,6) + "," + String(longi,6).toCharArray(payload,256)); // test

    // TEST
    // String(String("1520105731") + "," + String(lati,6) + "," + String(longi,6) + ";" + String(pm25) + "," + String(pm10)).toCharArray(payload,256); // works
    
    // send data
     result = lora.transferPacket(payload, 10); // original, works
     String theResult = "false";
     if (result) {
        theResult = "true";
      } else {
        theResult = "false";
      }
     // SerialUSB.println("theResult is: " + theResult);
     
    if (result) {
      short length;
      short rssi;
  
      memset(buffer, 0, 256);
      length = lora.receivePacket(buffer, 256, &rssi);
  
      if (length) {
        SerialUSB.print("Length is: ");
        SerialUSB.println(length);
        SerialUSB.print("RSSI is: ");
        SerialUSB.println(rssi);
        SerialUSB.print("Data is: ");
        for (unsigned char i = 0; i < length; i ++) {
          SerialUSB.print("0x");
          SerialUSB.print(buffer[i], HEX);
          SerialUSB.print(" ");
        }
        SerialUSB.println();
      }
    } else {
      // result is false
      SerialUSB.print("Result is false: " + result);
     }
  }
#endif