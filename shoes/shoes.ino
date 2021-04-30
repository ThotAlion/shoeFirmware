#include "HX711.h"
#include <Adafruit_NeoPixel.h>
#define NUM_LEDS 11
#define NEOPIXEL_PIN 5
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


/*
Description :
Ce firmware a pour rôle de récuperer les valeurs de l'ensemble des capteurs sur demande
via port série quand la lettre "A" est demandée. 
La réponse est envoyée en format json.
Voici le schema électrique de la carte :
                      +-----+
         +------------| USB |------------+
         |            +-----+            |
         | [ ]D13            MISO/D12[ ] |    
         | [ ]3.3V           MOSI/D11[ ]~|   
         | [ ]V.ref     ___    SS/D10[ ]~|   
 RB_DOUT | [ ]A0/D14   / N \       D9[ ]~| LF_DOUT
  RB_SCK | [ ]A1/D15  /  A  \      D8[ ] | LF_SCK
 RF_DOUT | [ ]A2/D16  \  N  /      D7[ ] | LB_DOUT
  RF_SCK | [ ]A3/D17   \_0_/       D6[ ]~| LB_SCK
     BNO | [ ]A4/D18/SDA           D5[ ]~| NEOPIXEL
     BNO | [ ]A5/D19/SCL           D4[ ] |   
         | [ ]A6                   D3[ ]~|   
         | [ ]A7                   D2[ ] |  interrupt
         | [ ]5V                  GND[ ] |     
         | [ ]RST                 RST[ ] |   
         | [ ]GND                 RX1[ ] |   
         | [ ]Vin                 TX1[ ] |   
         |                               |
         |                               |
         | NANO EVERY                    |
         +-------------------------------+
*/         

//Description des I/O et declarations
//Lecteurs de force sous les pieds
// capteur droit avant (jaune)
const int FOOT_RF_DOUT_PIN = 16;
const int FOOT_RF_SCK_PIN = 17;
// capteur droit arrière (noir)
const int FOOT_RB_DOUT_PIN = 14;
const int FOOT_RB_SCK_PIN = 15;
// capteur gauche avant (rouge)
const int FOOT_LF_DOUT_PIN = 9;
const int FOOT_LF_SCK_PIN = 8;
// capteur gauche arrière (bleu)
const int FOOT_LB_DOUT_PIN = 7;
const int FOOT_LB_SCK_PIN = 6;

//Bouton ILS pour démarrage
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// BNO055
Adafruit_BNO055 bnoR = Adafruit_BNO055(55, 0x29);
Adafruit_BNO055 bnoL = Adafruit_BNO055(56, 0x28);

//Temps d'echantillonnage (ms)
const int dt = 10;

//offset des capteurs de pression
const long force_rf_offset = 89287;
const long force_rb_offset = 41736;
const long force_lf_offset = 124089;
const long force_lb_offset = 17528;

// declaration des capteurs de force
HX711 foot_rf;
HX711 foot_rb;
HX711 foot_lf;
HX711 foot_lb;

//neopixel
Adafruit_NeoPixel pixels(NUM_LEDS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// declaration des variables
long force_rf=0;
long force_rb=0;
long force_lf=0;
long force_lb=0;
int  ready_rf=0;
int  ready_rb=0;
int  ready_lf=0;
int  ready_lb=0;
int test = 0;
char c,c0;

void setup() {
  pixels.begin();
  pixels.clear();
  pixels.setBrightness(50);
  pixels.setPixelColor(0, pixels.Color(255, 255, 255));
  pixels.show();
  
  delay(100);
  //pinMode(ILS_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  pixels.setPixelColor(1, pixels.Color(255, 255, 255));
  pixels.show();
  
  Serial.begin(115200);
  Serial1.begin(115200);

  pixels.setPixelColor(2, pixels.Color(255, 255, 255));
  pixels.show();
  /* Initialise the sensor */
  if(!bnoR.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 right detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!bnoL.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 left detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
  bnoR.setExtCrystalUse(true);
  bnoL.setExtCrystalUse(true);
  pixels.setPixelColor(3, pixels.Color(255, 255, 255));
  pixels.show();
  
  pixels.setPixelColor(4, pixels.Color(255, 255, 255));
  pixels.show();
  
  pixels.setPixelColor(5, pixels.Color(255, 255, 255));
  pixels.show();

  pixels.setPixelColor(5, pixels.Color(0, 0, 255));
  pixels.show();
  
  foot_rf.begin(FOOT_RF_DOUT_PIN, FOOT_RF_SCK_PIN);
  foot_rb.begin(FOOT_RB_DOUT_PIN, FOOT_RB_SCK_PIN);
  foot_lf.begin(FOOT_LF_DOUT_PIN, FOOT_LF_SCK_PIN);
  foot_lb.begin(FOOT_LB_DOUT_PIN, FOOT_LB_SCK_PIN);
  pixels.setPixelColor(5, pixels.Color(255, 255, 0));
  pixels.show();
}

void loop() {
  // lecture du BNO
  sensors_event_t orientationDataR,angVelocityDataR;
  bnoR.getEvent(&orientationDataR, Adafruit_BNO055::VECTOR_EULER);
  bnoR.getEvent(&angVelocityDataR, Adafruit_BNO055::VECTOR_GYROSCOPE);
  sensors_event_t orientationDataL,angVelocityDataL;
  bnoL.getEvent(&orientationDataL, Adafruit_BNO055::VECTOR_EULER);
  bnoL.getEvent(&angVelocityDataL, Adafruit_BNO055::VECTOR_GYROSCOPE);
  // lecture des capteurs
  if (foot_rf.is_ready()){
    force_rf = -(foot_rf.read()-force_rf_offset)/1000;
    ready_rf = 1;
  }else{
    ready_rf = 0;
  }
  if (foot_rb.is_ready()){
    force_rb = -(foot_rb.read()+force_rb_offset)/1000;
    ready_rb = 1;
  }else{
    ready_rb = 0;
  }
  if (foot_lf.is_ready()){
    force_lf = -(foot_lf.read()-force_lf_offset)/1000;
    ready_lf = 1;
  }else{
    ready_lf = 0;
  }
  if (foot_lb.is_ready()){
    force_lb = -(foot_lb.read()-force_lb_offset)/1000;
    ready_lb = 1;
  }else{
    ready_lb = 0;
  }

  // led
  pixels.clear();
  if(force_rf+force_rb+force_lf+force_lb>40){
    pixels.setPixelColor(10*(force_rf+force_rb)/(force_rf+force_rb+force_lf+force_lb), pixels.Color(0, 0, 150));
  }
  if(Serial.available()>0){
    c0 = (char)Serial.read();
    if(c0=='B'){
      if(test == 0){
        test = 1;
      }else{
        test = 0;
      }
    }
    if(c0=='A'){
      // envoyer le json
      Serial.print("{\"RF\":{\"F\":");
      Serial.print(force_rf);
      Serial.print(",\"S\":");
      Serial.print(ready_rf);
      Serial.print("},\"RB\":{\"F\":");
      Serial.print(force_rb);
      Serial.print(",\"S\":");
      Serial.print(ready_rb);
      Serial.print("},\"LF\":{\"F\":");
      Serial.print(force_lf);
      Serial.print(",\"S\":");
      Serial.print(ready_lf);
      Serial.print("},\"LB\":{\"F\":");
      Serial.print(force_lb);
      Serial.print(",\"S\":");
      Serial.print(ready_lb);
      Serial.print("},\"ANGR\":{\"X\":");
      Serial.print((float)orientationDataR.orientation.z);
      Serial.print(",\"Y\":");
      Serial.print((float)orientationDataR.orientation.y);
      Serial.print(",\"Z\":");
      Serial.print((float)orientationDataR.orientation.x);
      Serial.print("},\"GYRR\":{\"X\":");
      Serial.print((float)angVelocityDataR.gyro.x);
      Serial.print(",\"Y\":");
      Serial.print((float)angVelocityDataR.gyro.y);
      Serial.print(",\"Z\":");
      Serial.print((float)angVelocityDataR.gyro.z);
      Serial.print("},\"ANGL\":{\"X\":");
      Serial.print((float)orientationDataL.orientation.z);
      Serial.print(",\"Y\":");
      Serial.print((float)orientationDataL.orientation.y);
      Serial.print(",\"Z\":");
      Serial.print((float)orientationDataL.orientation.x);
      Serial.print("},\"GYRL\":{\"X\":");
      Serial.print((float)angVelocityDataL.gyro.x);
      Serial.print(",\"Y\":");
      Serial.print((float)angVelocityDataL.gyro.y);
      Serial.print(",\"Z\":");
      Serial.print((float)angVelocityDataL.gyro.z);
      Serial.print("}}\n");
    }
  }
  
  if(Serial1.available()>0){
    c = (char)Serial1.read();
    if(c=='A'){
      // envoyer le json
      Serial1.print("{\"RF\":{\"F\":");
      Serial1.print(force_rf);
      Serial1.print(",\"S\":");
      Serial1.print(ready_rf);
      Serial1.print("},\"RB\":{\"F\":");
      Serial1.print(force_rb);
      Serial1.print(",\"S\":");
      Serial1.print(ready_rb);
      Serial1.print("},\"LF\":{\"F\":");
      Serial1.print(force_lf);
      Serial1.print(",\"S\":");
      Serial1.print(ready_lf);
      Serial1.print("},\"LB\":{\"F\":");
      Serial1.print(force_lb);
      Serial1.print(",\"S\":");
      Serial1.print(ready_lb);
      Serial1.print("},\"ANGR\":{\"X\":");
      Serial1.print((float)orientationDataR.orientation.z);
      Serial1.print(",\"Y\":");
      Serial1.print((float)orientationDataR.orientation.y);
      Serial1.print(",\"Z\":");
      Serial1.print((float)orientationDataR.orientation.x);
      Serial1.print("},\"GYRR\":{\"X\":");
      Serial1.print((float)angVelocityDataR.gyro.x);
      Serial1.print(",\"Y\":");
      Serial1.print((float)angVelocityDataR.gyro.y);
      Serial1.print(",\"Z\":");
      Serial1.print((float)angVelocityDataR.gyro.z);
      Serial1.print("},\"ANGL\":{\"X\":");
      Serial1.print((float)orientationDataL.orientation.z);
      Serial1.print(",\"Y\":");
      Serial1.print((float)orientationDataL.orientation.y);
      Serial1.print(",\"Z\":");
      Serial1.print((float)orientationDataL.orientation.x);
      Serial1.print("},\"GYRL\":{\"X\":");
      Serial1.print((float)angVelocityDataL.gyro.x);
      Serial1.print(",\"Y\":");
      Serial1.print((float)angVelocityDataL.gyro.y);
      Serial1.print(",\"Z\":");
      Serial1.print((float)angVelocityDataL.gyro.z);
      Serial1.print("}}\n");
    }
  }
  if(test == 1){
    pixels.setPixelColor(0, pixels.Color(150, 0, 0));
  }
  pixels.show();
  //delay(dt);
}
