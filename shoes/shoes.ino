#include "HX711.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_NeoPixel.h>
#define NUM_LEDS 11
#define NEOPIXEL_PIN 5

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
     MPU | [ ]A4/D18/SDA           D5[ ]~| NEOPIXEL
     MPU | [ ]A5/D19/SCL           D4[ ] |   
         | [ ]A6                   D3[ ]~|   
         | [ ]A7                   D2[ ] | ILS  
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
const int ILS_PIN = 2;

//Temps d'echantillonnage (ms)
const int dt = 20;

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

// declaration du MPU
MPU6050 mpu(Wire);

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
  pixels.setPixelColor(5, pixels.Color(255, 255, 255));
  pixels.show();
  delay(100);
  pinMode(ILS_PIN, INPUT);
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("Init.");
  Wire.begin();
  mpu.begin();
  Serial.println("Start calibration.");
  pixels.setPixelColor(5, pixels.Color(255, 0, 0));
  pixels.show();
  delay(1000);
  mpu.calcGyroOffsets();
  Serial.println("Done !");
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
  mpu.update();
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
      Serial.print("},\"ANG\":{\"X\":");
      Serial.print(mpu.getAngleX());
      Serial.print(",\"Y\":");
      Serial.print(mpu.getAngleY());
      Serial.print(",\"Z\":");
      Serial.print(mpu.getAngleZ());
      Serial.print("},\"ILS\":{\"S\":");
      Serial.print(!digitalRead(ILS_PIN));
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
      Serial1.print("},\"ANG\":{\"X\":");
      Serial1.print(mpu.getAngleX());
      Serial1.print(",\"Y\":");
      Serial1.print(mpu.getAngleY());
      Serial1.print(",\"Z\":");
      Serial1.print(mpu.getAngleZ());
      Serial1.print("},\"ILS\":{\"S\":");
      Serial1.print(!digitalRead(ILS_PIN));
      Serial1.print("}}\n");
    }
  }
  if(test == 1){
    pixels.setPixelColor(0, pixels.Color(150, 0, 0));
  }
  pixels.show();
  delay(dt);
}
