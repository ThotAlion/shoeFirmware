#include "HX711.h"
#include <MPU6050_tockn.h>
#include <Wire.h>
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
         | [ ]GND                  TX[ ] |   
         | [ ]Vin                  RX[ ] |   
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

// declaration des capteurs de force
HX711 foot_rf;
HX711 foot_rb;
HX711 foot_lf;
HX711 foot_lb;

// declaration du MPU
MPU6050 mpu6050(Wire);

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

void setup() {
  pixels.begin();
  pixels.clear();
  pixels.setBrightness(50);
  pixels.setPixelColor(5, pixels.Color(255, 0, 0));
  pixels.show();
  pinMode(ILS_PIN, INPUT);
  Serial.begin(115200);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  foot_rf.begin(FOOT_RF_DOUT_PIN, FOOT_RF_SCK_PIN);
  foot_rb.begin(FOOT_RB_DOUT_PIN, FOOT_RB_SCK_PIN);
  foot_lf.begin(FOOT_LF_DOUT_PIN, FOOT_LF_SCK_PIN);
  foot_lb.begin(FOOT_LB_DOUT_PIN, FOOT_LB_SCK_PIN);
  pixels.setPixelColor(5, pixels.Color(0, 0, 150));
  pixels.show();
}

void loop() {
  // lecture des capteurs
  mpu6050.update();
  if (foot_rf.is_ready()){
    force_rf = -(foot_rf.read()-89287)/1000;
    ready_rf = 1;
  }else{
    ready_rf = 0;
  }
  if (foot_rb.is_ready()){
    force_rb = -(foot_rb.read()+41736)/1000;
    ready_rb = 1;
  }else{
    ready_rb = 0;
  }
  if (foot_lf.is_ready()){
    force_lf = -(foot_lf.read()-124089)/1000;
    ready_lf = 1;
  }else{
    ready_lf = 0;
  }
  if (foot_lb.is_ready()){
    force_lb = -(foot_lb.read()-17528)/1000;
    ready_lb = 1;
  }else{
    ready_lb = 0;
  }

  // led
  pixels.clear();
  if(force_rf+force_rb+force_lf+force_lb>40){
    pixels.setPixelColor(10*(force_rf+force_rb)/(force_rf+force_rb+force_lf+force_lb), pixels.Color(0, 0, 150));
  }
  pixels.show();
  
  if(Serial.available()>0){
    char c = (char)Serial.read();
    //Serial.println(c);
    if(c=='A'){
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
      Serial.print("},\"GYR\":{\"X\":");
      Serial.print(mpu6050.getGyroX());
      Serial.print(",\"Y\":");
      Serial.print(mpu6050.getGyroY());
      Serial.print(",\"Z\":");
      Serial.print(mpu6050.getGyroZ());
      Serial.print("},\"ANG\":{\"X\":");
      Serial.print(mpu6050.getAngleX());
      Serial.print(",\"Y\":");
      Serial.print(mpu6050.getAngleY());
      Serial.print(",\"Z\":");
      Serial.print(mpu6050.getAngleZ());
      Serial.print("},\"ILS\":{\"S\":");
      Serial.print(!digitalRead(ILS_PIN));
      Serial.print("}}\n");
    }
  }
  delay(10);
}
