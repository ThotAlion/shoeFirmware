#include "HX711.h"
#include <Adafruit_NeoPixel.h>
#define NUM_LEDS 11
#define NEOPIXEL_PIN 5
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
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
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  
  pixels.setPixelColor(3, pixels.Color(255, 255, 255));
  pixels.show();
  
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  
  pixels.setPixelColor(4, pixels.Color(255, 255, 255));
  pixels.show();
  
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  
  pixels.setPixelColor(5, pixels.Color(255, 255, 255));
  pixels.show();

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }
  
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
  //lecture du DMP
  // if programming failed, don't try to do anything
  if (dmpReady) {
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      if (mpuInterrupt && fifoCount < packetSize) {
        // try to get out of the infinite loop 
        fifoCount = mpu.getFIFOCount();
      }  
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    if(fifoCount < packetSize){
    }
    // check for overflow (this should never happen unless our code is too inefficient)
    else if ((mpuIntStatus & (0x01 << MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      //  fifoCount = mpu.getFIFOCount();  // will be zero after reset no need to ask
      Serial.println(F("FIFO overflow!"));
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & (0x01 << MPU6050_INTERRUPT_DMP_INT_BIT)) {
      // read a packet from FIFO
      while(fifoCount >= packetSize){ // Lets catch up to NOW, someone is using the dreaded delay()!
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
      }
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//      Serial.print("ypr\t");
//      Serial.print(ypr[0] * 180/M_PI);
//      Serial.print("\t");
//      Serial.print(ypr[1] * 180/M_PI);
//      Serial.print("\t");
//      Serial.println(ypr[2] * 180/M_PI);
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
    }
  }

  
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
      Serial.print(ypr[2] * 180/M_PI);
      Serial.print(",\"Y\":");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print(",\"Z\":");
      Serial.print(ypr[0] * 180/M_PI);
//      Serial.print("},\"ILS\":{\"S\":");
//      Serial.print(!digitalRead(ILS_PIN));
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
      Serial1.print(ypr[2] * 180/M_PI);
      Serial1.print(",\"Y\":");
      Serial1.print(ypr[1] * 180/M_PI);
      Serial1.print(",\"Z\":");
      Serial1.print(ypr[0] * 180/M_PI);
//      Serial1.print("},\"ILS\":{\"S\":");
//      Serial1.print(!digitalRead(ILS_PIN));
      Serial1.print("}}\n");
    }
  }
  if(test == 1){
    pixels.setPixelColor(0, pixels.Color(150, 0, 0));
  }
  pixels.show();
  //delay(dt);
}
