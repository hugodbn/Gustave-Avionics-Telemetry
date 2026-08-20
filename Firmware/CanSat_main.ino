#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <LoRa.h>
#include <DW1000.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM6DSOX.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Servo.h>

Adafruit_BMP3XX bmp; // Pour le BMP
Adafruit_LSM6DSOX sox; // Pour l'IMU
SFE_UBLOX_GNSS myGNSS; // Pour le GPS
Servo releaseServo; // Pour le servo de séparation
File fichierSD; // Fichier gardé ouvert pendant le vol

// LoRa
const int LORA_CS = 1;
const int LORA_RST = 18;
const int LORA_DIO0 = 0;

// Servo
const int SERVO_PIN = 12;

// Carte SD
const int SD_CS = 3;

// DW1000 (UWB)
const int DWM1000_CS = 13;
const int DWM1000_RST = 2;
const int DWM1000_IRQ = 9; 

// IDF Cansat
const byte CANSAT_ID = 1;

// Protocol UWB
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define LEN_DATA 16

byte data[LEN_DATA];

// Machine à états UWB
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
DW1000Time timePollSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
uint32_t lastActivity;
const uint32_t RESET_PERIOD = 250; 

// Variables globales de vol
bool systemeActif = true;
double lat = 0, lon = 0;
float altGPS = 0;
byte SIV = 0;
float pressionSol = 1013.25; // Sera calibrée au setup
String nomFichier;

// Variables de déploiement et filtrage
bool ejectionDetectee = false;
bool servoActive = false;
uint32_t tempsEjection = 0;
const uint32_t DELAI_SEPARATION = 15000; // 15 secondes


// Détection de décollage filtrée (Anti-rebond)
const float SEUIL_ACCELERATION = 15.0; 
const uint32_t DUREE_POUSSEE = 300; 
uint32_t debutPoussee = 0;
bool enPoussee = false;


// Fonction de signalement d'erreur
void signalErreur() {
  digitalWrite(LED_BUILTIN, HIGH);
}

void setup() {
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // LED éteinte = tout va bien

  // Sécurité bus SPI 
  pinMode(LORA_CS, OUTPUT); 
  digitalWrite(LORA_CS, HIGH);

  pinMode(SD_CS, OUTPUT); 
  digitalWrite(SD_CS, HIGH);

  pinMode(DWM1000_CS, OUTPUT); 
  digitalWrite(DWM1000_CS, HIGH);

  // Carte SD
  if (!SD.begin(SD_CS)) signalErreur();

  // BMP390
  if (bmp.begin_I2C(0x76)) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Calibration de la pression au sol moyenne sur 10 lectures
    float sumPression  = 0;
    int   validReadings = 0;
    for (int i = 0; i < 10; i++) {
      if (bmp.performReading()) {
        sumPression += bmp.pressure / 100.0;
        validReadings++;
      }
      delay(50);
    }
    if (validReadings > 0) pressionSol = sumPression / validReadings;

  } else {
    signalErreur();
  }

  // Gyroscope (IMU)
  if (sox.begin_I2C()) {
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);  
    sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  } else {
    signalErreur();
  }

  // DW 1000 
  DW1000.begin(DWM1000_IRQ, DWM1000_RST);
  DW1000.select(DWM1000_CS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(CANSAT_ID + 1);
  DW1000.setNetworkId(10);
  DW1000.setChannel(5);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000.useSmartPower(false); // Désactive le Smart TX Power 
  DW1000.commitConfiguration();
  DW1000.setTXPower(0x85858585);
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);

  // LoRa
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (LoRa.begin(868E6)) {
    LoRa.setSpreadingFactor(7); // débit rapide pour du temps réel
    LoRa.setTxPower(14);        // puissance max légale 868 MHz
  } else {
    signalErreur();
  }

  // GPS
  Serial1.begin(38400);
  if (myGNSS.begin(Serial1)) {
    myGNSS.setUART1Output(COM_TYPE_UBX);
    myGNSS.setNavigationFrequency(5);
    myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g); // Optimisé hautes accélérations
  } else {
    signalErreur();
  }

  // Servo
  releaseServo.attach(SERVO_PIN);
  releaseServo.write(0); // Position fermée initiale
}

void loop() {
  if (!systemeActif) return;

  if (!ejectionDetectee) {
    detecterDecollage();
  } else {
    gestionDeploiement();
    gestionUWB();
    Lcapteur();
  }
}


void detecterDecollage() {
  sensors_event_t accel, gyro, tempIMU;
  sox.getEvent(&accel, &gyro, &tempIMU);

  float accTotale = sqrt(
    accel.acceleration.x * accel.acceleration.x +
    accel.acceleration.y * accel.acceleration.y +
    accel.acceleration.z * accel.acceleration.z
  );

  if (accTotale >= SEUIL_ACCELERATION) {
    if (!enPoussee) {
      // démarre le chrono
      enPoussee    = true;
      debutPoussee = millis();
    } else if (millis() - debutPoussee >= DUREE_POUSSEE) {
      ejectionDetectee = true;
      tempsEjection    = millis();

      // Ouverture des fichiers (gardés ouverts tout le vol)
      fichierSD = SD.open("data.csv", FILE_WRITE);
      if (fichierSD) {
        fichierSD.println("ID;TimeGPS;Temps(ms);EtatEjection;Pression;AltBaro;Temp;"
                          "AccX;AccY;AccZ;GyrX;GyrY;GyrZ;Lat;Lon;AltGPS;SIV");
        fichierSD.flush();
      }

      // Lancement du ranging UWB
      transmitPoll();
      noteActivity();
    }
  } else {
    enPoussee = false;
  }
}

void gestionDeploiement() {
  if (!servoActive && millis() - tempsEjection >= DELAI_SEPARATION) {
    releaseServo.write(90); // Séparation CanSat 
    servoActive = true;
  }
}

void gestionUWB() {
  uint32_t curMillis = millis();

  if (!sentAck && !receivedAck) {
    if (curMillis - lastActivity > RESET_PERIOD) resetInactive();
    return;
  }

  if (sentAck) {
    sentAck = false;
    byte msgId = data[0];

    if (msgId == POLL) {
      DW1000.getTransmitTimestamp(timePollSent);
    }
    else if (msgId == RANGE) {
      DW1000.getTransmitTimestamp(timeRangeSent);
      DW1000.newReceive();
      DW1000.setDefaults();
      DW1000.startReceive();
    }
    noteActivity();
  }

  if (receivedAck) {
    receivedAck = false;
    DW1000.getData(data, LEN_DATA);
    byte msgId = data[0];

    if (msgId == POLL_ACK) {
      DW1000.getReceiveTimestamp(timePollAckReceived);
      transmitRange();
      noteActivity();
    }
  }
}

void transmitPoll() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRange() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE;
  timePollSent.getTimestamp(data + 1); // envoi du POLL
  timePollAckReceived.getTimestamp(data + 6); // réception du POLL_ACK
  timeRangeSent.getTimestamp(data + 11); // envoi du RANGE (approx.)
  data[15] = CANSAT_ID; // Idf cansat
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void noteActivity() { lastActivity = millis(); }
void resetInactive() { transmitPoll(); noteActivity(); }
void handleSent() { sentAck = true; }
void handleReceived(){ receivedAck = true; }

void Lcapteur() {
  static uint32_t lastRead = 0;
  uint32_t curMillis = millis();

  if (curMillis - lastRead <= 100) return; // 10 Hz
  lastRead = curMillis;

  char timeGPSBuf[24] = "00:00:00.000";
  if (myGNSS.getPVT()) {
    lat    = myGNSS.getLatitude()  / 10000000.0;
    lon    = myGNSS.getLongitude() / 10000000.0;
    altGPS = myGNSS.getAltitude()  / 1000.0;
    SIV    = myGNSS.getSIV();
    sprintf(timeGPSBuf, "%02d:%02d:%02d.%03d",
            myGNSS.getHour(), myGNSS.getMinute(),
            myGNSS.getSecond(), myGNSS.getMillisecond());
  }

  float pression = 0, temp = 0, altBaro = 0;
  if (bmp.performReading()) {
    pression = bmp.pressure / 100.0;
    temp     = bmp.temperature;
    altBaro  = bmp.readAltitude(pressionSol); // Altitude relative au pas de tir
  }

  sensors_event_t accel, gyro, tempIMU;
  sox.getEvent(&accel, &gyro, &tempIMU);

  String dataStr = String(CANSAT_ID) + ";" +
                  String(timeGPSBuf) + ";" +
                   String(curMillis)  + ";" +
                   String(ejectionDetectee) + ";" +
                   String(pression) + ";" +
                   String(altBaro) + ";" +
                   String(temp) + ";" +
                   String(accel.acceleration.x)+ ";" +
                   String(accel.acceleration.y) + ";" +
                   String(accel.acceleration.z)+ ";" +
                   String(gyro.gyro.x) + ";" +
                   String(gyro.gyro.y) + ";" +
                   String(gyro.gyro.z) + ";" +
                   String(lat, 6)+ ";" +
                   String(lon, 6)+ ";" +
                   String(altGPS) + ";" +
                   String(SIV);

  // Carte SD
  if (fichierSD) {
    fichierSD.println(dataStr);
    fichierSD.flush();
  }

  //LoRa
  if (dataStr.length() <= 255) {
    LoRa.beginPacket();
    LoRa.print(dataStr);
    LoRa.endPacket();
  }
}

