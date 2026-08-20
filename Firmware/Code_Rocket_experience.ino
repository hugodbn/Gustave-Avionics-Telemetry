#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM6DSOX.h>
#include <SPI.h>
#include <DW1000.h>
#include <LoRa.h>
#include <SparkFun_u-blox_GNSS_v3.h>

Adafruit_BMP3XX bmp; // BMP
Adafruit_LSM6DSOX sox; // IMU (Unité de Mesure Inertielle)
SFE_UBLOX_GNSS myGNSS; // GPS

// Fichier CSV
File fichierPrincipal; // Fichier principal 
File fichierBackup;   // Fichier backup 

// PIN
int led1 = 5; // LED1
int led2 = 40; // LED_RDY
int Jack = 35; // SW_CNS_1 
int RESET_GPS = 4; // RESET_GPS
int CSD = BUILTIN_SDCARD;

// Moteurs (éjection CanSats)
int motor1 = 23; // SNG_CNS_1
int motor2 = 22; // SNG_CNS_2
int motor3 = 21; // SNG_CNS_3

// Interface JST
int SW_RDY  = 41;
int SW_CNS1 = 17;
int SW_CNS2 = 16;
int SW_CNS3 = 15;

// Variables pour stocker l'état de l'interface
int etat_SW_RDY  = 0;
int etat_SW_CNS1 = 0;
int etat_SW_CNS2 = 0;
int etat_SW_CNS3 = 0;

// LoRa
int LORA_cs = 14;
int LORA_RST = 30;
int LORA_DIO0 = 34;

// DW1000
const uint8_t PIN_RST = 7;
const uint8_t PIN_IRQ = 0;
const uint8_t PIN_CS  = 10;

// TODO replace by enum
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

// message flow state
volatile byte expectedMsgId  = POLL;

// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;

// protocol error state
boolean protocolFailed = false;

// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;

// last computed range/time
DW1000Time timeComputedRange;

// data buffer
#define LEN_DATA 16
byte data[LEN_DATA];

// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;

// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 3000;

// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod  = 0;
float samplingRate = 0;

// Distances des 6 CanSats possibles (index = CANSAT_ID - 1)
float distances[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// Variables Vol
bool systemeActif = true;
bool carteSD = false;
uint32_t tempsDepart  = 0;

// Durée max d'enregistrement : 10 minutes
const uint32_t DUREE_MAX_VOL = 600000;

// GPS
double lat = 0;
double lon = 0;
float altGPS = 0;
byte SIV  = 0;

// Détection atterrissage
const float SEUIL_ACCEL_SOL = 1.5;
const float SEUIL_ALTITUDE_SOL = 20.0;
const uint32_t DUREE_DETECTION = 10000;
uint32_t tempsAuSol = 0;
bool candidatAuSol = false;
float pressionSol = 1013.25; // Calibrée au setup

void setup() {
  Serial.begin(115200);

  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  digitalWrite(led2, HIGH); // LED_RDY allumée = en attente

  pinMode(Jack, INPUT_PULLUP);

  // Carte SD
  if (!SD.begin(CSD)) {
    signalErreur(2);
  }
  carteSD = true;

  // Ouverture des fichiers (gardés ouverts tout le vol)
  fichierPrincipal = SD.open("data.csv", FILE_WRITE);
  fichierBackup    = SD.open("backup.csv", FILE_WRITE);
  if (fichierPrincipal) {
    fichierPrincipal.println("sep=;");
    fichierPrincipal.println("ID;TimeGPS;Temps(ms);Pression;Temp_C;"
                             "AccX;AccY;AccZ;GyrX;GyrY;GyrZ;"
                             "Lat;Lon;AltGPS;AltBaro;SIV;"
                             "Dist1;Dist2;Dist3;Dist4;Dist5;Dist6;"
                             "SW_RDY;SW1;SW2;SW3"); // Dist = distance cansat
    fichierPrincipal.flush();
  }
  if (fichierBackup) {
    fichierBackup.println("sep=;");
    fichierBackup.println("ID;TimeGPS;Temps(ms);Pression;Temp_C;"
                          "AccX;AccY;AccZ;GyrX;GyrY;GyrZ;"
                          "Lat;Lon;AltGPS;AltBaro;SIV;"
                          "Dist1;Dist2;Dist3;Dist4;Dist5;Dist6;"
                          "SW_RDY;SW1;SW2;SW3");
    fichierBackup.flush();
  }

  // BMP
  if (!bmp.begin_I2C()) {
    signalErreur(3);
  }
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  // Calibration pression sol (moyenne 10 lectures)
  float sumPression = 0;
  int validReadings = 0;
  for (int i = 0; i < 10; i++) {
    if (bmp.performReading()) {
      sumPression += bmp.pressure / 100.0;
      validReadings++;
    }
  delay(50);
  }
  if (validReadings > 0) {
    pressionSol = sumPression / validReadings;
  }

  // Gyroscope (IMU)
  if (!sox.begin_I2C()) {
    signalErreur(4);
  }
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

  // DW1000 (UWB Anchor)
  // initialize the driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_CS);

  // general configuration
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(1); // Fusée = adresse 1 (CanSats = CANSAT_ID + 1)
  DW1000.setNetworkId(10);
  DW1000.setChannel(5);
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_ACCURACY);
  DW1000.useSmartPower(false); // Désactive le Smart TX Power 
  DW1000.commitConfiguration();
  DW1000.setTXPower(0x85858585); // valeur par défaut Decawave (APS023)
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);
  receiver(); // La fusée démarre en écoute permanente 
  noteActivity();
  rangingCountPeriod = millis();

  // LoRa
  SPI1.setMISO(1);
  SPI1.setMOSI(26);
  SPI1.setSCK(27);
  SPI1.begin();

  LoRa.setSPI(SPI1);
  LoRa.setPins(LORA_cs, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(868E6)) {
    signalErreur(6);
  }
  LoRa.setSpreadingFactor(7); // Rapide pour du temps réel
  LoRa.setTxPower(14); // Puissance max légale
 
  // GPS
  pinMode(RESET_GPS, OUTPUT);
  digitalWrite(RESET_GPS, LOW);
  delay(10);
  digitalWrite(RESET_GPS, HIGH);
  delay(100);

  Serial7.begin(38400);

  if (!myGNSS.begin(Serial7)) {
    signalErreur(5);
  }
  myGNSS.setUART1Output(COM_TYPE_UBX);
  myGNSS.setNavigationFrequency(5);
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);
  myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g); // Optimise le GPS pour les hautes accélérations

  // Interface JST 
  pinMode(SW_RDY,  INPUT_PULLUP);
  pinMode(SW_CNS1, INPUT_PULLUP);
  pinMode(SW_CNS2, INPUT_PULLUP);
  pinMode(SW_CNS3, INPUT_PULLUP);

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);

  analogWrite(motor1, 0);
  analogWrite(motor2, 0);
  analogWrite(motor3, 0);
}

void loop() {
  int lancement = demarage_system();

  if (lancement == 1 && systemeActif && carteSD) {

    // Arrêt automatique après DUREE_MAX_VOL (10 min)
    if (tempsDepart > 0 && millis() - tempsDepart > DUREE_MAX_VOL) {
      systemeActif = false;
      return;
    }

    etat_SW_RDY = digitalRead(SW_RDY);

    if (etat_SW_RDY == HIGH) {
      analogWrite(motor1, 90); // Lancement Cansat
      analogWrite(motor2, 90);
      analogWrite(motor3, 90);
    }

    Lcapteur(); // Enregistrement des données

  } else if (!systemeActif) {
    digitalWrite(led1, HIGH); // Fin de mission
  }
}

int demarage_system() {
  int etatJack = digitalRead(Jack);

  if (etatJack == HIGH) { // Jack retiré 
    if (tempsDepart == 0) {
      tempsDepart = millis(); // Démarre le chrono une seule fois
    }
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    return 1;
  } else {
    digitalWrite(led2, HIGH); // En attente
    return 0;
  }
}

void Lcapteur() {
  uint32_t curMillis = millis();

  // DW1000 UWB
  if (sentAck) {
    sentAck = false;
    if (data[0] == POLL_ACK) {
      DW1000.getTransmitTimestamp(timePollAckSent);
      noteActivity();
    }
  }

  if (receivedAck) {
    receivedAck = false;
    DW1000.getData(data, LEN_DATA);
    byte msgId = data[0];

    if (msgId == POLL) {
      protocolFailed = false;
      DW1000.getReceiveTimestamp(timePollReceived);
      expectedMsgId = RANGE;
      transmitPollAck();
      noteActivity();
    }
    else if (msgId == RANGE) {
      DW1000.getReceiveTimestamp(timeRangeReceived);
      expectedMsgId = POLL;

      if (!protocolFailed) {
        timePollSent.setTimestamp(data + 1);
        timePollAckReceived.setTimestamp(data + 6);
        timeRangeSent.setTimestamp(data + 11);
        computeRangeAsymmetric();

        byte cansatID = data[15]; // L'ID est dans le dernier octet du buffer
        float distCalculee = timeComputedRange.getAsMeters();

        if (cansatID >= 1 && cansatID <= 6) {
          distances[cansatID - 1] = distCalculee;
        }

        transmitRangeReport(timeComputedRange.getAsMicroSeconds());
        successRangingCount++;

        if (curMillis - rangingCountPeriod > 1000) {
          samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
          rangingCountPeriod = curMillis;
          successRangingCount = 0;
        }
      } else {
        transmitRangeFailed();
      }
      noteActivity();
    }
  }

  // Reset si inactivité UWB
  if (curMillis - lastActivity > resetPeriod) {
    resetInactive();
  }

  // GPS
  char timeGPSBuf[24] = "00:00:00.000";
  if (myGNSS.getPVT()) {
    lat = myGNSS.getLatitude() / 10000000.0;
    lon = myGNSS.getLongitude() / 10000000.0;
    altGPS = myGNSS.getAltitude() / 1000.0;
    SIV = myGNSS.getSIV();

    // TimeGPS pour synchronisation post-prod avec les CSV CanSats
    sprintf(timeGPSBuf, "%02d:%02d:%02d.%03d",
            myGNSS.getHour(), myGNSS.getMinute(),
            myGNSS.getSecond(), myGNSS.getMillisecond());
  }

  // BMP
  float pression = 0, temperature = 0, altBaro = 0;
  if (bmp.performReading()) {
  pression = bmp.pressure / 100.0;
  temperature = bmp.temperature;
  altBaro = bmp.readAltitude(pressionSol);
  }

  // IMU 
  sensors_event_t accel, gyro, temp;
  sox.getEvent(&accel, &gyro, &temp);

  // Calcul de l'accélération totale
  float accTotale = sqrt(
    accel.acceleration.x * accel.acceleration.x +
    accel.acceleration.y * accel.acceleration.y +
    accel.acceleration.z * accel.acceleration.z
  );

  // Détection atterrissage
  bool accelStatique = abs(accTotale - 9.81) < SEUIL_ACCEL_SOL;
  bool altitudeBasse = altBaro < SEUIL_ALTITUDE_SOL;

  if (accelStatique && altitudeBasse) {
    if (!candidatAuSol) {
      candidatAuSol = true;
      tempsAuSol = millis();
    } else if (millis() - tempsAuSol >= DUREE_DETECTION) {
      systemeActif = false;
    }
  } else {
    candidatAuSol = false;
  }

  // Interface JST
  etat_SW_RDY = digitalRead(SW_RDY);
  etat_SW_CNS1 = digitalRead(SW_CNS1);
  etat_SW_CNS2 = digitalRead(SW_CNS2);
  etat_SW_CNS3 = digitalRead(SW_CNS3);

  // Formatage CSV 
  String dataline = String(0) + ";" +
                    String(timeGPSBuf) + ";" +
                    String(curMillis)  + ";" +
                    String(pression)   + ";" +
                    String(temperature) + ";" +
                    String(accel.acceleration.x) + ";" +
                    String(accel.acceleration.y) + ";" +
                    String(accel.acceleration.z) + ";" +
                    String(gyro.gyro.x) + ";" +
                    String(gyro.gyro.y) + ";" +
                    String(gyro.gyro.z) + ";" +
                    String(lat, 7) + ";" +
                    String(lon, 7) + ";" +
                    String(altGPS) + ";" +
                    String(altBaro) + ";" +
                    String(SIV) + ";" +
                    String(distances[0]) + ";" +
                    String(distances[1]) + ";" +
                    String(distances[2]) + ";" +
                    String(distances[3]) + ";" +
                    String(distances[4]) + ";" +
                    String(distances[5]) + ";" +
                    String(etat_SW_RDY) + ";" +
                    String(etat_SW_CNS1) + ";" +
                    String(etat_SW_CNS2) + ";" +
                    String(etat_SW_CNS3);

  // Carte SD (flush sans close) 
  if (fichierPrincipal) {
    fichierPrincipal.println(dataline);
    fichierPrincipal.flush();
  }
  if (fichierBackup) {
    fichierBackup.println(dataline);
    fichierBackup.flush();
  }

  // LoRa
  if (dataline.length() <= 255) {
    LoRa.beginPacket();
    LoRa.print(dataline);
    LoRa.endPacket();
  }
}

// DW100 fonction pour son fonctionnement 
void noteActivity()  { 
  lastActivity = millis(); 
}

void resetInactive() {
  expectedMsgId = POLL;
  receiver();
  noteActivity();
}

void handleSent() { 
  sentAck = true; 
}

void handleReceived(){ 
  receivedAck = true; 
}

void receiver() {
  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true); // Repasse auto en écoute après chaque TX
  DW1000.startReceive();
}

void transmitPollAck() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = POLL_ACK;
  DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
  DW1000.setDelay(deltaTime);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRangeReport(float curRange) {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE_REPORT;
  memcpy(data + 1, &curRange, 4);
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void transmitRangeFailed() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  data[0] = RANGE_FAILED;
  DW1000.setData(data, LEN_DATA);
  DW1000.startTransmit();
}

void computeRangeAsymmetric() {
  // TWR asymétrique : plus précis, compensé sur la dérive d'horloge
  DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
  DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
  DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
  DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
  DW1000Time tof = (round1 * round2 - reply1 * reply2) /
                      (round1 + round2 + reply1 + reply2);
  timeComputedRange.setTimestamp(tof);
}

void computeRangeSymmetric() {
  // TWR symétrique : moins de calcul, plus sensible à la dérive d'horloge
  DW1000Time tof = ((timePollAckReceived - timePollSent) -
                    (timePollAckSent - timePollReceived) +
                    (timeRangeReceived - timePollAckSent) -
                    (timeRangeSent - timePollAckReceived)) * 0.25f;
  timeComputedRange.setTimestamp(tof);
}

// Fonction pour signaler une erreur spécifique et bloquer le système
void signalErreur(int clignotements) {
  while (1) {
    for (int i = 0; i < clignotements; i++) {
      digitalWrite(led1, HIGH); delay(200);
      digitalWrite(led1, LOW);  delay(200);
    }
    delay(1000);
  }
}
