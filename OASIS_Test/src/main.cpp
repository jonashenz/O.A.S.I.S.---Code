/* ============================================================================
 * OASIS — Haptisches Navigationssystem
 * ============================================================================
 * Board:     DFRobot FireBeetle 2 ESP32-S3 (DFR0975)
 * Sensoren: 3x VL53L0X ToF (Time-of-Flight, IR, I2C)
 * Aktoren:  3x DC ERM Vibrationsmotoren via N-CH MOSFET (NDS355AN)
 *
 * Version:  5.0 (Hardware-Optimiert für feste Platine — Low-Speed Direct-Reg)
 * ============================================================================ */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "Adafruit_VL53L0X.h"

/* ============================================================================
 * HARDWARE-PINS — laut OASIS Schaltschema, nicht ändern!
 * ============================================================================ */

// I2C-Bus (gemeinsam für alle 3 Sensoren)
const uint8_t  PIN_I2C_SDA      = 10;
const uint8_t  PIN_I2C_SCL      = 11;
const uint32_t I2C_BUS_SPEED_HZ = 20000;    // Gedrosselt auf 20 kHz wegen langer Leitungen

// XSHUT-Pins (Sensor Reset/Enable für Adress-Vergabe beim Boot)
const uint8_t  PIN_XSHUT_VORN   = 47;
const uint8_t  PIN_XSHUT_MITTE  = 21;       // Achtung: auch On-Board LED!
const uint8_t  PIN_XSHUT_HINTEN = 12;

// PWM-Ausgänge zu MOSFET-Gates der Motoren
const uint8_t  PIN_MOTOR_VORN   = 18;
const uint8_t  PIN_MOTOR_MITTE  = 14;
const uint8_t  PIN_MOTOR_HINTEN = 13;

// I2C-Zieladressen (0x36 bleibt für dein Onboard-Gerät reserviert)
const uint8_t  ADRESSE_SENSOR_VORN   = 0x30;
const uint8_t  ADRESSE_SENSOR_MITTE  = 0x35;
const uint8_t  ADRESSE_SENSOR_HINTEN = 0x34;

/* ============================================================================
 * VIBRATIONS-PROFIL — Zentrale Parameter
 * ============================================================================ */
const int16_t DISTANZ_OFFSET_CM = 0;

const uint16_t ZONE_KRITISCH_BASIS_CM = 10;
const uint16_t ZONE_NAH_BASIS_CM      = 25;
const uint16_t ZONE_MITTEL_BASIS_CM   = 50;
const uint16_t ZONE_FERN_BASIS_CM     = 100;

const float FREQ_FERN_HZ        = 1.0f;   
const float FREQ_MITTEL_HZ      = 3.0f;   

const float FREQ_NAH_MIN_HZ     = 4.0f;   
const float FREQ_NAH_MAX_HZ     = 12.0f;  

const uint8_t STAERKE_FERN      = 130;   
const uint8_t STAERKE_MITTEL    = 180;   
const uint8_t STAERKE_NAH       = 220;   
const uint8_t STAERKE_KRITISCH  = 255;   

const uint16_t DISTANZ_KEIN_ECHO_CM = 200;

/* ============================================================================
 * INTERNE EINSTELLUNGEN
 * ============================================================================ */
const uint32_t PWM_FREQUENZ_HZ  = 20000;   // 20 kHz
const uint8_t  PWM_AUFLOESUNG   = 8;       // 8 Bit -> 0..255

const uint8_t  PWM_KANAL_VORN   = 0;
const uint8_t  PWM_KANAL_MITTE  = 1;
const uint8_t  PWM_KANAL_HINTEN = 2;

const uint32_t MESSINTERVALL_MS = 50;
const uint32_t DEBUG_AUSGABE_MS = 250;
const float    FILTER_STAERKE   = 0.4f;

int16_t zoneKritischCm;
int16_t zoneNahCm;
int16_t zoneMittelCm;
int16_t zoneFernCm;

/* ============================================================================
 * GLOBALE VARIABLEN
 * ============================================================================ */
Adafruit_VL53L0X sensorVorn;
Adafruit_VL53L0X sensorMitte;
Adafruit_VL53L0X sensorHinten;

bool   sensorVornOk   = false;
bool   sensorMitteOk  = false;
bool   sensorHintenOk = false;

float  distanzVorn    = DISTANZ_KEIN_ECHO_CM;
float  distanzMitte   = DISTANZ_KEIN_ECHO_CM;
float  distanzHinten  = DISTANZ_KEIN_ECHO_CM;

uint32_t letzteMessung      = 0;
uint32_t letzteDebugAusgabe = 0;

/* ============================================================================
 * HILFSFUNKTIONEN FÜR DIE HARDWARE-STEUERUNG
 * ============================================================================ */
bool ping(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// Beschreibt das Adressregister des VL53L0X direkt über rohes I2C
bool changeAddressRawAggressive(uint8_t oldAddr, uint8_t newAddr) {
  for (uint8_t i = 0; i < 10; i++) {
    Wire.beginTransmission(oldAddr);
    Wire.write(0x8A);             
    Wire.write(newAddr & 0x7F);   
    if (Wire.endTransmission() == 0) {
      delay(40);
      if (ping(newAddr)) return true;
    }
    delay(60);
  }
  return false;
}

uint16_t distanzLesen(Adafruit_VL53L0X &sensor, bool sensorAktiv) {
  if (!sensorAktiv) return DISTANZ_KEIN_ECHO_CM;

  VL53L0X_RangingMeasurementData_t messung;
  sensor.rangingTest(&messung, false);

  if (messung.RangeStatus == 4) return DISTANZ_KEIN_ECHO_CM;

  uint16_t cm = messung.RangeMilliMeter / 10;
  if (cm > DISTANZ_KEIN_ECHO_CM) cm = DISTANZ_KEIN_ECHO_CM;
  return cm;
}

float frequenzNaheZoneExp(float distanzCm) {
  float zonenBreite = (float)(zoneNahCm - zoneKritischCm);
  if (zonenBreite <= 0) return FREQ_NAH_MAX_HZ;

  float t = (zoneNahCm - distanzCm) / zonenBreite;
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  return FREQ_NAH_MIN_HZ * powf(FREQ_NAH_MAX_HZ / FREQ_NAH_MIN_HZ, t);
}

uint8_t vibrationBerechnen(float distanzCm, uint32_t jetztMs) {
  if (distanzCm >= zoneFernCm) return 0;
  if (distanzCm <= zoneKritischCm) return STAERKE_KRITISCH;

  uint8_t  staerke;
  float    frequenzHz;

  if (distanzCm <= zoneNahCm) {
    staerke    = STAERKE_NAH;
    frequenzHz = frequenzNaheZoneExp(distanzCm);
  }
  else if (distanzCm <= zoneMittelCm) {
    staerke    = STAERKE_MITTEL;
    frequenzHz = FREQ_MITTEL_HZ;
  }
  else {
    staerke    = STAERKE_FERN;
    frequenzHz = FREQ_FERN_HZ;
  }

  uint32_t pulsdauerMs = (uint32_t)(1000.0f / frequenzHz);
  if (pulsdauerMs == 0) pulsdauerMs = 1;

  uint32_t phase = jetztMs % pulsdauerMs;
  return (phase < (pulsdauerMs / 2)) ? staerke : 0;
}

const char* zoneAlsText(float distanzCm) {
  if (distanzCm >= zoneFernCm)     return "AUS     ";
  if (distanzCm <= zoneKritischCm) return "KRITISCH";
  if (distanzCm <= zoneNahCm)      return "NAH-EXP ";
  if (distanzCm <= zoneMittelCm)   return "MITTEL  ";
  return                                  "FERN    ";
}

/* ============================================================================
 * SETUP
 * ============================================================================ */
void setup() {
  Serial.begin(115200);
  delay(2500);

  Serial.println("\n==========================================");
  Serial.println("  OASIS - Haptisches Navigationssystem");
  Serial.println("  v5.0 - Hardware-Prototyp-Modus");
  Serial.println("==========================================");

  zoneKritischCm = ZONE_KRITISCH_BASIS_CM + DISTANZ_OFFSET_CM;
  zoneNahCm      = ZONE_NAH_BASIS_CM      + DISTANZ_OFFSET_CM;
  zoneMittelCm   = ZONE_MITTEL_BASIS_CM   + DISTANZ_OFFSET_CM;
  zoneFernCm     = ZONE_FERN_BASIS_CM     + DISTANZ_OFFSET_CM;

  if (zoneKritischCm < 1) zoneKritischCm = 1;
  if (zoneNahCm < zoneKritischCm + 1) zoneNahCm = zoneKritischCm + 1;
  if (zoneMittelCm < zoneNahCm + 1)   zoneMittelCm = zoneNahCm + 1;
  if (zoneFernCm < zoneMittelCm + 1)  zoneFernCm = zoneMittelCm + 1;

  // I2C-Bus zwingend zuerst starten (Verhindert Bus-Sperre durch geladene Leitungen)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_SPEED_HZ);
  delay(100);

  Serial.println("\n[1/4] Sensor-Reset & Drive Strength Anpassung");
  pinMode(PIN_XSHUT_VORN,   OUTPUT);
  pinMode(PIN_XSHUT_MITTE,  OUTPUT);
  pinMode(PIN_XSHUT_HINTEN, OUTPUT);

  // Maximale Leistung für steile Schaltflanken trotz MOSFET-Gates aktivieren
  gpio_set_drive_capability((gpio_num_t)PIN_XSHUT_VORN, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_XSHUT_MITTE, GPIO_DRIVE_CAP_3);
  gpio_set_drive_capability((gpio_num_t)PIN_XSHUT_HINTEN, GPIO_DRIVE_CAP_3);

  digitalWrite(PIN_XSHUT_VORN,   LOW);
  digitalWrite(PIN_XSHUT_MITTE,  LOW);
  digitalWrite(PIN_XSHUT_HINTEN, LOW);
  delay(500); 

  Serial.println("[2/4] Sequentielle Adressierung via Direct-Register...");

  // VORN
  digitalWrite(PIN_XSHUT_VORN, HIGH);
  delay(200);
  if (ping(0x29)) {
    if (changeAddressRawAggressive(0x29, ADRESSE_SENSOR_VORN)) {
      sensorVornOk = sensorVorn.begin(ADRESSE_SENSOR_VORN, false, &Wire);
      if(sensorVornOk) sensorVorn.setMeasurementTimingBudgetMicroSeconds(40000);
    }
  }

  // MITTE
  digitalWrite(PIN_XSHUT_MITTE, HIGH);
  delay(200);
  if (ping(0x29)) {
    if (changeAddressRawAggressive(0x29, ADRESSE_SENSOR_MITTE)) {
      sensorMitteOk = sensorMitte.begin(ADRESSE_SENSOR_MITTE, false, &Wire);
      if(sensorMitteOk) sensorMitte.setMeasurementTimingBudgetMicroSeconds(40000);
    }
  }

  // HINTEN
  digitalWrite(PIN_XSHUT_HINTEN, HIGH);
  delay(400); 
  if (ping(0x29)) {
    if (changeAddressRawAggressive(0x29, ADRESSE_SENSOR_HINTEN)) {
      sensorHintenOk = sensorHinten.begin(ADRESSE_SENSOR_HINTEN, false, &Wire);
      if(sensorHintenOk) sensorHinten.setMeasurementTimingBudgetMicroSeconds(40000);
    }
  }

  Serial.print("[3/4] Status: ");
  Serial.print((sensorVornOk?1:0) + (sensorMitteOk?1:0) + (sensorHintenOk?1:0));
  Serial.println(" von 3 ToF-Sensoren aktiv.");

  Serial.println("[4/4] PWM-Kanaele fuer Motoren einrichten (20 kHz, 8 Bit)");
  ledcSetup(PWM_KANAL_VORN,   PWM_FREQUENZ_HZ, PWM_AUFLOESUNG);
  ledcSetup(PWM_KANAL_MITTE,  PWM_FREQUENZ_HZ, PWM_AUFLOESUNG);
  ledcSetup(PWM_KANAL_HINTEN, PWM_FREQUENZ_HZ, PWM_AUFLOESUNG);

  // HIER DIE KORREKTUR: PWM_KANAL_... statt PIN_KANAL_... verwendet!
  ledcAttachPin(PIN_MOTOR_VORN,   PWM_KANAL_VORN);
  ledcAttachPin(PIN_MOTOR_MITTE,  PWM_KANAL_MITTE);
  ledcAttachPin(PIN_MOTOR_HINTEN, PWM_KANAL_HINTEN);

  ledcWrite(PWM_KANAL_VORN,   0);
  ledcWrite(PWM_KANAL_MITTE,  0);
  ledcWrite(PWM_KANAL_HINTEN, 0);

  Serial.println("\n==========================================");
  Serial.println("  OASIS Hauptschleife gestartet.");
  Serial.println("==========================================");
}

/* ============================================================================
 * LOOP
 * ============================================================================ */
void loop() {
  uint32_t jetzt = millis();

  // --- 1. Distanzen messen ---
  if (jetzt - letzteMessung >= MESSINTERVALL_MS) {
    letzteMessung = jetzt;

    uint16_t rohVorn   = distanzLesen(sensorVorn,   sensorVornOk);
    uint16_t rohMitte  = distanzLesen(sensorMitte,  sensorMitteOk);
    uint16_t rohHinten = distanzLesen(sensorHinten, sensorHintenOk);

    // Tiefpassfilter
    distanzVorn   = (1.0f - FILTER_STAERKE) * distanzVorn   + FILTER_STAERKE * rohVorn;
    distanzMitte  = (1.0f - FILTER_STAERKE) * distanzMitte  + FILTER_STAERKE * rohMitte;
    distanzHinten = (1.0f - FILTER_STAERKE) * distanzHinten + FILTER_STAERKE * rohHinten;
  }

  // --- 2. Vibrationsmuster berechnen und an Motoren ausgeben ---
  uint8_t pwmVorn   = vibrationBerechnen(distanzVorn,   jetzt);
  uint8_t pwmMitte  = vibrationBerechnen(distanzMitte,  jetzt);
  uint8_t pwmHinten = vibrationBerechnen(distanzHinten, jetzt);

  ledcWrite(PWM_KANAL_VORN,   pwmVorn);
  ledcWrite(PWM_KANAL_MITTE,  pwmMitte);
  ledcWrite(PWM_KANAL_HINTEN, pwmHinten);

  // --- 3. Debug-Ausgabe ---
  if (jetzt - letzteDebugAusgabe >= DEBUG_AUSGABE_MS) {
    letzteDebugAusgabe = jetzt;

    Serial.print("VORN  ");   Serial.print((int)distanzVorn);   Serial.print(" cm [");
    Serial.print(zoneAlsText(distanzVorn));   Serial.print("]  |  ");
    Serial.print("MITTE ");   Serial.print((int)distanzMitte);  Serial.print(" cm [");
    Serial.print(zoneAlsText(distanzMitte));  Serial.print("]  |  ");
    Serial.print("HINTEN ");  Serial.print((int)distanzHinten); Serial.print(" cm [");
    Serial.print(zoneAlsText(distanzHinten)); Serial.println("]");
  }
}