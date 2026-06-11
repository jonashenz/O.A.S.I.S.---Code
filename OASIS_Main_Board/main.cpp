/* ============================================================================
 * OASIS — Haptisches Navigationssystem
 * ============================================================================
 * Board:    DFRobot FireBeetle 2 ESP32-S3 (DFR0975)
 * Sensoren: 3x VL53L0X ToF (Time-of-Flight, IR, I2C)
 * Aktoren:  3x DC ERM Vibrationsmotoren via N-CH MOSFET (NDS355AN)
 *
 * Funktion:
 *   Drei IR-Sensoren messen Distanzen zu Hindernissen. Drei Vibrationsmotoren
 *   geben pulsierendes haptisches Feedback nach dem Vorbild von Auto-Park-
 *   sensoren. Mapping ist 1:1 fest verkabelt (S1->M1, S2->M2, S3->M3).
 *
 *   Vibrationsprofil:
 *     - Drei diskrete Zonen mit fester Pulsfrequenz (mittel/fern)
 *     - In der NAHEN Zone steigt die Pulsfrequenz EXPONENTIELL (Eskalation)
 *     - Bei KRITISCH durchgehende Vibration
 *
 *   Warum nicht exponentielle PWM-Stärke?
 *     -> Weber-Fechner-Gesetz: Mensch nimmt Vibration logarithmisch wahr.
 *     -> Über PWM 220 kein Wahrnehmungsgewinn mehr, nur Stromverschwendung.
 *     -> Frequenzänderung dagegen sehr gut wahrnehmbar bis ca. 10 Hz.
 *
 * Autor:    Jonas Henz
 * Schule:   BÜP Elektroniker EFZ, 3. Lehrjahr
 * Version:  4.0  (Mai 2026 — exponentielle Frequenz, Offset)
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
const uint32_t I2C_BUS_SPEED_HZ = 400000;   // 400 kHz Fast-Mode

// XSHUT-Pins (Sensor Reset/Enable für Adress-Vergabe beim Boot)
const uint8_t  PIN_XSHUT_VORN   = 47;
const uint8_t  PIN_XSHUT_MITTE  = 21;       // Achtung: auch On-Board LED!
const uint8_t  PIN_XSHUT_HINTEN = 12;

// PWM-Ausgänge zu MOSFET-Gates der Motoren
const uint8_t  PIN_MOTOR_VORN   = 18;
const uint8_t  PIN_MOTOR_MITTE  = 14;
const uint8_t  PIN_MOTOR_HINTEN = 13;

// I2C-Adressen (aus Default 0x29 beim Boot umprogrammiert)
const uint8_t  ADRESSE_SENSOR_VORN   = 0x30;
const uint8_t  ADRESSE_SENSOR_MITTE  = 0x31;
const uint8_t  ADRESSE_SENSOR_HINTEN = 0x32;

/* ============================================================================
 * VIBRATIONS-PROFIL — hier alle Werte zentral anpassen
 * ============================================================================
 *
 *   Distanz       Verhalten                              Wahrnehmung
 *   ──────────────────────────────────────────────────────────────────────
 *   >= ZONE_FERN          Aus                            Stille
 *   ZONE_MITTEL..FERN     Langsames Pulsen (1 Hz)        "etwas da hinten"
 *   ZONE_NAH..MITTEL      Mittleres Pulsen (3 Hz)        "kommt näher"
 *   ZONE_KRITISCH..NAH    Pulsen 4 -> 12 Hz EXPONENTIELL "ESKALATION"
 *   <= ZONE_KRITISCH      Dauervibration                 "STOPP, kollidiert!"
 *
 * Tipp: Mit DISTANZ_OFFSET_CM kannst du ALLE Zonen gleichzeitig verschieben,
 *       ohne jeden Wert einzeln anzupassen.
 * ============================================================================ */

// --- GLOBALER OFFSET: verschiebt alle Zonen-Grenzen gleichzeitig ---
// Positiv  = sensibler (alles reagiert früher, z.B. +20 -> Aus erst bei 120 cm)
// Negativ  = unempfindlicher (alles reagiert später, z.B. -20 -> Aus schon bei 80 cm)
//   0 = Standard
const int16_t DISTANZ_OFFSET_CM = 0;

// --- Zonen-Grenzen in cm (Basiswerte, werden mit Offset addiert) ---
const uint16_t ZONE_KRITISCH_BASIS_CM = 10;
const uint16_t ZONE_NAH_BASIS_CM      = 25;
const uint16_t ZONE_MITTEL_BASIS_CM   = 50;
const uint16_t ZONE_FERN_BASIS_CM     = 100;

// --- Pulsfrequenzen für die festen Zonen ---
const float FREQ_FERN_HZ        = 1.0f;   // 1 Puls pro Sekunde
const float FREQ_MITTEL_HZ      = 3.0f;   // 3 Pulse pro Sekunde

// --- Exponentielle Eskalation in der NAHEN Zone ---
// An der Aussengrenze der nahen Zone:    FREQ_NAH_MIN_HZ
// An der Innengrenze (Übergang kritisch): FREQ_NAH_MAX_HZ
// Die Frequenz steigt exponentiell dazwischen.
const float FREQ_NAH_MIN_HZ     = 4.0f;   // bei ZONE_NAH
const float FREQ_NAH_MAX_HZ     = 12.0f;  // bei ZONE_KRITISCH

// --- Vibrationsstärke pro Zone (PWM 0-255) ---
// Bewusst NICHT mehr stark gestaffelt: Mensch unterscheidet schlecht.
// Mehr Stärke kommt nur bei "kritisch", den Rest macht die Frequenz.
const uint8_t STAERKE_FERN      = 130;   // ~50% — sanft aber spürbar
const uint8_t STAERKE_MITTEL    = 180;   // ~70% — deutlich
const uint8_t STAERKE_NAH       = 220;   // ~85% — kraftvoll
const uint8_t STAERKE_KRITISCH  = 255;   // 100% — max

// Wenn der Sensor kein Echo bekommt, gilt "Hindernis weit weg"
const uint16_t DISTANZ_KEIN_ECHO_CM = 200;

/* ============================================================================
 * INTERNE EINSTELLUNGEN — normalerweise nicht ändern
 * ============================================================================ */

// PWM-Konfiguration (ESP32 LEDC Hardware-Peripherie)
const uint32_t PWM_FREQUENZ_HZ  = 20000;   // 20 kHz (über Hörbereich)
const uint8_t  PWM_AUFLOESUNG   = 8;       // 8 Bit -> 0..255

// LEDC-Kanäle
const uint8_t  PWM_KANAL_VORN   = 0;
const uint8_t  PWM_KANAL_MITTE  = 1;
const uint8_t  PWM_KANAL_HINTEN = 2;

// Timing
const uint32_t MESSINTERVALL_MS = 50;
const uint32_t DEBUG_AUSGABE_MS = 250;

// Tiefpassfilter — glättet Sensor-Rauschen
// 0.0 = sehr träge, 1.0 = sehr nervös
const float    FILTER_STAERKE   = 0.4f;

/* ============================================================================
 * EFFEKTIVE ZONEN-GRENZEN (mit Offset) — werden in setup() berechnet
 * ============================================================================ */
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
 * FUNKTION: Einen Sensor initialisieren mit eindeutiger I2C-Adresse
 *
 * Ablauf:
 *   1. XSHUT HIGH  -> Sensor wacht auf, antwortet auf Default 0x29
 *   2. begin(neueAdresse) -> Library schreibt neue Adresse ins Sensor-Register
 *   3. Sensor antwortet ab jetzt nur noch auf die neue Adresse
 *
 * Wichtig: Sensoren MÜSSEN nacheinander aktiviert werden (nie parallel).
 * ============================================================================ */
bool sensorAktivieren(Adafruit_VL53L0X &sensor, uint8_t xshutPin,
                      uint8_t neueAdresse, const char* bezeichnung) {
  Serial.print("  -> ");
  Serial.print(bezeichnung);
  Serial.print(" (Adresse 0x");
  Serial.print(neueAdresse, HEX);
  Serial.print(") ... ");

  digitalWrite(xshutPin, HIGH);
  delay(20);  // Boot-Zeit VL53L0X (Datenblatt ~1.2 ms, sicher 20 ms)

  if (!sensor.begin(neueAdresse, false, &Wire)) {
    Serial.println("FEHLER!");
    return false;
  }
  Serial.println("OK");
  return true;
}

/* ============================================================================
 * FUNKTION: Distanz lesen (in cm)
 * ============================================================================ */
uint16_t distanzLesen(Adafruit_VL53L0X &sensor, bool sensorAktiv) {
  if (!sensorAktiv) {
    return DISTANZ_KEIN_ECHO_CM;
  }

  VL53L0X_RangingMeasurementData_t messung;
  sensor.rangingTest(&messung, false);

  // Status 4 = "Out of Range"
  if (messung.RangeStatus == 4) {
    return DISTANZ_KEIN_ECHO_CM;
  }

  uint16_t cm = messung.RangeMilliMeter / 10;
  if (cm > DISTANZ_KEIN_ECHO_CM) {
    cm = DISTANZ_KEIN_ECHO_CM;
  }
  return cm;
}

/* ============================================================================
 * FUNKTION: Frequenz in der NAHEN Zone exponentiell berechnen
 *
 * Eingabe:  distanzCm — irgendwo zwischen zoneKritischCm und zoneNahCm
 * Ausgabe:  Pulsfrequenz in Hz
 *
 * Mathematik:
 *   Wir mappen die Distanz auf einen Faktor t von 0 bis 1:
 *     t = 0  bei der ÄUSSEREN Grenze (zoneNahCm)        -> FREQ_NAH_MIN_HZ
 *     t = 1  bei der INNEREN Grenze (zoneKritischCm)    -> FREQ_NAH_MAX_HZ
 *
 *   Dann skalieren wir die Frequenz exponentiell:
 *     freq = FREQ_NAH_MIN_HZ * (FREQ_NAH_MAX_HZ / FREQ_NAH_MIN_HZ) ^ t
 *
 *   Beispiel mit 4 Hz -> 12 Hz:
 *     bei t=0.0 (25 cm):  4 Hz
 *     bei t=0.5 (17.5 cm): 6.93 Hz
 *     bei t=1.0 (10 cm):  12 Hz
 * ============================================================================ */
float frequenzNaheZoneExp(float distanzCm) {
  // Distanzbereich: nahe Zone geht von zoneNahCm (aussen) nach zoneKritischCm (innen)
  float zonenBreite = (float)(zoneNahCm - zoneKritischCm);
  if (zonenBreite <= 0) return FREQ_NAH_MAX_HZ;  // Sicherheit

  // t = Position innerhalb der Zone (0 = aussen, 1 = innen, je näher desto höher)
  float t = (zoneNahCm - distanzCm) / zonenBreite;

  // Begrenzen auf [0, 1]
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;

  // Exponentielle Interpolation
  return FREQ_NAH_MIN_HZ * powf(FREQ_NAH_MAX_HZ / FREQ_NAH_MIN_HZ, t);
}

/* ============================================================================
 * FUNKTION: Vibrationsmuster berechnen (Auto-Parksensor + exponentielle Eskalation)
 * ============================================================================ */
uint8_t vibrationBerechnen(float distanzCm, uint32_t jetztMs) {

  // --- Zone "AUS" (>= 100 cm + Offset) ---
  if (distanzCm >= zoneFernCm) {
    return 0;
  }

  // --- Zone "KRITISCH" (<= 10 cm + Offset) — Dauervibration ---
  if (distanzCm <= zoneKritischCm) {
    return STAERKE_KRITISCH;
  }

  // --- Zonen mit Pulsen: Stärke und Frequenz festlegen ---
  uint8_t  staerke;
  float    frequenzHz;

  if (distanzCm <= zoneNahCm) {
    // 10–25 cm: NAHE Zone — exponentielle Frequenz-Eskalation
    staerke    = STAERKE_NAH;
    frequenzHz = frequenzNaheZoneExp(distanzCm);
  }
  else if (distanzCm <= zoneMittelCm) {
    // 25–50 cm: feste mittlere Frequenz
    staerke    = STAERKE_MITTEL;
    frequenzHz = FREQ_MITTEL_HZ;
  }
  else {
    // 50–100 cm: feste langsame Frequenz
    staerke    = STAERKE_FERN;
    frequenzHz = FREQ_FERN_HZ;
  }

  // --- Pulsen erzeugen ---
  // Pulsdauer = 1/Frequenz, Tastverhältnis 50% AN / 50% AUS
  uint32_t pulsdauerMs = (uint32_t)(1000.0f / frequenzHz);
  if (pulsdauerMs == 0) pulsdauerMs = 1;  // Sicherheit gegen Division durch null

  uint32_t phase = jetztMs % pulsdauerMs;

  if (phase < (pulsdauerMs / 2)) {
    return staerke;
  } else {
    return 0;
  }
}

/* ============================================================================
 * FUNKTION: Aktuelle Zone als Text (nur Debug)
 * ============================================================================ */
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
  delay(2000);

  Serial.println();
  Serial.println("==========================================");
  Serial.println("  OASIS - Haptisches Navigationssystem");
  Serial.println("  v4.0 - Auto-Profil + Exp.-Eskalation");
  Serial.println("==========================================");

  // --- Effektive Zonen-Grenzen mit Offset berechnen ---
  zoneKritischCm = ZONE_KRITISCH_BASIS_CM + DISTANZ_OFFSET_CM;
  zoneNahCm      = ZONE_NAH_BASIS_CM      + DISTANZ_OFFSET_CM;
  zoneMittelCm   = ZONE_MITTEL_BASIS_CM   + DISTANZ_OFFSET_CM;
  zoneFernCm     = ZONE_FERN_BASIS_CM     + DISTANZ_OFFSET_CM;

  // Sicherheit: Zonen dürfen nicht negativ werden
  if (zoneKritischCm < 1) zoneKritischCm = 1;
  if (zoneNahCm < zoneKritischCm + 1) zoneNahCm = zoneKritischCm + 1;
  if (zoneMittelCm < zoneNahCm + 1)   zoneMittelCm = zoneNahCm + 1;
  if (zoneFernCm < zoneMittelCm + 1)  zoneFernCm = zoneMittelCm + 1;

  // --- 1. Sensoren in Reset ---
  Serial.println("\n[1/4] Sensor-Reset (alle XSHUT auf LOW)");
  pinMode(PIN_XSHUT_VORN,   OUTPUT);
  pinMode(PIN_XSHUT_MITTE,  OUTPUT);
  pinMode(PIN_XSHUT_HINTEN, OUTPUT);
  digitalWrite(PIN_XSHUT_VORN,   LOW);
  digitalWrite(PIN_XSHUT_MITTE,  LOW);
  digitalWrite(PIN_XSHUT_HINTEN, LOW);
  delay(20);

  // --- 2. I2C-Bus starten ---
  Serial.print("[2/4] I2C-Bus starten (SDA=IO");
  Serial.print(PIN_I2C_SDA);
  Serial.print(", SCL=IO");
  Serial.print(PIN_I2C_SCL);
  Serial.println(", 400 kHz)");
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_SPEED_HZ);

  // --- 3. Sensoren einzeln aktivieren mit eigenen Adressen ---
  Serial.println("[3/4] Sensoren mit eindeutigen Adressen ausstatten:");
  sensorVornOk   = sensorAktivieren(sensorVorn,   PIN_XSHUT_VORN,
                                    ADRESSE_SENSOR_VORN,   "Sensor VORN  ");
  sensorMitteOk  = sensorAktivieren(sensorMitte,  PIN_XSHUT_MITTE,
                                    ADRESSE_SENSOR_MITTE,  "Sensor MITTE ");
  sensorHintenOk = sensorAktivieren(sensorHinten, PIN_XSHUT_HINTEN,
                                    ADRESSE_SENSOR_HINTEN, "Sensor HINTEN");

  uint8_t anzahl = (sensorVornOk?1:0) + (sensorMitteOk?1:0) + (sensorHintenOk?1:0);
  Serial.print("  -> ");
  Serial.print(anzahl);
  Serial.println(" von 3 Sensoren bereit");

  // --- 4. PWM-Kanäle einrichten ---
  Serial.println("[4/4] PWM-Kanaele fuer Motoren einrichten (20 kHz, 8 Bit)");

  ledcSetup(PWM_KANAL_VORN,   PWM_FREQUENZ_HZ, PWM_AUFLOESUNG);
  ledcSetup(PWM_KANAL_MITTE,  PWM_FREQUENZ_HZ, PWM_AUFLOESUNG);
  ledcSetup(PWM_KANAL_HINTEN, PWM_FREQUENZ_HZ, PWM_AUFLOESUNG);

  ledcAttachPin(PIN_MOTOR_VORN,   PWM_KANAL_VORN);
  ledcAttachPin(PIN_MOTOR_MITTE,  PWM_KANAL_MITTE);
  ledcAttachPin(PIN_MOTOR_HINTEN, PWM_KANAL_HINTEN);

  ledcWrite(PWM_KANAL_VORN,   0);
  ledcWrite(PWM_KANAL_MITTE,  0);
  ledcWrite(PWM_KANAL_HINTEN, 0);

  // --- Zonen-Tabelle ausgeben ---
  Serial.println("\n==========================================");
  Serial.println("  System bereit. Hauptschleife startet.");
  Serial.println("==========================================");
  Serial.print("Distanz-Offset: ");
  Serial.print(DISTANZ_OFFSET_CM);
  Serial.println(" cm");
  Serial.println("\nVibrations-Zonen:");
  Serial.print("  > "); Serial.print(zoneFernCm);     Serial.println(" cm        -> AUS");
  Serial.print("  ");   Serial.print(zoneMittelCm);   Serial.print("-");
  Serial.print(zoneFernCm);     Serial.print(" cm    -> Pulsen ");
  Serial.print(FREQ_FERN_HZ);   Serial.println(" Hz");
  Serial.print("  ");   Serial.print(zoneNahCm);      Serial.print("-");
  Serial.print(zoneMittelCm);   Serial.print(" cm     -> Pulsen ");
  Serial.print(FREQ_MITTEL_HZ); Serial.println(" Hz");
  Serial.print("  ");   Serial.print(zoneKritischCm); Serial.print("-");
  Serial.print(zoneNahCm);      Serial.print(" cm     -> Pulsen ");
  Serial.print(FREQ_NAH_MIN_HZ); Serial.print("->");
  Serial.print(FREQ_NAH_MAX_HZ); Serial.println(" Hz (exp.)");
  Serial.print("  < "); Serial.print(zoneKritischCm); Serial.println(" cm         -> Dauervibration\n");
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

  // --- 2. Vibrationsmuster jeden Loop berechnen (für sauberes Pulsen) ---
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
