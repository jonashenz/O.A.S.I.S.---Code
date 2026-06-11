# Projekt-Kontext: ESP32-S3 mit 3x VL53L0X & 3x DC-Motoren

## Hardware-Setup
- **MCU:** ESP32-S3 (DFR0975 DevBoard)
- **Sensoren:** 3x VL53L0X (ToF) an einem I2C-Bus.
- **Aktoren:** 3x DC-Motoren via NDS355AN (Logic-Level N-Channel MOSFETs) in Low-Side-Konfiguration.
- **Schutzbeschaltung:** 200Ω Gate-Widerstände, 10kΩ Pull-Downs, NBR0530T1G Freilaufdioden.

## Pin-Belegung (Aktueller Stand)
- **I2C:** SDA (IO10), SCL (IO11) mit 4.7kΩ Pull-Ups.
- **XSHUT (Adressierung):** - Sensor 1: IO47
  - Sensor 2: IO21
  - Sensor 3: IO12
- **Motor-Steuerung (PWM):**
  - Motor 1: IO18
  - Motor 2: IO14
  - Motor 3: IO13 (**WICHTIG:** Wurde von IO0 wegverlegt, da IO0 ein Strapping-Pin ist und mit Pull-Down den Bootvorgang blockiert).

## Kritische Learnings & Logik
1. **I2C-Adresskonflikt:** Alle VL53L0X starten auf 0x29. Lösung: Sequenzielles Booten via XSHUT. Erst S1 aktivieren -> Adresse ändern -> S2 aktivieren -> Adresse ändern usw.
2. **Strapping-Pins:** GPIO0 darf beim ESP32-S3 beim Booten nicht via Pull-Down auf LOW gezogen werden (erzwingt Download-Modus).
3. **Optische Interferenz:** Um Cross-talk zwischen den ToF-Sensoren zu vermeiden, wird der "Single Shot Mode" genutzt. Der ESP32 triggert die Sensoren nacheinander per I2C-Befehl, nicht gleichzeitig.
4. **XSHUT-Status:** Die XSHUT-Leitungen werden nur für die Initialisierung/Reset genutzt. Im laufenden Betrieb bleiben sie dauerhaft HIGH, um die zugewiesenen Adressen nicht zu verlieren.
5. **Stromversorgung:** Achtung bei gemeinsamer VCC-Schiene für Motoren und MCU (Brownout-Gefahr). Empfehlung: Puffer-Kondensatoren (min. 470µF) oder getrennte Versorgung.