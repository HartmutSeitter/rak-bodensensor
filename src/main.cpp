#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <SPI.h>
#include <ModbusMaster.h>

// ============================================================
// BufferedStream: puffert write(byte)-Aufrufe von ModbusMaster
// und sendet ALLES in einem einzigen flush()-DMA-Transfer.
// Verhindert FreeRTOS-Task-Switch-Lücken zwischen Bytes
// (Modbus RTU: max. 1,5 Zeichen Gap = 3,1ms bei 4800 Baud).
// ============================================================
class BufferedStream : public Stream {
public:
    void attach(HardwareSerial &s) { _s = &s; _pos = 0; }
    int available() override { return _s ? _s->available() : 0; }
    int read()      override { return _s ? _s->read()      : -1; }
    int peek()      override { return _s ? _s->peek()      : -1; }
    size_t write(uint8_t b) override {
        if (_pos < (int)sizeof(_buf)) _buf[_pos++] = b;
        return 1;
    }
    void flush() override {
        if (_s && _pos > 0) { _s->write(_buf, _pos); _s->flush(); }
        _pos = 0;
    }
    operator bool() { return _s != nullptr; }
private:
    HardwareSerial *_s = nullptr;
    uint8_t _buf[32];
    int     _pos = 0;
};

BufferedStream modbusSerial;
// Modbus Instanz
ModbusMaster node;

// Pins für RAK5802 (RS485)
#define RS485_TX 16
#define RS485_RX 15

// 1. SPI Instance for the Linker
SPIClass SPI_LORA(NRF_SPIM3, 45, 43, 44); 

// 2. OTAA Keys — per NODE_ID selektiert (Build-Flag in platformio.ini)
#if NODE_ID == 1  // hs-rak-bodensensor-panorama
uint8_t nodeDeviceEUI[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x28, 0x5A, 0xB2};
uint8_t nodeAppEUI[8]    = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t nodeAppKey[16]   = {***REMOVED***};
#elif NODE_ID == 2  // hs-bodensensor-beet-sued-mitte
uint8_t nodeDeviceEUI[8] = {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x70, 0x23};
uint8_t nodeAppEUI[8]    = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
uint8_t nodeAppKey[16]   = {***REMOVED***};
#else
#error "NODE_ID nicht definiert — in platformio.ini env Build-Flag setzen"
#endif

// 3. Payload structures (Fixed the 'lmh' typo here)
lmh_app_data_t m_lora_app_data;
uint8_t m_lora_app_data_buffer[64];

static uint32_t sendInterval = 60000; // 1 erste Übertragung nach 1 Minute
static volatile bool joined  = false;  // Join-Flag — volatile wegen Callback-Kontext

// Callbacks
void lorawan_has_joined_handler(void) { Serial.println(">>> JOIN OK! <<<"); joined = true; }
void lorawan_join_failed_handler(eDeviceClass deviceClass) { Serial.println("Join Failed!"); }
void lorawan_rx_handler(lmh_app_data_t *app_data) { Serial.println("Downlink!"); }

static lmh_callback_t lora_callbacks = {
    BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
    lorawan_rx_handler, lorawan_has_joined_handler, lorawan_join_failed_handler, NULL
};
// postTransmission: nach dem letzten Byte → Buffer rausschicken (atomisch via DMA)
void postTransmission() {
    modbusSerial.flush();  // sendet alle gepufferten Bytes in einem DMA-Transfer
    delay(5);              // warte bis letztes Stop-Bit physisch gesendet (2ms bei 4800 Bd)
}

void initModbus() {
    Serial1.begin(4800, SERIAL_8N1);
    modbusSerial.attach(Serial1);   // BufferedStream an Serial1 koppeln
    Serial.println("--- RS485 mit 4800 Baud ---");
    node.begin(1, modbusSerial);    // ModbusMaster nutzt gepufferten Stream
    node.postTransmission(postTransmission);  // flush() nach jedem Request
    Serial.println("Modbus RS485 initialisiert.");
}


uint16_t readBatteryMV() {
    analogReference(AR_INTERNAL_3_0);
    analogReadResolution(12);
    float raw = analogRead(WB_A0);
    analogReference(AR_DEFAULT);
    analogReadResolution(10);
    // Spannungsteiler 1:2, empirisch kalibriert (Faktor 4200/4900)
    return (uint16_t)(raw * 2571.0 / 4096.0 * 2.0);
}

void sendSensorData(int16_t soil_temp, uint16_t soil_hum, uint16_t soil_ec) {
    m_lora_app_data.port = 2;

    // 1. Batterie (Bytes 0-1)
    uint16_t batMV = readBatteryMV();

    m_lora_app_data_buffer[0] = (uint8_t)(batMV >> 8);
    m_lora_app_data_buffer[1] = (uint8_t)(batMV & 0xFF);

    // 2. DS18B20 Temp (Bytes 2-3)
    // Falls du keinen DS18B20 hast, senden wir hier 0 oder einen Dummy
    int16_t tempDS = 0; 
    m_lora_app_data_buffer[2] = (uint8_t)(tempDS >> 8);
    m_lora_app_data_buffer[3] = (uint8_t)(tempDS & 0xFF);

    // 3. Bodenfeuchte (Bytes 4-5) - Formatter erwartet value/100
    // Wenn der Sensor 451 für 45.1% liefert, sende 4510
    m_lora_app_data_buffer[4] = (uint8_t)(soil_hum >> 8);
    m_lora_app_data_buffer[5] = (uint8_t)(soil_hum & 0xFF);

    // 4. Bodentemp (Bytes 6-7) - Formatter erwartet value/100
    m_lora_app_data_buffer[6] = (uint8_t)(soil_temp >> 8);
    m_lora_app_data_buffer[7] = (uint8_t)(soil_temp & 0xFF);

    // 5. Leitfähigkeit EC (Bytes 8-9) - Formatter erwartet value (1:1)
    m_lora_app_data_buffer[8] = (uint8_t)(soil_ec >> 8);
    m_lora_app_data_buffer[9] = (uint8_t)(soil_ec & 0xFF);

    m_lora_app_data.buffsize = 10;
    m_lora_app_data.buffer = m_lora_app_data_buffer;

    // Versand
    lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
    if (error == LMH_SUCCESS) {
        Serial.println(">> Echte Sensordaten übertragen.");
    } else {
        Serial.printf(">> Fehler beim Senden: %d\n", error);
    }
}


void feedWatchdog(); // Forward-Declaration

void readSoilSensor() {
    Serial.println("--- Starte Modbus-Abfrage (ID 1, 4800 Baud) ---");

    // Wir lesen 3 Register ab Adresse 0x0000:
    // Register 0: Feuchtigkeit (0.1 % Schritte)
    // Register 1: Temperatur (0.1 °C Schritte)
    // Register 2: Leitfähigkeit (1 us/cm)
    uint8_t result = node.readHoldingRegisters(0x0000, 3);
    feedWatchdog(); // nach Modbus-Read WDT füttern

    if (result == node.ku8MBSuccess) {
        // Rohdaten aus dem Buffer holen
        uint16_t rawHum  = node.getResponseBuffer(0); 
        int16_t  rawTemp = (int16_t)node.getResponseBuffer(1); 
        uint16_t rawEC   = node.getResponseBuffer(2);

        // Umrechnung in lesbare Werte (Dezimalstelle verschieben)
        float humidity = rawHum / 10.0;
        float temperature = rawTemp / 10.0;

        // Ausgabe im Serial Monitor
        Serial.printf(">> DATEN EMPFANGEN <<\n");
        Serial.printf("Batterie:      %.2f V\n", readBatteryMV() / 1000.0);
        Serial.printf("Feuchtigkeit: %.1f %%\n", humidity);
        Serial.printf("Temperatur:   %.1f °C\n", temperature);
        Serial.printf("Leitfähigkeit: %u us/cm\n", rawEC);
        Serial.println("-----------------------");

        // Hier werden die Daten an deine LoRa-Sendefunktion übergeben
        // Wir senden die Rohwerte (z.B. 225 statt 22.5), um Bytes zu sparen
        // Decoder erwartet ×100, Sensor liefert ×10 → ×10 multiplizieren
        sendSensorData(rawTemp * 10, rawHum * 10, rawEC);

    } else {
        // Fehlerdiagnose
        Serial.print("Modbus Fehler: ");
        if (result == 0xE2) Serial.println("Timeout (Sensor antwortet nicht)");
        else if (result == 0xE0) Serial.println("CRC Fehler (Signal gestört)");
        else Serial.printf("Code 0x%02X\n", result);
    }
}


void initWatchdog() {
    // Watchdog: Reset nach 120 Sekunden ohne "Füttern"
    // 120s gibt ausreichend Spielraum für LoRa TX + RX-Fenster + Modbus
    NRF_WDT->CONFIG         = 0x01; // Läuft auch im Sleep
    NRF_WDT->CRV            = 32768 * 120; // 120 Sekunden
    NRF_WDT->RREN           = 0x01; // Reload-Register 0 aktivieren
    NRF_WDT->TASKS_START    = 1;    // Watchdog starten
}

void feedWatchdog() {
    NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    initWatchdog();

    Serial.println("RAK4631 Start");
    uint32_t startWait = millis();
    while (!Serial && (millis() - startWait) < 5000);

    Serial.println("RAK4631 Booting...");

    // Sensor-Power einschalten (WB_IO2 = P1.02)
    pinMode(WB_IO2, OUTPUT);
    digitalWrite(WB_IO2, HIGH);
    delay(500); // kurz warten bis 3V3_S stabil

    // Sensor-Aufwachzeit: CWT-SOIL-THC-S braucht Zeit nach Power-On
    delay(1500);

    // Modbus initialisieren
    Serial.println("init Modbus");
    initModbus();

    Serial.println("init lora");
    lora_rak4630_init();

    // Register Fix for Antenna Switch
    uint8_t dio2_config = 0x01; 
    SX126xWriteRegisters(0x058B, &dio2_config, 1);

    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);

    lmh_param_t lora_param = {LORAWAN_ADR_OFF, DR_0, LORAWAN_PUBLIC_NETWORK, 8};
    
    // Using LORAMAC_REGION_EU868 for the V2.0 Library
    if (lmh_init(&lora_callbacks, lora_param, true, CLASS_A, LORAMAC_REGION_EU868) == 0) {
        Serial.println("Joining...");
        lmh_join();
    }
    //scanModbus();
}

void loop() {
    feedWatchdog(); // Watchdog füttern — verhindert Reset solange loop() läuft
    delay(10);

    static uint32_t lastSendTick = 0;
    
    // Überprüfen, ob das Intervall abgelaufen ist
    if (millis() - lastSendTick > sendInterval) {
        lastSendTick = millis();

        Serial.println("\n--- Starte Messzyklus ---");

        // 1. Nur senden wenn Join bestätigt
        if (!joined) {
            Serial.println("Noch nicht gejoint — warte...");
            return;
        }

        // 2. Sensor auslesen
        readSoilSensor();

        // 2. Sendintervall dynamisch nach Batteriespannung anpassen
        // Achtung: RAK fällt bei ~3.75V aus — Stufen mit ausreichend Puffer wählen
        uint16_t batMV = readBatteryMV();
        if (batMV < 3900) {
            sendInterval = 3600000; //3600000; // 60 min bei Bat < 3.9V (Puffer vor Ausfall bei ~3.75V)
        } else if (batMV < 4000) {
            sendInterval = 1800000; //1800000; // 30 min bei Bat < 4.0V
        } else {
            sendInterval = 1200000; //1200000; // 20 min normal (Bat >= 4.0V)
        }
        Serial.printf("Naechste Messung in %lu min (Bat: %u mV)\n", sendInterval / 60000, batMV);
    }
   
}