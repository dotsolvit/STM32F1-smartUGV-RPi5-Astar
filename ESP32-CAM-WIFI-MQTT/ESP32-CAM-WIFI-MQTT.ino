//ESP32-CAM-WIFI-MQTT V2
//Board: AI Thinker ESP32-CAM
//Tools -> Flash Frequency: 80MHz
//Tools -> Partition Scheme: Huge APP (3MB No OTA/1MB SPIFFS)
//Lib: PubSubClient by Nick O'Leary 2.8
//Lib: ArduinoJson by Benoit Blanchon 7.4.3

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
// Підключення секретних налаштувань Wi-Fi
#include "myssid.h" 

#define MAX_SCAN_POINTS 35 // Із розрахунку сканування з кроком 6 градусів від 0 до 180град = 31
#define JSON_BUF_SIZE   1024 // 512 байт для 31 точки може бути впритул, 1024 — ідеально

const int flashPin = 4; // Пін led спалаху на ESP32-CAM
// Налаштування параметрів ШІМ для led спалаху
const int freq = 10000;       // Частота 10 кГц
const int resolution = 8;    // 8-бітна роздільна здатність (0-255)

// WiFi settings 
const char* ssid = MY_WIFI_SSID;
const char* password = MY_WIFI_PASS;
// Налаштування MQTT
const char* mqtt_server = "192.168.1.4"; // IP ADDRESS RPI5
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// --- СТРУКТУРИ ДЛЯ UART ОБМІНУ З STM32 ---
// Використовуємо #pragma pack(push, 1), щоб структури в пам'яті були без пустих байт-вирівнювань
#pragma pack(push, 1)

// Пакет 1: Команди керування (robot/control -> STM32)
struct ControlPacket {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xB1; // Маркер типу пакета (Control)
    uint8_t action;         // 1 - start, 2 - stop, 3 - calibrate
    uint8_t crc8;
};

// Пакет 2: Точка маршруту (robot/path -> STM32). Передаємо по одній точці за раз
struct PathPointPacket {
    uint8_t header1 = 0xAA;
    uint8_t header2 = 0xB2; // Маркер типу пакета (Path Point)
    float x;                // Фізична координата X (метри)
    float y;                // Фізична координата Y (метри)
    uint8_t is_last;        // 1 якщо це остання точка масиву, 0 якщо ні
    uint8_t crc8;
};

// Пакет 3: Телеметрія від STM32 (Для відправки в robot/telemetry)
struct UartTelemetryPacket {
    uint8_t header1;       // Має бути 0xDE
    uint8_t header2;       // Має бути 0x01
    float x;
    float y;
    float theta_deg;
    uint8_t battery_pct;
    uint8_t status_code;   // 0-stopped, 1-moving, 2-scanning, 3-calibrating, 4-arrived
    uint8_t crc8;
};

// Пакет 4: Сканування перешкод від STM32 (Для відправки в robot/obstacle_scan)
struct UartScanPacket {
    uint8_t header1;       // Має бути 0xDE
    uint8_t header2;       // Має бути 0x02
    int16_t angle;         // Поточний кут голови (-90...90)
    uint16_t distance_mm;  // Відстань в мм
    uint8_t is_end_of_scan;// 1 якщо це останній вимір панорами
    uint8_t crc8;
};
#pragma pack(pop)

// --- ФУНКЦІЯ ОБЧИСЛЕННЯ CRC8 (Dallas/Maxim) ---
uint8_t getCRC8(uint8_t *buffer, uint8_t size) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < size; i++) {
        uint8_t data = buffer[i];
        for (uint8_t j = 8; j > 0; j--) {
            uint8_t mix = (crc ^ data) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C;
            data >>= 1;
        }
    }
    return crc;
}

// --- ОБРОБКА ДАНИХ З СЕРВЕРА RPi5 (MQTT -> ESP32 -> UART STM32) ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String strTopic = String(topic);
    
    // 1. Обробка команд robot/control
    if (strTopic == "robot/control") {
        StaticJsonDocument<128> doc;
        if (deserializeJson(doc, payload, length) == DeserializationError::Ok) {
            String cmd = doc["command"] | "stop";
            ControlPacket p;
            if (cmd == "start") p.action = 1;
            else if (cmd == "stop") p.action = 2;
            else if (cmd == "calibrate") p.action = 3;
            else p.action = 2; // Безпека: за замовчуванням стоп
            
            p.crc8 = getCRC8((uint8_t*)&p.action, 1);
            Serial.write((uint8_t*)&p, sizeof(p)); // Відправляємо на STM32
        }
    }
    
    // 2. Обробка маршруту robot/path
    else if (strTopic == "robot/path") {
        // Динамічний документ, бо масив точок може бути великим
        DynamicJsonDocument doc(2048);
        if (deserializeJson(doc, payload, length) == DeserializationError::Ok) {
            JsonArray points = doc["points"];
            size_t total_points = points.size();
            
            for (size_t i = 0; i < total_points; i++) {
                PathPointPacket p;
                p.x = points[i]["x"] | 0.0f;
                p.y = points[i]["y"] | 0.0f;
                p.is_last = (i == total_points - 1) ? 1 : 0;
                
                // Рахуємо CRC8 для даних пакета (x, y, is_last займають 9 байт)
                p.crc8 = getCRC8((uint8_t*)&p.x, 9);
                
                Serial.write((uint8_t*)&p, sizeof(p));
                delay(5); // Невелика затримка, щоб STM32 встигав приймати потік точок
            }
        }
    }
}

// --- ПРИЙОМ ДАНИХ ВІД STM32 (UART -> ESP32 -> MQTT RPi5) ---
void handleUartRx() {
    static uint8_t rx_buf[64];
    static uint8_t rx_idx = 0;
    
    while (Serial.available() > 0) {
        uint8_t b = Serial.read();
        
        // Шукаємо стартовий маркер пакетів від STM32 (0xDE)
        if (rx_idx == 0 && b != 0xDE) continue;
        
        rx_buf[rx_idx++] = b;
        
        // Визначаємо тип пакета за другим байтом
        if (rx_idx >= 2) {
            uint8_t type = rx_buf[1];
            
            // Тип 1: Пакет Телеметрії (Розмір структури UartTelemetryPacket = 17 байт)
            if (type == 0x01 && rx_idx >= 17) {
                if (getCRC8(&rx_buf[2], 14) == rx_buf[16]) { // CRC від даних
                    UartTelemetryPacket* p = (UartTelemetryPacket*)rx_buf;
                    
                    const char* status_strings[] = {"stopped", "moving", "scanning", "calibrating", "arrived"};
                    const char* current_status = (p->status_code < 5) ? status_strings[p->status_code] : "stopped";
                    
                    StaticJsonDocument<256> jsonDoc;
                    jsonDoc["x"] = p->x;
                    jsonDoc["y"] = p->y;
                    jsonDoc["theta_deg"] = p->theta_deg;
                    jsonDoc["battery_pct"] = p->battery_pct;
                    jsonDoc["status"] = current_status;
                    
                    char out_buf[256];
                    serializeJson(jsonDoc, out_buf);
                    client.publish("robot/telemetry", out_buf);
                }
                rx_idx = 0;
            }
            // Тип 2: Пакет Обходу Перешкод (Буферуємо та надсилаємо панораму за один раз)
            else if (type == 0x02 && rx_idx >= 8) {
                if (getCRC8(&rx_buf[2], 5) == rx_buf[7]) {
                    UartScanPacket* p = (UartScanPacket*)rx_buf;
                    
                    // Статичні масиви для збору повної панорами
                    static int16_t scan_angles[MAX_SCAN_POINTS];
                    static int32_t scan_distances[MAX_SCAN_POINTS];
                    static uint8_t scan_count = 0;

                    if (scan_count < MAX_SCAN_POINTS) {
                        scan_angles[scan_count] = p->angle;
                        scan_distances[scan_count] = p->distance_mm;
                        scan_count++;
                    }

                    // Якщо STM32 сигналізує, що панорама завершена (або буфер заповнений)
                    if (p->is_end_of_scan == 1 || scan_count >= MAX_SCAN_POINTS) {
                        // Використовуємо чітко визначений розмір для ArduinoJson
                        StaticJsonDocument<JSON_BUF_SIZE> scanDoc;
                        JsonArray angles_arr = scanDoc.createNestedArray("angles");
                        JsonArray dist_arr = scanDoc.createNestedArray("distances");

                        for (uint8_t i = 0; i < scan_count; i++) {
                            angles_arr.add(scan_angles[i]);
                            dist_arr.add(scan_distances[i]);
                        }

                        char out_buf[JSON_BUF_SIZE]; // Задаємо масив фіксованого розміру
                        serializeJson(scanDoc, out_buf, sizeof(out_buf));
                        client.publish("robot/obstacle_scan", out_buf);

                        scan_count = 0; // Скидаємо накопичувач для наступного сканування
                    }

                }
                rx_idx = 0;
            }
            // Захист від переповнення буфера при смітті в ефірі
            else if (rx_idx >= 64) {
                rx_idx = 0;
            }
        }
    }
}

void setup() {
    // UART0 ініціалізація (Швидкість зв'язку з STM32)
    Serial.begin(115200);

    // Налаштування ШІМ для led спалаху: пін, частота та бітність.
    ledcAttach(flashPin, freq, resolution);
    // Стан для блимання (true - горить, false - вимкнено)
    bool ledState = false;

    // Wi-Fi Менеджер підключення з індикацією підключення
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        // Змінюємо стан на протилежний при кожній ітерації
        ledState = !ledState;
        // Якщо true — вмикаємо легке поблискування (яскравість 3), якщо false — вимикаємо
        ledcWrite(flashPin, ledState ? 3 : 0);
        delay(500);
    }
    // Результат успішного підключення: постійно горить на мінімумі (яскравість 1)
    ledcWrite(flashPin, 1);
    
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqttCallback);
}

void reconnectMQTT() {
    static uint32_t last_reconnect = 0;
    if (!client.connected() && (millis() - last_reconnect > 5000)) {
        last_reconnect = millis();
        if (client.connect("SmartUGV_ESP32_Bridge")) {
            client.subscribe("robot/control");
            client.subscribe("robot/path");
        }
    }
}

void loop() {
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();
    handleUartRx();
}
