#include "NetworkManager.h"
#include "esp_wifi.h"
#include <ArduinoJson.h>

String NetworkManager::_lastMessage = "";

NetworkManager::NetworkManager(const char* ssid, const char* password,
                               const char* mqttServer, int mqttPort)
    : _ssid(ssid), _password(password), _mqttServer(mqttServer),
      _mqttPort(mqttPort), mqttClient(wifiClient) {}

void NetworkManager::begin() {
    Serial.println("🌐 Initializing network...");
    WiFi.onEvent(NetworkManager::onWiFiEvent);
    connectWiFi();

    mqttClient.setServer(_mqttServer, _mqttPort);
    mqttClient.setCallback(NetworkManager::callback);
    mqttClient.setBufferSize(256);  // Optional: For large JSON payloads

    connectMQTT();
}

void NetworkManager::loop() {
    if (!mqttClient.connected()) {
        connectMQTT();
    }
    mqttClient.loop();
}

void NetworkManager::send(const char* topic, const char* payload) {
    mqttClient.publish(topic, payload);
    Serial.printf("📤 Sent to [%s]: %s\n", topic, payload);
}

void NetworkManager::connectWiFi() {
    Serial.printf("🔐 Connecting to %s…\n", _ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid, _password);

    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; ++i) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("✅ Wi‑Fi connected. IP: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("❌ Wi‑Fi failed. Rebooting in 5s...");
        delay(5000);
        ESP.restart();
    }
}

void NetworkManager::connectMQTT() {
    while (!mqttClient.connected()) {
        Serial.print("🔁 Connecting to MQTT... ");
        if (mqttClient.connect("ESP32Client")) {  // Make sure this is unique
            Serial.println("connected!");
            if (mqttClient.subscribe("esp32/control/esp2")) {
                Serial.println("✅ Subscribed to esp32/control/esp2");
            } else {
                Serial.println("❌ Subscription failed");
            }
        } else {
            Serial.printf("failed, rc=%d. Retrying...\n", mqttClient.state());
            delay(2000);
        }
    }
}

void NetworkManager::callback(char* topic, byte* payload, unsigned int length) {
    _lastMessage = "";
    for (unsigned int i = 0; i < length; ++i) {
        _lastMessage += (char)payload[i];
    }

    Serial.print("📨 MQTT message [");
    Serial.print(topic);
    Serial.print("]: ");
    Serial.println(_lastMessage);
}

String NetworkManager::getLastMessage() {
    return _lastMessage;
}

String NetworkManager::getStatusFromLastMessage() {

    ArduinoJson::StaticJsonDocument<256> doc;

    // Parse the JSON string from the last received message
    ArduinoJson::DeserializationError error = ArduinoJson::deserializeJson(doc, _lastMessage);

    // Handle parsing error
    if (error) {
        Serial.print("⚠️ JSON parse failed: ");
        Serial.println(error.f_str());  // Proper way to print DeserializationError
        return "";
    }

    // Access the "status" key safely
    const char* status = doc["status"];
    return String(status ? status : "");
}

void NetworkManager::onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
        Serial.printf("❌ Wi‑Fi dropped. Reason %d → %s\n",
                      info.wifi_sta_disconnected.reason,
                      reasonToText(info.wifi_sta_disconnected.reason));
    }
}

const char* NetworkManager::reasonToText(uint8_t r) {
    switch (r) {
        case WIFI_REASON_NO_AP_FOUND: return "No AP found (201)";
        case WIFI_REASON_AUTH_FAIL: return "Auth fail (202)";
        case WIFI_REASON_BEACON_TIMEOUT: return "Beacon timeout (200)";
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "Handshake timeout (15)";
        default: return "Other / unknown reason";
    }
}
