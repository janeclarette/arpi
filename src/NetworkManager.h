#pragma once
#include <WiFi.h>
#include <PubSubClient.h>

class NetworkManager {
public:
    NetworkManager(const char* ssid, const char* password,
                   const char* mqttServer, int mqttPort);

    void begin();                                // Initialize Wi-Fi and MQTT
    void loop();                                 // Call this in main loop
    void send(const char* topic, const char* payload); // Send message

    static String getLastMessage();              // Raw received payload
    static String getStatusFromLastMessage();    // Parse and return "status"

private:
    const char* _ssid;
    const char* _password;
    const char* _mqttServer;
    int         _mqttPort;

    WiFiClient      wifiClient;
    PubSubClient    mqttClient;

    static String   _lastMessage;

    void connectWiFi();
    void connectMQTT();

    static void callback(char* topic, byte* payload, unsigned int length);
    static void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
    static const char* reasonToText(uint8_t reason);
};
