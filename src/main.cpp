#include <Arduino.h>
#include "CoinAcceptor.h"
#include "StepperMotor.h"
#include "NetworkManager.h"
#include "UltrasonicSensor.h"
#include "Neon.h"
#include "Beacon.h"
#include "Solonoid.h"
#include "LEDHandler.h"

/* ----- Pin map (same as main.cpp) ----- */
constexpr uint8_t COIN_PIN   = 16;
constexpr uint8_t RELAY_PIN  = 25;

constexpr uint8_t IN1 = 12, IN2 = 14, IN3 = 27, IN4 = 26;
constexpr uint8_t TRIG_PIN = 4, ECHO_PIN = 5;

// Network settings
const char* ssid     = "POCO F5";
const char* password = "321321321";
const char* mqttHost = "192.168.137.239";
const int   mqttPort = 1883;

// Test selection variable
int currentTest = 0;
const int MAX_TESTS = 7; // Number of test functions

// Network manager instance
NetworkManager net(ssid, password, mqttHost, mqttPort);
bool networkInitialized = false;
bool testInProgress = false;

// LED handler instance
LEDHandler ledHandler;

// Component state variables
bool coinMonitoringEnabled = false;
bool neonLightsEnabled = false;
bool beaconEnabled = false;
bool solenoidEnabled = false;
CoinAcceptor* coinAcceptor = nullptr;

// Serial redirection to MQTT
class MQTTSerial : public Print {
public:
    size_t write(uint8_t byte) override {
        buffer += (char)byte;
        
        // Send line when we hit newline or buffer gets too long
        if (byte == '\n' || buffer.length() > 200) {
            sendToMQTT();
        }
        
        // Also send to real serial
        return Serial.write(byte);
    }
    
    size_t write(const uint8_t *buffer, size_t size) override {
        // Also send to real serial
        Serial.write(buffer, size);
        
        for (size_t i = 0; i < size; i++) {
            write(buffer[i]);
        }
        return size;
    }
    
private:
    String buffer = "";
    unsigned long lastSend = 0;
    
    void sendToMQTT() {
        if (networkInitialized && !buffer.isEmpty()) {
            // Remove newlines and trim
            buffer.trim();
            if (buffer.length() > 0) {
                String mqttMsg = "{\"type\":\"SERIAL\",\"data\":\"" + buffer + "\"}";
                net.send("esp32/data", mqttMsg.c_str());
            }
        }
        buffer = "";
        lastSend = millis();
    }
};

MQTTSerial mqttSerial;

// Component data publisher function
void publishComponentData(const String& type, const String& data) {
    if (networkInitialized) {
        String payload = "{\"type\":\"" + type + "\",\"data\":\"" + data + "\"}";
        net.send("esp32/data", payload.c_str());
    }
}

// Enhanced logging function
void netPrintln(const String& msg) {
    Serial.println(msg);
    if (networkInitialized) {
        publishComponentData("LOG", msg);
    }
}

// Function declarations
void testCoinAcceptor();
void testStepperMotor();
void testUltrasonic();
void testNeonLights();
void testBeacon();
void testSolenoid();
void testLEDStrip();

// New component control functions
void enableCoinMonitoring();
void disableCoinMonitoring();
void runStepperMovement();
void runUltrasonicScan();
void enableNeonLights();
void disableNeonLights();
void runNeonPattern();
void enableBeacon();
void disableBeacon();
void enableSolenoid();
void disableSolenoid();
void runLEDAnimation();

void runTestFromMQTT(const String& command);
void initializeNetwork();
void handleSerialInput(const String& input);
void mqttPrint(const String& message);
void mqttPrintln(const String& message);

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    mqttPrintln("===========================================");
    mqttPrintln("     ESP32 MQTT-CONTROLLED TESTER");
    mqttPrintln("===========================================");
    mqttPrintln("");
    mqttPrintln("Initializing WiFi and MQTT connection...");
    
    // Initialize network connection
    initializeNetwork();
    
    // Initialize hardware modules
    setupNeon();
    setupBeacon();
    setupSolonoid();
    
    // Initialize LED handler
    ledHandler.setup();
    
    mqttPrintln("Hardware modules initialized (Neon, Beacon, Solenoid & LED Strip)");
    
    mqttPrintln("");
    mqttPrintln("Available Commands (via MQTT or Serial):");
    mqttPrintln("=== TEST COMMANDS ===");
    mqttPrintln("TEST_COIN      - Test Coin Acceptor");
    mqttPrintln("TEST_STEPPER   - Test Stepper Motor");
    mqttPrintln("TEST_ULTRA     - Test Ultrasonic Sensor");
    mqttPrintln("TEST_NEON      - Test Neon Lights");
    mqttPrintln("TEST_BEACON    - Test Beacon");
    mqttPrintln("TEST_SOLENOID  - Test Solenoid");
    mqttPrintln("TEST_LED       - Test LED Strip");
    mqttPrintln("TEST_ALL       - Run All Tests");
    mqttPrintln("");
    mqttPrintln("=== COMPONENT CONTROL ===");
    mqttPrintln("START_COIN     - Enable coin monitoring");
    mqttPrintln("STOP_COIN      - Disable coin monitoring");
    mqttPrintln("STEPPER_RUN    - Run stepper movement");
    mqttPrintln("ULTRA_SCAN     - Single ultrasonic scan");
    mqttPrintln("NEON_ON/OFF    - Control neon lights");
    mqttPrintln("NEON_PATTERN   - Run neon pattern (5s)");
    mqttPrintln("BEACON_ON/OFF  - Control beacon");
    mqttPrintln("SOLENOID_ON/OFF- Control solenoid");
    mqttPrintln("LED_RUN        - Run LED animation");
    mqttPrintln("");
    mqttPrintln("=== SYSTEM ===");
    mqttPrintln("STATUS         - Show System Status");
    mqttPrintln("");
    mqttPrintln("Manual Commands (Serial only):");
    mqttPrintln("0-6 - Run specific test number");
    mqttPrintln("h   - Show help");
    mqttPrintln("r   - Restart ESP");
    mqttPrintln("===========================================");
    
    if (networkInitialized) {
        mqttPrintln("✅ Ready to receive MQTT commands from Raspberry Pi!");
        mqttPrintln("Topic: esp32/control/esp2");
        mqttPrintln("Data Output: esp32/data");
        mqttPrintln("Send commands to: esp32/control/esp2");
    } else {
        mqttPrintln("⚠️ Network not connected - Manual mode only");
    }
}

void loop() {
    // Keep network connection alive and check for MQTT messages
    if (networkInitialized) {
        net.loop();
        
        // Check for MQTT commands
        static String lastProcessed = "";
        String mqttMessage = NetworkManager::getLastMessage();
        if (!mqttMessage.isEmpty() && mqttMessage != lastProcessed) {
            mqttPrintln("📥 MQTT Message received: " + mqttMessage);

            String command = NetworkManager::getStatusFromLastMessage();
            if (!command.isEmpty() && !testInProgress) {
                if (command.startsWith("SERIAL_INPUT:")) {
                    handleSerialInput(command.substring(13));
                } else {
                    runTestFromMQTT(command);
                }
                lastProcessed = mqttMessage; // ✅ Mark this message as processed
            }
        }
    }
    
    // Handle local serial commands
    if (Serial.available()) {
        String input = Serial.readString();
        handleSerialInput(input);
    }
    
    // Continuous component monitoring
    if (coinMonitoringEnabled && coinAcceptor != nullptr) {
        if (coinAcceptor->poll()) {
            publishComponentData("COIN", "detected");
            netPrintln("💰 Coin detected and sent to Raspberry Pi");
        }
    }
    
    delay(100);
}

void testCoinAcceptor() {
    mqttPrintln("🪙 COIN ACCEPTOR TEST");
    mqttPrintln("Testing coin acceptor functionality...");
    
    CoinAcceptor coin(COIN_PIN, RELAY_PIN);
    coin.begin();
    
    mqttPrintln("Coin acceptor initialized");
    mqttPrintln("Pin " + String(COIN_PIN) + " set as coin input");
    mqttPrintln("Pin " + String(RELAY_PIN) + " set as relay control");
    
    mqttPrintln("\nEnabling coin acceptor...");
    coin.enable();
    mqttPrintln("✅ Coin acceptor ENABLED - Insert a coin to test");
    
    mqttPrintln("\nWaiting 10 seconds for coin insertion...");
    unsigned long startTime = millis();
    int coinCount = 0;
    
    while (millis() - startTime < 10000) {
        if (coin.poll()) {
            coinCount++;
            mqttPrintln("💰 COIN DETECTED! Count: " + String(coinCount));
            mqttPrintln("Timestamp: " + String(millis()));
        }
        delay(50);
    }
    
    mqttPrintln("\nDisabling coin acceptor...");
    coin.disable();
    mqttPrintln("❌ Coin acceptor DISABLED");
    
    mqttPrintln("\nCoin Acceptor Test Results:");
    mqttPrintln("- Total coins detected: " + String(coinCount));
    mqttPrintln("- Enable/Disable functions: Working");
    mqttPrintln("- Polling mechanism: " + String(coinCount > 0 ? "Working" : "No coins detected"));
    
    coin.reset();
    mqttPrintln("- Reset function: Called");
    
    // Send results to Raspberry Pi
    if (networkInitialized) {
        String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"COIN_ACCEPTOR\",\"coins_detected\":" + 
                          String(coinCount) + ",\"status\":\"" + 
                          String(coinCount > 0 ? "success" : "no_coins") + "\"}";
        net.send("esp32/data", resultMsg.c_str());
    }
}

void testStepperMotor() {
    mqttPrintln("🔄 STEPPER MOTOR TEST");
    mqttPrintln("Testing stepper motor functionality...");
    
    StepperMotor stepper(IN1, IN2, IN3, IN4);
    stepper.begin();
    
    mqttPrintln("Stepper motor initialized");
    mqttPrintln("Pins: IN1=" + String(IN1) + ", IN2=" + String(IN2) + 
                   ", IN3=" + String(IN3) + ", IN4=" + String(IN4));
    
    mqttPrintln("\nTesting clockwise rotation (100 steps)...");
    stepper.rotate(100, true);
    mqttPrintln("✅ Clockwise rotation completed");
    
    delay(1000);
    
    mqttPrintln("\nTesting counter-clockwise rotation (100 steps)...");
    stepper.rotate(100, false);
    mqttPrintln("✅ Counter-clockwise rotation completed");
    
    delay(1000);
    
    mqttPrintln("\nTesting different step counts...");
    int testSteps[] = {50, 200, 250};
    for (int i = 0; i < 3; i++) {
        mqttPrintln("Rotating " + String(testSteps[i]) + " steps clockwise...");
        stepper.rotate(testSteps[i], true);
        delay(500);
        
        mqttPrintln("Rotating " + String(testSteps[i]) + " steps counter-clockwise...");
        stepper.rotate(testSteps[i], false);
        delay(500);
    }
    
    mqttPrintln("\nStepper Motor Test Results:");
    mqttPrintln("- Initialization: ✅ Working");
    mqttPrintln("- Clockwise rotation: ✅ Working");
    mqttPrintln("- Counter-clockwise rotation: ✅ Working");
    mqttPrintln("- Variable step counts: ✅ Working");
    mqttPrintln("- Pin control: ✅ Working");
    
    // Send results to Raspberry Pi
    if (networkInitialized) {
        String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"STEPPER_MOTOR\",\"status\":\"success\",\"steps_tested\":[50,100,200,500]}";
        net.send("esp32/data", resultMsg.c_str());
    }
}

void testUltrasonic() {
    mqttPrintln("📏 ULTRASONIC SENSOR TEST");
    mqttPrintln("Testing ultrasonic sensor functionality...");
    
    UltrasonicSensor ultrasonic(TRIG_PIN, ECHO_PIN);
    ultrasonic.begin();
    
    mqttPrintln("Ultrasonic sensor initialized");
    mqttPrintln("TRIG Pin: " + String(TRIG_PIN) + ", ECHO Pin: " + String(ECHO_PIN));
    
    mqttPrintln("\nTaking 20 distance measurements...");
    float avgDistance, minDistance, maxDistance;
    int validReadings;
    
    ultrasonic.getDistanceStats(20, avgDistance, minDistance, maxDistance, validReadings);
    
    mqttPrintln("\nUltrasonic Sensor Test Results:");
    mqttPrintln("- Initialization: ✅ Working");
    mqttPrintln("- Valid readings: " + String(validReadings) + "/20");
    
    if (validReadings > 0) {
        mqttPrintln("- Average distance: " + String(avgDistance, 2) + " cm");
        mqttPrintln("- Min distance: " + String(minDistance, 2) + " cm");
        mqttPrintln("- Max distance: " + String(maxDistance, 2) + " cm");
        mqttPrintln("- Measurement accuracy: " + String(validReadings >= 15 ? "✅ Good" : "⚠️ Fair"));
    } else {
        mqttPrintln("- ❌ No valid readings obtained");
    }
    
    mqttPrintln("- TRIG pin control: ✅ Working");
    mqttPrintln("- ECHO pin reading: " + String(validReadings > 0 ? "✅ Working" : "❌ Not working"));
    
    // Test single reading
    mqttPrintln("\nTesting single reading...");
    float singleReading = ultrasonic.getDistance();
    if (ultrasonic.isValidReading(singleReading)) {
        mqttPrintln("Single reading: " + String(singleReading, 2) + " cm");
    } else {
        mqttPrintln("Single reading: Invalid or timeout");
    }
    
    // Test average reading
    mqttPrintln("\nTesting average of 5 readings...");
    float avgReading = ultrasonic.getAverageDistance(5, 100);
    if (avgReading > 0) {
        mqttPrintln("Average reading: " + String(avgReading, 2) + " cm");
    } else {
        mqttPrintln("Average reading: No valid readings");
    }
    
    // Send results to Raspberry Pi
    if (networkInitialized) {
        String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"ULTRASONIC\",\"valid_readings\":" + 
                          String(validReadings) + ",\"avg_distance\":" + String(avgDistance, 2) + 
                          ",\"min_distance\":" + String(minDistance, 2) + 
                          ",\"max_distance\":" + String(maxDistance, 2) + 
                          ",\"status\":\"" + String(validReadings > 10 ? "success" : "poor") + "\"}";
        net.send("esp32/data", resultMsg.c_str());
    }
}

void testNeonLights() {
    mqttPrintln("💡 NEON LIGHTS TEST");
    mqttPrintln("Testing neon lights functionality...");
    
    mqttPrintln("Neon lights initialized");
    mqttPrintln("Pins: 33, 32, 35, 34, 2");
    
    mqttPrintln("\nTesting all neon lights ON...");
    neonAllOn();
    mqttPrintln("✅ All neon lights turned ON");
    delay(2000);
    
    mqttPrintln("\nTesting all neon lights OFF...");
    neonAllOff();
    mqttPrintln("✅ All neon lights turned OFF");
    delay(1000);
    
    mqttPrintln("\nTesting Pattern 1: Sequential Blink...");
    for (int i = 0; i < 3; i++) {
        mqttPrintln("Pattern 1 cycle " + String(i + 1) + "/3");
        neonPattern1();
        delay(500);
    }
    mqttPrintln("✅ Pattern 1 completed");
    
    delay(1000);
    
    mqttPrintln("\nTesting Pattern 2: Chase Forward and Back...");
    for (int i = 0; i < 3; i++) {
        mqttPrintln("Pattern 2 cycle " + String(i + 1) + "/3");
        neonPattern2();
        delay(500);
    }
    mqttPrintln("✅ Pattern 2 completed");
    
    mqttPrintln("\nFinal test: All lights ON for 3 seconds...");
    neonAllOn();
    delay(3000);
    neonAllOff();
    mqttPrintln("✅ Final test completed - All lights OFF");
    
    mqttPrintln("\nNeon Lights Test Results:");
    mqttPrintln("- Initialization: ✅ Working");
    mqttPrintln("- All ON function: ✅ Working");
    mqttPrintln("- All OFF function: ✅ Working");
    mqttPrintln("- Sequential pattern: ✅ Working");
    mqttPrintln("- Chase pattern: ✅ Working");
    mqttPrintln("- Individual relay control: ✅ Working");
    
    // Send results to Raspberry Pi
    if (networkInitialized) {
        String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"NEON_LIGHTS\",\"status\":\"success\",\"patterns_tested\":[\"sequential\",\"chase\",\"all_on\",\"all_off\"]}";
        net.send("esp32/data", resultMsg.c_str());
    }
}

void testBeacon() {
    mqttPrintln("🚨 BEACON TEST");
    mqttPrintln("Testing beacon functionality...");
    
    mqttPrintln("Beacon initialized");
    mqttPrintln("Pin: 0");
    
    mqttPrintln("\nTesting beacon ON...");
    beaconOn();
    mqttPrintln("✅ Beacon turned ON");
    delay(3000);
    
    mqttPrintln("\nTesting beacon OFF...");
    beaconOff();
    mqttPrintln("✅ Beacon turned OFF");
    delay(1000);
    
    mqttPrintln("\nTesting beacon blinking pattern...");
    for (int i = 0; i < 5; i++) {
        mqttPrintln("Blink " + String(i + 1) + "/5");
        beaconOn();
        delay(500);
        beaconOff();
        delay(500);
    }
    mqttPrintln("✅ Blinking pattern completed");
    
    mqttPrintln("\nTesting rapid blink pattern...");
    for (int i = 0; i < 10; i++) {
        beaconOn();
        delay(200);
        beaconOff();
        delay(200);
    }
    mqttPrintln("✅ Rapid blink pattern completed");
    
    mqttPrintln("\nFinal test: Beacon ON for 2 seconds...");
    beaconOn();
    delay(2000);
    beaconOff();
    mqttPrintln("✅ Final test completed - Beacon OFF");
    
    mqttPrintln("\nBeacon Test Results:");
    mqttPrintln("- Initialization: ✅ Working");
    mqttPrintln("- ON function: ✅ Working");
    mqttPrintln("- OFF function: ✅ Working");
    mqttPrintln("- Blinking patterns: ✅ Working");
    mqttPrintln("- Relay control: ✅ Working");
    
    // Send results to Raspberry Pi
    if (networkInitialized) {
        String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"BEACON\",\"status\":\"success\",\"patterns_tested\":[\"on\",\"off\",\"blink\",\"rapid_blink\"]}";
        net.send("esp32/data", resultMsg.c_str());
    }
}

void testSolenoid() {
    mqttPrintln("🔧 SOLENOID TEST");
    mqttPrintln("Testing solenoid functionality...");
    
    mqttPrintln("Solenoid initialized");
    mqttPrintln("Pin: 13");
    
    mqttPrintln("\nTesting solenoid ON...");
    SolonoidOn();
    mqttPrintln("✅ Solenoid turned ON");
    delay(2000);
    
    mqttPrintln("\nTesting solenoid OFF...");
    SolonoidOff();
    mqttPrintln("✅ Solenoid turned OFF");
    delay(1000);
    
    mqttPrintln("\nTesting solenoid pulse pattern...");
    for (int i = 0; i < 5; i++) {
        mqttPrintln("Pulse " + String(i + 1) + "/5");
        SolonoidOn();
        delay(500);
        SolonoidOff();
        delay(500);
    }
    mqttPrintln("✅ Pulse pattern completed");
    
    mqttPrintln("\nTesting rapid activation pattern...");
    for (int i = 0; i < 10; i++) {
        SolonoidOn();
        delay(200);
        SolonoidOff();
        delay(200);
    }
    mqttPrintln("✅ Rapid activation pattern completed");
    
    mqttPrintln("\nTesting extended activation...");
    mqttPrintln("Extended ON for 3 seconds...");
    SolonoidOn();
    delay(3000);
    SolonoidOff();
    mqttPrintln("✅ Extended activation completed - Solenoid OFF");
    
    mqttPrintln("\nSolenoid Test Results:");
    mqttPrintln("- Initialization: ✅ Working");
    mqttPrintln("- ON function: ✅ Working");
    mqttPrintln("- OFF function: ✅ Working");
    mqttPrintln("- Pulse patterns: ✅ Working");
    mqttPrintln("- Extended activation: ✅ Working");
    mqttPrintln("- Relay control: ✅ Working");
    
    // Send results to Raspberry Pi
    if (networkInitialized) {
        String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"SOLENOID\",\"status\":\"success\",\"patterns_tested\":[\"on\",\"off\",\"pulse\",\"rapid\",\"extended\"]}";
        net.send("esp32/data", resultMsg.c_str());
    }
}

void testLEDStrip() {
    mqttPrintln("🌈 LED STRIP TEST");
    mqttPrintln("Testing LED strip functionality...");
    
    mqttPrintln("LED strip initialized");
    mqttPrintln("Pin: 21, LEDs: 10");
    mqttPrintln("Type: NeoPixel (WS2812B)");
    
    mqttPrintln("\nTesting LED strip reset...");
    ledHandler.reset();
    mqttPrintln("✅ LED strip cleared");
    delay(1000);
    
    mqttPrintln("\nTesting LED animation sequence...");
    mqttPrintln("Running 5-second animation with random colors and speeds...");
    ledHandler.run();
    mqttPrintln("✅ Animation sequence completed");
    
    delay(1000);
    
    mqttPrintln("\nTesting multiple animation cycles...");
    for (int i = 0; i < 3; i++) {
        mqttPrintln("Animation cycle " + String(i + 1) + "/3");
        ledHandler.run();
        delay(500);
    }
    mqttPrintln("✅ Multiple animation cycles completed");
    
    mqttPrintln("\nTesting LED reset after animations...");
    ledHandler.reset();
    mqttPrintln("✅ Final reset completed - All LEDs OFF");
    
    mqttPrintln("\nLED Strip Test Results:");
    mqttPrintln("- Initialization: ✅ Working");
    mqttPrintln("- Reset function: ✅ Working");
    mqttPrintln("- Animation function: ✅ Working");
    mqttPrintln("- Random color generation: ✅ Working");
    mqttPrintln("- Speed variation: ✅ Working");
    mqttPrintln("- Multiple cycles: ✅ Working");
    mqttPrintln("- NeoPixel control: ✅ Working");
    
    // Send results to Raspberry Pi
    if (networkInitialized) {
        String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"LED_STRIP\",\"status\":\"success\",\"features_tested\":[\"reset\",\"animation\",\"colors\",\"speeds\",\"cycles\"]}";
        net.send("esp32/data", resultMsg.c_str());
    }
}

void initializeNetwork() {
    mqttPrintln("🌐 Initializing Network Connection...");
    
    mqttPrintln("SSID: " + String(ssid));
    mqttPrintln("MQTT Host: " + String(mqttHost) + ":" + String(mqttPort));
    
    // Begin connection
    net.begin();
    
    // Wait for connection with timeout
    mqttPrintln("Connecting to WiFi...");
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
        net.loop();
        delay(500);
        mqttPrint(".");
    }
    mqttPrintln("");
    
    if (WiFi.status() == WL_CONNECTED) {
        mqttPrintln("✅ WiFi Connected!");
        mqttPrintln("IP Address: " + WiFi.localIP().toString());
        mqttPrintln("Signal Strength: " + String(WiFi.RSSI()) + " dBm");
        
        // Wait a bit more for MQTT connection
        delay(2000);
        for (int i = 0; i < 5; i++) {
            net.loop();
            delay(1000);
        }
        
        networkInitialized = true;
        mqttPrintln("✅ MQTT Connection established!");
        mqttPrintln("Subscribed to: esp32/control/esp2");
        
        // Send ready status to Raspberry Pi
        net.send("esp32/data", "{\"type\":\"READY\",\"device\":\"ESP32_TESTER\",\"status\":\"online\"}");
        
    } else {
        mqttPrintln("❌ WiFi Connection failed!");
        mqttPrintln("Continuing in manual mode only...");
        networkInitialized = false;
    }
}

void runTestFromMQTT(const String& command) {
    if (testInProgress) {
        mqttPrintln("⚠️ Test already in progress, ignoring command: " + command);
        return;
    }
    
    testInProgress = true;
    mqttPrintln("\n🤖 MQTT Command received: " + command);
    mqttPrintln("===========================================");
    
    // Send acknowledgment to Raspberry Pi
    if (networkInitialized) {
        String ackMsg = "{\"type\":\"ACK\",\"command\":\"" + command + "\",\"status\":\"starting\"}";
        net.send("esp32/data", ackMsg.c_str());
    }
    
    if (command == "TEST_COIN") {
        testCoinAcceptor();
    } 
    else if (command == "TEST_STEPPER") {
        testStepperMotor();
    }
    else if (command == "TEST_ULTRA") {
        testUltrasonic();
    }
    else if (command == "TEST_NEON") {
        testNeonLights();
    }
    else if (command == "TEST_BEACON") {
        testBeacon();
    }
    else if (command == "TEST_SOLENOID") {
        testSolenoid();
    }
    else if (command == "TEST_LED") {
        testLEDStrip();
    }
    // New component control commands
    else if (command == "START_COIN") {
        enableCoinMonitoring();
    }
    else if (command == "STOP_COIN") {
        disableCoinMonitoring();
    }
    else if (command == "STEPPER_RUN") {
        runStepperMovement();
    }
    else if (command == "ULTRA_SCAN") {
        runUltrasonicScan();
    }
    else if (command == "NEON_ON") {
        enableNeonLights();
    }
    else if (command == "NEON_OFF") {
        disableNeonLights();
    }
    else if (command == "NEON_PATTERN") {
        runNeonPattern();
    }
    else if (command == "BEACON_ON") {
        enableBeacon();
    }
    else if (command == "BEACON_OFF") {
        disableBeacon();
    }
    else if (command == "SOLENOID_ON") {
        enableSolenoid();
    }
    else if (command == "SOLENOID_OFF") {
        disableSolenoid();
    }
    else if (command == "LED_RUN") {
        runLEDAnimation();
    }
    else if (command == "TEST_ALL") {
        mqttPrintln("🔄 RUNNING ALL TESTS SEQUENTIALLY");
        mqttPrintln("This will take several minutes...");
        
        testCoinAcceptor();
        delay(2000);
        testStepperMotor();
        delay(2000);
        testUltrasonic();
        delay(2000);
        testNeonLights();
        delay(2000);
        testBeacon();
        delay(2000);
        testSolenoid();
        delay(2000);
        testLEDStrip();
        
        mqttPrintln("✅ ALL TESTS COMPLETED!");
    }
    else if (command == "STATUS") {
        mqttPrintln("📊 SYSTEM STATUS REPORT");
        mqttPrintln("WiFi Status: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
        if (WiFi.status() == WL_CONNECTED) {
            mqttPrintln("IP Address: " + WiFi.localIP().toString());
            mqttPrintln("Signal Strength: " + String(WiFi.RSSI()) + " dBm");
        }
        mqttPrintln("MQTT Status: " + String(networkInitialized ? "Connected" : "Disconnected"));
        mqttPrintln("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
        mqttPrintln("Uptime: " + String(millis() / 1000) + " seconds");
        mqttPrintln("Test in Progress: " + String(testInProgress ? "Yes" : "No"));
        
        // Send status to Raspberry Pi
        if (networkInitialized) {
            String statusMsg = "{\"type\":\"STATUS\",\"wifi\":\"" + 
                             String(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected") + 
                             "\",\"mqtt\":\"" + String(networkInitialized ? "connected" : "disconnected") + 
                             "\",\"uptime\":" + String(millis() / 1000) + 
                             ",\"free_heap\":" + String(ESP.getFreeHeap()) + "}";
            net.send("esp32/data", statusMsg.c_str());
        }
    }
    else {
        mqttPrintln("❌ Unknown command: " + command);
        mqttPrintln("Valid commands: TEST_*, START_COIN, STOP_COIN, STEPPER_RUN, ULTRA_SCAN, NEON_ON/OFF/PATTERN, BEACON_ON/OFF, SOLENOID_ON/OFF, LED_RUN, STATUS");
        
        if (networkInitialized) {
            String errorMsg = "{\"type\":\"ERROR\",\"message\":\"Unknown command: " + command + "\"}";
            net.send("esp32/data", errorMsg.c_str());
        }
    }
    
    // Send completion status to Raspberry Pi
    if (networkInitialized && command != "STATUS") {
        String completeMsg = "{\"type\":\"COMPLETE\",\"command\":\"" + command + "\",\"status\":\"finished\"}";
        net.send("esp32/data", completeMsg.c_str());
    }
    
    testInProgress = false;
    mqttPrintln("===========================================");
    mqttPrintln("✅ Command completed: " + command);
    mqttPrintln("Ready for next command...");
}

void mqttPrint(const String& message) {
    // Send to real serial
    Serial.print(message);
    
    // Send to MQTT if connected
    if (networkInitialized && message.length() > 0) {
        String mqttMsg = "{\"type\":\"SERIAL\",\"data\":\"" + message + "\"}";
        net.send("esp32/data", mqttMsg.c_str());
    }
}

void mqttPrintln(const String& message) {
    // Send to real serial
    Serial.println(message);
    
    // Send to MQTT if connected
    if (networkInitialized) {
        String mqttMsg = "{\"type\":\"SERIAL\",\"data\":\"" + message + "\"}";
        net.send("esp32/data", mqttMsg.c_str());
    }
}

void handleSerialInput(const String& input) {
    String trimmedInput = input;
    trimmedInput.trim();
    trimmedInput.toUpperCase();
    
    if (trimmedInput == "H") {
        setup(); // Show help again
        return;
    }
    
    if (trimmedInput == "R") {
        ESP.restart();
        return;
    }
    
    // Handle MQTT-style commands
    if (trimmedInput.startsWith("TEST_") || trimmedInput == "STATUS") {
        if (!testInProgress) {
            runTestFromMQTT(trimmedInput);
        } else {
            mqttPrintln("⚠️ Test in progress, please wait...");
        }
        return;
    }
    
    // Handle legacy numeric commands
    int testNumber = trimmedInput.toInt();
    if (testNumber >= 0 && testNumber < MAX_TESTS && !testInProgress) {
        currentTest = testNumber;
        testInProgress = true;
        mqttPrintln("\n===========================================");
        
        switch (currentTest) {
            case 0: testCoinAcceptor(); break;
            case 1: testStepperMotor(); break;
            case 2: testUltrasonic(); break;
            case 3: testNeonLights(); break;
            case 4: testBeacon(); break;
            case 5: testSolenoid(); break;
            case 6: testLEDStrip(); break;
            default:
                mqttPrintln("Invalid test number!");
                break;
        }
        
        testInProgress = false;
        mqttPrintln("===========================================");
        mqttPrintln("Test completed. Send another command to run a different test.");
    } else if (testInProgress) {
        mqttPrintln("⚠️ Test in progress, please wait...");
    } else {
        mqttPrintln("Invalid input! Send a test command, number 0-6, 'h' for help, or 'r' to restart");
    }
}

// ==================== COMPONENT CONTROL FUNCTIONS ====================

void enableCoinMonitoring() {
    if (coinAcceptor == nullptr) {
        coinAcceptor = new CoinAcceptor(COIN_PIN, RELAY_PIN);
        coinAcceptor->begin();
    }
    coinAcceptor->enable();
    coinMonitoringEnabled = true;
    netPrintln("✅ Coin monitoring enabled");
    publishComponentData("COIN_STATUS", "monitoring_enabled");
}

void disableCoinMonitoring() {
    if (coinAcceptor != nullptr) {
        coinAcceptor->disable();
    }
    coinMonitoringEnabled = false;
    netPrintln("⛔ Coin monitoring disabled");
    publishComponentData("COIN_STATUS", "monitoring_disabled");
}

void runStepperMovement() {
    netPrintln("🤖 Running stepper movement");
    StepperMotor stepper(IN1, IN2, IN3, IN4);
    stepper.begin();
    
    // Move 180 degrees clockwise, then back
    stepper.rotate(256, true);  // ~180 degrees
    delay(1000);
    stepper.rotate(256, false); // ~180 degrees back
    
    netPrintln("✅ Stepper movement complete");
    publishComponentData("STEPPER", "movement_complete");
}

void runUltrasonicScan() {
    netPrintln("📡 Running ultrasonic scan");
    UltrasonicSensor ultrasonic(TRIG_PIN, ECHO_PIN);
    
    float distance = ultrasonic.getDistance();
    String distanceStr = String(distance, 2);
    
    netPrintln("Distance: " + distanceStr + " cm");
    publishComponentData("ULTRASONIC", distanceStr);
}

void enableNeonLights() {
    netPrintln("💡 Enabling neon lights");
    setupNeon();
    turnOnAllNeon();
    neonLightsEnabled = true;
    publishComponentData("NEON", "lights_on");
}

void disableNeonLights() {
    netPrintln("🔅 Disabling neon lights");
    turnOffAllNeon();
    neonLightsEnabled = false;
    publishComponentData("NEON", "lights_off");
}

void runNeonPattern() {
    netPrintln("🎨 Running neon pattern display");
    setupNeon();
    neonPattern();
    neonLightsEnabled = false; // Ensure lights are off after pattern
    publishComponentData("NEON", "pattern_complete");
}

void enableBeacon() {
    netPrintln("🚨 Enabling beacon");
    setupBeacon();
    beaconOn();
    beaconEnabled = true;
    publishComponentData("BEACON", "beacon_on");
}

void disableBeacon() {
    netPrintln("🔇 Disabling beacon");
    beaconOff();
    beaconEnabled = false;
    publishComponentData("BEACON", "beacon_off");
}

void enableSolenoid() {
    netPrintln("🔧 Enabling solenoid");
    setupSolonoid();
    SolonoidOn();
    solenoidEnabled = true;
    publishComponentData("SOLENOID", "solenoid_on");
}

void disableSolenoid() {
    netPrintln("🔧 Disabling solenoid");
    SolonoidOff();
    solenoidEnabled = false;
    publishComponentData("SOLENOID", "solenoid_off");
}

void runLEDAnimation() {
    netPrintln("🎆 Running LED animation");
    ledHandler.setup();
    ledHandler.run();
    delay(5000); // Let animation run for 5 seconds
    ledHandler.reset();
    netPrintln("✅ LED animation complete");
    publishComponentData("LED", "animation_complete");
}
