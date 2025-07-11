// #include <Arduino.h>
// #include "CoinAcceptor.h"
// #include "StepperMotor.h"
// #include "RFIDReader.h"
// #include "NetworkManager.h"

// /* ----- Pin map (same as main.cpp) ----- */
// constexpr uint8_t COIN_PIN   = 16;
// constexpr uint8_t RELAY_PIN  = 15;

// constexpr uint8_t IN1 = 12, IN2 = 14, IN3 = 27, IN4 = 26;
// constexpr uint8_t TRIG_PIN = 4, ECHO_PIN = 5;

// constexpr uint8_t SS_PIN = 21;
// constexpr uint8_t RST_PIN = 22;

// // Network settings
// const char* ssid     = "Ecommerce";
// const char* password = "Admin@123";
// const char* mqttHost = "192.168.97.35";
// const int   mqttPort = 1883;

// // Test selection variable
// int currentTest = 0;
// const int MAX_TESTS = 4; // Number of test functions

// // Network manager instance
// NetworkManager net(ssid, password, mqttHost, mqttPort);
// bool networkInitialized = false;
// bool testInProgress = false;

// // Serial redirection to MQTT
// class MQTTSerial : public Print {
// public:
//     size_t write(uint8_t byte) override {
//         buffer += (char)byte;
        
//         // Send line when we hit newline or buffer gets too long
//         if (byte == '\n' || buffer.length() > 200) {
//             sendToMQTT();
//         }
        
//         // Also send to real serial
//         return Serial.write(byte);
//     }
    
//     size_t write(const uint8_t *buffer, size_t size) override {
//         // Also send to real serial
//         Serial.write(buffer, size);
        
//         for (size_t i = 0; i < size; i++) {
//             write(buffer[i]);
//         }
//         return size;
//     }
    
// private:
//     String buffer = "";
//     unsigned long lastSend = 0;
    
//     void sendToMQTT() {
//         if (networkInitialized && !buffer.isEmpty()) {
//             // Remove newlines and trim
//             buffer.trim();
//             if (buffer.length() > 0) {
//                 String mqttMsg = "{\"type\":\"SERIAL\",\"data\":\"" + buffer + "\"}";
//                 net.send("esp32/test/serial", mqttMsg.c_str());
//             }
//         }
//         buffer = "";
//         lastSend = millis();
//     }
// };

// MQTTSerial mqttSerial;

// // Function declarations
// void testCoinAcceptor();
// void testStepperMotor();
// void testRFIDReader();
// void testUltrasonic();
// void runTestFromMQTT(const String& command);
// void initializeNetwork();
// void handleSerialInput(const String& input);
// void mqttPrint(const String& message);
// void mqttPrintln(const String& message);

// void setup() {
//     Serial.begin(115200);
//     delay(1000);
    
//     mqttPrintln("===========================================");
//     mqttPrintln("     ESP32 MQTT-CONTROLLED TESTER");
//     mqttPrintln("===========================================");
//     mqttPrintln("");
//     mqttPrintln("Initializing WiFi and MQTT connection...");
    
//     // Initialize network connection
//     initializeNetwork();
    
//     mqttPrintln("");
//     mqttPrintln("Available Commands (via MQTT or Serial):");
//     mqttPrintln("TEST_COIN    - Test Coin Acceptor");
//     mqttPrintln("TEST_STEPPER - Test Stepper Motor");
//     mqttPrintln("TEST_RFID    - Test RFID Reader");
//     mqttPrintln("TEST_ULTRA   - Test Ultrasonic Sensor");
//     mqttPrintln("TEST_ALL     - Run All Tests");
//     mqttPrintln("STATUS       - Show System Status");
//     mqttPrintln("");
//     mqttPrintln("Manual Commands (Serial only):");
//     mqttPrintln("0-3 - Run specific test number");
//     mqttPrintln("h   - Show help");
//     mqttPrintln("r   - Restart ESP");
//     mqttPrintln("===========================================");
    
//     if (networkInitialized) {
//         mqttPrintln("✅ Ready to receive MQTT commands from Raspberry Pi!");
//         mqttPrintln("Topic: esp32/test/commands");
//         mqttPrintln("Serial Monitor: esp32/test/serial");
//         mqttPrintln("Send commands to: esp32/test/input");
//     } else {
//         mqttPrintln("⚠️ Network not connected - Manual mode only");
//     }
// }

// void loop() {
//     // Keep network connection alive and check for MQTT messages
//     if (networkInitialized) {
//         net.loop();
        
//         // Check for MQTT commands
//         String mqttMessage = NetworkManager::getLastMessage();
//         if (!mqttMessage.isEmpty()) {
//             mqttPrintln("📥 MQTT Message received: " + mqttMessage);
            
//             // Parse command from MQTT message
//             String command = NetworkManager::getStatusFromLastMessage();
//             if (!command.isEmpty() && !testInProgress) {
//                 // Check if it's a direct command or serial input
//                 if (command.startsWith("SERIAL_INPUT:")) {
//                     // Handle serial input from MQTT
//                     String serialInput = command.substring(13); // Remove "SERIAL_INPUT:" prefix
//                     mqttPrintln("🖥️ Remote Serial Input: " + serialInput);
//                     handleSerialInput(serialInput);
//                 } else {
//                     // Handle direct MQTT command
//                     runTestFromMQTT(command);
//                 }
//             }
//         }
//     }
    
//     // Handle local serial commands
//     if (Serial.available()) {
//         String input = Serial.readString();
//         handleSerialInput(input);
//     }
    
//     delay(100);
// }

// void testCoinAcceptor() {
//     mqttPrintln("🪙 COIN ACCEPTOR TEST");
//     mqttPrintln("Testing coin acceptor functionality...");
    
//     CoinAcceptor coin(COIN_PIN, RELAY_PIN);
//     coin.begin();
    
//     mqttPrintln("Coin acceptor initialized");
//     mqttPrintln("Pin " + String(COIN_PIN) + " set as coin input");
//     mqttPrintln("Pin " + String(RELAY_PIN) + " set as relay control");
    
//     mqttPrintln("\nEnabling coin acceptor...");
//     coin.enable();
//     mqttPrintln("✅ Coin acceptor ENABLED - Insert a coin to test");
    
//     mqttPrintln("\nWaiting 10 seconds for coin insertion...");
//     unsigned long startTime = millis();
//     int coinCount = 0;
    
//     while (millis() - startTime < 10000) {
//         if (coin.poll()) {
//             coinCount++;
//             mqttPrintln("💰 COIN DETECTED! Count: " + String(coinCount));
//             mqttPrintln("Timestamp: " + String(millis()));
//         }
//         delay(50);
//     }
    
//     mqttPrintln("\nDisabling coin acceptor...");
//     coin.disable();
//     mqttPrintln("❌ Coin acceptor DISABLED");
    
//     mqttPrintln("\nCoin Acceptor Test Results:");
//     mqttPrintln("- Total coins detected: " + String(coinCount));
//     mqttPrintln("- Enable/Disable functions: Working");
//     mqttPrintln("- Polling mechanism: " + String(coinCount > 0 ? "Working" : "No coins detected"));
    
//     coin.reset();
//     mqttPrintln("- Reset function: Called");
    
//     // Send results to Raspberry Pi
//     if (networkInitialized) {
//         String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"COIN_ACCEPTOR\",\"coins_detected\":" + 
//                           String(coinCount) + ",\"status\":\"" + 
//                           String(coinCount > 0 ? "success" : "no_coins") + "\"}";
//         net.send("esp32/test/results", resultMsg.c_str());
//     }
// }

// void testStepperMotor() {
//     mqttPrintln("🔄 STEPPER MOTOR TEST");
//     mqttPrintln("Testing stepper motor functionality...");
    
//     StepperMotor stepper(IN1, IN2, IN3, IN4);
//     stepper.begin();
    
//     mqttPrintln("Stepper motor initialized");
//     mqttPrintln("Pins: IN1=" + String(IN1) + ", IN2=" + String(IN2) + 
//                    ", IN3=" + String(IN3) + ", IN4=" + String(IN4));
    
//     mqttPrintln("\nTesting clockwise rotation (100 steps)...");
//     stepper.rotate(100, true);
//     mqttPrintln("✅ Clockwise rotation completed");
    
//     delay(1000);
    
//     mqttPrintln("\nTesting counter-clockwise rotation (100 steps)...");
//     stepper.rotate(100, false);
//     mqttPrintln("✅ Counter-clockwise rotation completed");
    
//     delay(1000);
    
//     mqttPrintln("\nTesting different step counts...");
//     int testSteps[] = {50, 200, 500};
//     for (int i = 0; i < 3; i++) {
//         mqttPrintln("Rotating " + String(testSteps[i]) + " steps clockwise...");
//         stepper.rotate(testSteps[i], true);
//         delay(500);
        
//         mqttPrintln("Rotating " + String(testSteps[i]) + " steps counter-clockwise...");
//         stepper.rotate(testSteps[i], false);
//         delay(500);
//     }
    
//     mqttPrintln("\nStepper Motor Test Results:");
//     mqttPrintln("- Initialization: ✅ Working");
//     mqttPrintln("- Clockwise rotation: ✅ Working");
//     mqttPrintln("- Counter-clockwise rotation: ✅ Working");
//     mqttPrintln("- Variable step counts: ✅ Working");
//     mqttPrintln("- Pin control: ✅ Working");
    
//     // Send results to Raspberry Pi
//     if (networkInitialized) {
//         String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"STEPPER_MOTOR\",\"status\":\"success\",\"steps_tested\":[50,100,200,500]}";
//         net.send("esp32/test/results", resultMsg.c_str());
//     }
// }

// void testRFIDReader() {
//     mqttPrintln("📇 RFID READER TEST");
//     mqttPrintln("Testing RFID reader functionality...");
    
//     RFIDReader rfid(SS_PIN, RST_PIN);
//     rfid.begin();
    
//     mqttPrintln("RFID reader initialized");
//     mqttPrintln("SS Pin: " + String(SS_PIN) + ", RST Pin: " + String(RST_PIN));
    
//     mqttPrintln("\nPlace an RFID card/tag near the reader...");
//     mqttPrintln("Waiting 15 seconds for card detection...");
    
//     unsigned long startTime = millis();
//     int cardCount = 0;
//     String lastUID = "";
    
//     while (millis() - startTime < 15000) {
//         if (rfid.poll()) {
//             cardCount++;
//             String currentUID = rfid.uid();
//             mqttPrintln("📋 CARD DETECTED #" + String(cardCount));
//             mqttPrintln("   UID: " + currentUID);
//             mqttPrintln("   Length: " + String(currentUID.length()) + " characters");
//             mqttPrintln("   Timestamp: " + String(millis()));
            
//             if (lastUID != currentUID) {
//                 mqttPrintln("   🆕 New card detected!");
//                 lastUID = currentUID;
//             } else {
//                 mqttPrintln("   🔄 Same card re-detected");
//             }
            
//             delay(1000); // Prevent rapid re-detection
//         }
//         delay(100);
//     }
    
//     mqttPrintln("\nRFID Reader Test Results:");
//     mqttPrintln("- Initialization: ✅ Working");
//     mqttPrintln("- Card detection: " + String(cardCount > 0 ? "✅ Working" : "❌ No cards detected"));
//     mqttPrintln("- UID reading: " + String(!lastUID.isEmpty() ? "✅ Working" : "❌ No UID read"));
//     mqttPrintln("- Total cards detected: " + String(cardCount));
//     mqttPrintln("- Last UID: " + (lastUID.isEmpty() ? "None" : lastUID));
    
//     // Send results to Raspberry Pi
//     if (networkInitialized) {
//         String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"RFID_READER\",\"cards_detected\":" + 
//                           String(cardCount) + ",\"last_uid\":\"" + lastUID + "\",\"status\":\"" + 
//                           String(cardCount > 0 ? "success" : "no_cards") + "\"}";
//         net.send("esp32/test/results", resultMsg.c_str());
//     }
// }

// void testUltrasonic() {
//     mqttPrintln("📏 ULTRASONIC SENSOR TEST");
//     mqttPrintln("Testing ultrasonic sensor functionality...");
    
//     pinMode(TRIG_PIN, OUTPUT);
//     pinMode(ECHO_PIN, INPUT);
    
//     mqttPrintln("Ultrasonic sensor initialized");
//     mqttPrintln("TRIG Pin: " + String(TRIG_PIN) + ", ECHO Pin: " + String(ECHO_PIN));
    
//     mqttPrintln("\nTaking 20 distance measurements...");
//     float totalDistance = 0;
//     int validReadings = 0;
//     float minDistance = 999;
//     float maxDistance = 0;
    
//     for (int i = 0; i < 20; i++) {
//         // Trigger pulse
//         digitalWrite(TRIG_PIN, LOW);
//         delayMicroseconds(2);
//         digitalWrite(TRIG_PIN, HIGH);
//         delayMicroseconds(10);
//         digitalWrite(TRIG_PIN, LOW);
        
//         // Read echo
//         long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
        
//         if (duration > 0) {
//             float distance = (duration * 0.034) / 2; // Convert to cm
            
//             mqttPrintln("Reading " + String(i+1) + ": " + String(distance, 2) + " cm");
            
//             if (distance > 2 && distance < 400) { // Valid range for HC-SR04
//                 totalDistance += distance;
//                 validReadings++;
                
//                 if (distance < minDistance) minDistance = distance;
//                 if (distance > maxDistance) maxDistance = distance;
//             } else {
//                 mqttPrintln("   ⚠️ Out of range reading");
//             }
//         } else {
//             mqttPrintln("Reading " + String(i+1) + ": Timeout/No echo");
//         }
        
//         delay(200);
//     }
    
//     mqttPrintln("\nUltrasonic Sensor Test Results:");
//     mqttPrintln("- Initialization: ✅ Working");
//     mqttPrintln("- Valid readings: " + String(validReadings) + "/20");
    
//     if (validReadings > 0) {
//         float avgDistance = totalDistance / validReadings;
//         mqttPrintln("- Average distance: " + String(avgDistance, 2) + " cm");
//         mqttPrintln("- Min distance: " + String(minDistance, 2) + " cm");
//         mqttPrintln("- Max distance: " + String(maxDistance, 2) + " cm");
//         mqttPrintln("- Measurement accuracy: " + String(validReadings >= 15 ? "✅ Good" : "⚠️ Fair"));
//     } else {
//         mqttPrintln("- ❌ No valid readings obtained");
//     }
    
//     mqttPrintln("- TRIG pin control: ✅ Working");
//     mqttPrintln("- ECHO pin reading: " + String(validReadings > 0 ? "✅ Working" : "❌ Not working"));
    
//     // Send results to Raspberry Pi
//     if (networkInitialized) {
//         String resultMsg = "{\"type\":\"TEST_RESULT\",\"test\":\"ULTRASONIC\",\"valid_readings\":" + 
//                           String(validReadings) + ",\"avg_distance\":" + String(validReadings > 0 ? totalDistance/validReadings : 0, 2) + 
//                           ",\"status\":\"" + String(validReadings > 10 ? "success" : "poor") + "\"}";
//         net.send("esp32/test/results", resultMsg.c_str());
//     }
// }

// void initializeNetwork() {
//     mqttPrintln("🌐 Initializing Network Connection...");
    
//     mqttPrintln("SSID: " + String(ssid));
//     mqttPrintln("MQTT Host: " + String(mqttHost) + ":" + String(mqttPort));
    
//     // Begin connection
//     net.begin();
    
//     // Wait for connection with timeout
//     mqttPrintln("Connecting to WiFi...");
//     unsigned long startTime = millis();
//     while (WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
//         net.loop();
//         delay(500);
//         mqttPrint(".");
//     }
//     mqttPrintln("");
    
//     if (WiFi.status() == WL_CONNECTED) {
//         mqttPrintln("✅ WiFi Connected!");
//         mqttPrintln("IP Address: " + WiFi.localIP().toString());
//         mqttPrintln("Signal Strength: " + String(WiFi.RSSI()) + " dBm");
        
//         // Wait a bit more for MQTT connection
//         delay(2000);
//         for (int i = 0; i < 5; i++) {
//             net.loop();
//             delay(1000);
//         }
        
//         networkInitialized = true;
//         mqttPrintln("✅ MQTT Connection established!");
//         mqttPrintln("Subscribed to: esp32/test/commands");
        
//         // Send ready status to Raspberry Pi
//         net.send("esp32/test/status", "{\"type\":\"READY\",\"device\":\"ESP32_TESTER\",\"status\":\"online\"}");
        
//     } else {
//         mqttPrintln("❌ WiFi Connection failed!");
//         mqttPrintln("Continuing in manual mode only...");
//         networkInitialized = false;
//     }
// }

// void runTestFromMQTT(const String& command) {
//     if (testInProgress) {
//         mqttPrintln("⚠️ Test already in progress, ignoring command: " + command);
//         return;
//     }
    
//     testInProgress = true;
//     mqttPrintln("\n🤖 MQTT Command received: " + command);
//     mqttPrintln("===========================================");
    
//     // Send acknowledgment to Raspberry Pi
//     if (networkInitialized) {
//         String ackMsg = "{\"type\":\"ACK\",\"command\":\"" + command + "\",\"status\":\"starting\"}";
//         net.send("esp32/test/response", ackMsg.c_str());
//     }
    
//     if (command == "TEST_COIN") {
//         testCoinAcceptor();
//     } 
//     else if (command == "TEST_STEPPER") {
//         testStepperMotor();
//     }
//     else if (command == "TEST_RFID") {
//         testRFIDReader();
//     }
//     else if (command == "TEST_ULTRA") {
//         testUltrasonic();
//     }
//     else if (command == "TEST_ALL") {
//         mqttPrintln("🔄 RUNNING ALL TESTS SEQUENTIALLY");
//         mqttPrintln("This will take several minutes...");
        
//         testCoinAcceptor();
//         delay(2000);
//         testStepperMotor();
//         delay(2000);
//         testRFIDReader();
//         delay(2000);
//         testUltrasonic();
        
//         mqttPrintln("✅ ALL TESTS COMPLETED!");
//     }
//     else if (command == "STATUS") {
//         mqttPrintln("📊 SYSTEM STATUS REPORT");
//         mqttPrintln("WiFi Status: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
//         if (WiFi.status() == WL_CONNECTED) {
//             mqttPrintln("IP Address: " + WiFi.localIP().toString());
//             mqttPrintln("Signal Strength: " + String(WiFi.RSSI()) + " dBm");
//         }
//         mqttPrintln("MQTT Status: " + String(networkInitialized ? "Connected" : "Disconnected"));
//         mqttPrintln("Free Heap: " + String(ESP.getFreeHeap()) + " bytes");
//         mqttPrintln("Uptime: " + String(millis() / 1000) + " seconds");
//         mqttPrintln("Test in Progress: " + String(testInProgress ? "Yes" : "No"));
        
//         // Send status to Raspberry Pi
//         if (networkInitialized) {
//             String statusMsg = "{\"type\":\"STATUS\",\"wifi\":\"" + 
//                              String(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected") + 
//                              "\",\"mqtt\":\"" + String(networkInitialized ? "connected" : "disconnected") + 
//                              "\",\"uptime\":" + String(millis() / 1000) + 
//                              ",\"free_heap\":" + String(ESP.getFreeHeap()) + "}";
//             net.send("esp32/test/response", statusMsg.c_str());
//         }
//     }
//     else {
//         mqttPrintln("❌ Unknown command: " + command);
//         mqttPrintln("Valid commands: TEST_COIN, TEST_STEPPER, TEST_RFID, TEST_ULTRA, TEST_ALL, STATUS");
        
//         if (networkInitialized) {
//             String errorMsg = "{\"type\":\"ERROR\",\"message\":\"Unknown command: " + command + "\"}";
//             net.send("esp32/test/response", errorMsg.c_str());
//         }
//     }
    
//     // Send completion status to Raspberry Pi
//     if (networkInitialized && command != "STATUS") {
//         String completeMsg = "{\"type\":\"COMPLETE\",\"command\":\"" + command + "\",\"status\":\"finished\"}";
//         net.send("esp32/test/response", completeMsg.c_str());
//     }
    
//     testInProgress = false;
//     mqttPrintln("===========================================");
//     mqttPrintln("✅ Command completed: " + command);
//     mqttPrintln("Ready for next command...");
// }

// void mqttPrint(const String& message) {
//     // Send to real serial
//     Serial.print(message);
    
//     // Send to MQTT if connected
//     if (networkInitialized && message.length() > 0) {
//         String mqttMsg = "{\"type\":\"SERIAL\",\"data\":\"" + message + "\"}";
//         net.send("esp32/test/serial", mqttMsg.c_str());
//     }
// }

// void mqttPrintln(const String& message) {
//     // Send to real serial
//     Serial.println(message);
    
//     // Send to MQTT if connected
//     if (networkInitialized) {
//         String mqttMsg = "{\"type\":\"SERIAL\",\"data\":\"" + message + "\"}";
//         net.send("esp32/test/serial", mqttMsg.c_str());
//     }
// }

// void handleSerialInput(const String& input) {
//     String trimmedInput = input;
//     trimmedInput.trim();
//     trimmedInput.toUpperCase();
    
//     if (trimmedInput == "H") {
//         setup(); // Show help again
//         return;
//     }
    
//     if (trimmedInput == "R") {
//         ESP.restart();
//         return;
//     }
    
//     // Handle MQTT-style commands
//     if (trimmedInput.startsWith("TEST_") || trimmedInput == "STATUS") {
//         if (!testInProgress) {
//             runTestFromMQTT(trimmedInput);
//         } else {
//             mqttPrintln("⚠️ Test in progress, please wait...");
//         }
//         return;
//     }
    
//     // Handle legacy numeric commands
//     int testNumber = trimmedInput.toInt();
//     if (testNumber >= 0 && testNumber < MAX_TESTS && !testInProgress) {
//         currentTest = testNumber;
//         testInProgress = true;
//         mqttPrintln("\n===========================================");
        
//         switch (currentTest) {
//             case 0: testCoinAcceptor(); break;
//             case 1: testStepperMotor(); break;
//             case 2: testRFIDReader(); break;
//             case 3: testUltrasonic(); break;
//             default:
//                 mqttPrintln("Invalid test number!");
//                 break;
//         }
        
//         testInProgress = false;
//         mqttPrintln("===========================================");
//         mqttPrintln("Test completed. Send another command to run a different test.");
//     } else if (testInProgress) {
//         mqttPrintln("⚠️ Test in progress, please wait...");
//     } else {
//         mqttPrintln("Invalid input! Send a test command, number 0-3, 'h' for help, or 'r' to restart");
//     }
// }