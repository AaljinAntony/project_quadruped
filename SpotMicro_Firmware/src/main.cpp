#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>

// --- NETWORK CONFIGURATION ---
// Target: Host machine on Home Wi-Fi
const char* ssid = "WE_Kaliyath _2_4G"; 
const char* psk  = "nisha8182";
IPAddress agent_ip(192, 168, 1, 34); 
const uint16_t agent_port = 8888;

WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  delay(3000); 
  Serial.println("\n\n===============================================");
  Serial.println("   ESP32 NETWORK DIAGNOSTIC SUITE v1.1         ");
  Serial.println("===============================================");
  
  // 1. SCAN PHASE
  Serial.println("\n[PHASE 1] Wi-Fi Scan");
  Serial.println("[*] Searching for 2.4GHz access points...");
  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("[!] ERROR: No networks found.");
  } else {
    Serial.printf("[+] Found %d networks. Looking for '%s'...\n", n, ssid);
    bool found_target = false;
    for (int i = 0; i < n; ++i) {
      Serial.printf("    - %-20s (RSSI: %3d dBm, Ch: %2d) %s\n", 
                    WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                    (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "OPEN" : "SECURE");
      if (WiFi.SSID(i) == ssid) found_target = true;
    }
  }

  // 2. CONNECTION PHASE
  Serial.println("\n[PHASE 2] Connecting to Wi-Fi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, psk);
  Serial.printf("[*] Connecting to %s ", ssid);
  
  unsigned long start_attempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start_attempt < 20000) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[+] CONNECTION SUCCESSFUL!");
    Serial.print("    - Local IP:   "); Serial.println(WiFi.localIP());
    Serial.print("    - Gateway:    "); Serial.println(WiFi.gatewayIP());
    
    // CRITICAL: Disable power-save
    WiFi.setSleep(false);
    esp_wifi_set_ps(WIFI_PS_NONE);
    Serial.println("[*] Wi-Fi Power Save DISABLED.");
  } else {
    Serial.printf("[!] FATAL: Connection failed with status %d\n", WiFi.status());
    return;
  }

  // 3. GATEWAY PING PHASE
  Serial.println("\n[PHASE 3] Link Layer Verification (Gateway)");
  IPAddress gw = WiFi.gatewayIP();
  Serial.printf("[*] Pinging Gateway %s ...\n", gw.toString().c_str());
  if (Ping.ping(gw, 3)) {
    Serial.println("[+] Gateway REACHABLE.");
  } else {
    Serial.println("[!] ERROR: Gateway UNREACHABLE.");
  }

  // 4. HOST PING PHASE
  Serial.println("\n[PHASE 4] Host Layer Verification (Laptop)");
  Serial.printf("[*] Pinging Host Laptop %s ...\n", agent_ip.toString().c_str());
  if (Ping.ping(agent_ip, 3)) {
    Serial.println("[+] Host REACHABLE!");
  } else {
    Serial.println("[!] ERROR: Host UNREACHABLE.");
    Serial.println("[?] Possible cause: Firewall on laptop.");
  }

  // 5. UDP LOOPBACK PHASE
  Serial.println("\n[PHASE 5] Transport Layer Verification (UDP)");
  Serial.printf("[*] Sending to %s:%d\n", agent_ip.toString().c_str(), agent_port);
  udp.begin(agent_port);
  
  for (int i = 1; i <= 5; i++) {
    Serial.printf("[*] Sending UDP Probe #%d ... ", i);
    udp.beginPacket(agent_ip, agent_port);
    udp.printf("DIAG_PROBE_%d_CLK_%lu", i, millis());
    
    if (udp.endPacket() == 1) {
      Serial.println("SENT OK.");
    } else {
      Serial.print("FAILED (Error: "); Serial.print(12); Serial.println(")");
    }

    // Wait for ACK
    unsigned long wait_start = millis();
    bool ack_received = false;
    while (millis() - wait_start < 2000) {
      int packetSize = udp.parsePacket();
      if (packetSize) {
        char buffer[100];
        int len = udp.read(buffer, sizeof(buffer)-1);
        if (len > 0) buffer[len] = '\0';
        Serial.printf("    [+] RECEIVED ACK: '%s'\n", buffer);
        ack_received = true;
        break;
      }
      delay(10);
    }
    if (!ack_received) Serial.println("    [-] NO RESPONSE.");
    delay(1000);
  }

  Serial.println("\n[DIAGNOSTIC COMPLETE]");
  Serial.println("Send 'r' to restart test.");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') ESP.restart();
  }
}