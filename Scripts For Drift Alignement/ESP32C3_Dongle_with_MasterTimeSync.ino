#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include "esp_timer.h" // For 64-bit precise microsecond timing

#define MAX_NODES 7
#define PIN_E 3  // GPIO you want to control
#define PIN_T 2  // GPIO you want to control

// Nordic UART Service UUIDs
static BLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID txUUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");  // write
static BLEUUID rxUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");  // notify
static BLEUUID syncServiceUUID("12345678-90ab-cdef-1234-567890abcdef"); 
static BLEUUID syncRxUUID("12345678-90ab-cdef-1234-567891abcdef");

BLEScan* pBLEScan;

struct NodeClient {
  String name;  // ← Must be an Arduino String
  BLEAdvertisedDevice device;
  BLEClient* client = nullptr;
  BLERemoteService* service = nullptr;
  BLERemoteCharacteristic* txChar = nullptr;
  BLERemoteCharacteristic* rxChar = nullptr;
  BLERemoteService* syncService = nullptr;
  BLERemoteCharacteristic* syncRxChar = nullptr;
  bool connected = false;
  
  // --- New Sync Variables ---
  int64_t lastPingSentUs = 0;
  int64_t t1_departure = 0;
  bool waitingForPong = false;

  int64_t recordingStartUs = 0;
};

NodeClient nodes[MAX_NODES];
int nodeCount = 0;
bool aliveEnabled = true;
volatile bool hasData = false;
char dataBuf[128];

String now_us() {
  char buf[32];
  unsigned long long us = millis();
  sprintf(buf, "[%010lu] ", us);
  return String(buf);
}

// Count connected nodes
int getConnectedCount() {
  int count = 0;
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].connected) count++;
  }
  return count;
}

// =========================================================================
// Dedicated Sync & Battery Notification Callback
// =========================================================================
void syncNotifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  // Capture T2 Arrival Time the microsecond the callback fires
  int64_t t2_arrival = esp_timer_get_time(); 

  BLEClient* client = pChar->getRemoteService()->getClient();
  String nodeName = "UNKNOWN";
  int nodeIndex = -1;

  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].client == client) {
      nodeName = nodes[i].name;
      nodeIndex = i;
      break;
    }
  }

  if (length < 1) return;

  // --- SYNC PONG PACKET ---
  if (pData[0] == 'p' && length >= 5) {
    if (nodeIndex != -1 && nodes[nodeIndex].waitingForPong) {
      uint32_t t_node = 0;
      memcpy(&t_node, &pData[1], sizeof(uint32_t));
      
      Serial.printf("[SYNC] %s T1=%lld T2=%lld Tnode=%u\n", 
                    nodeName.c_str(), nodes[nodeIndex].t1_departure, 
                    t2_arrival, t_node);
                    
      nodes[nodeIndex].waitingForPong = false;
    }
  }
  // --- BATTERY PACKET ---
  else if (pData[0] == 'b' && length >= 6) {
    uint8_t battery_percent = pData[1];
    uint32_t dropped = 0;
    memcpy(&dropped, &pData[2], sizeof(uint32_t));
    
    if (battery_percent == 255) {
      Serial.printf("[BATT] %s BUSY (Dropped: %u)\n", nodeName.c_str(), dropped);
    } else {
      Serial.printf("[BATT] %s %u%% (Dropped: %u)\n", nodeName.c_str(), battery_percent, dropped);
    }
  }
}

// Notification callback with node name
void notifyCallback(BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  // 1. INSTANTLY capture T2 Arrival Time the microsecond the callback fires
  int64_t t2_arrival = esp_timer_get_time(); 

  BLEClient* client = pChar->getRemoteService()->getClient();
  String nodeName = "UNKNOWN";
  int nodeIndex = -1;

  // Identify which node sent the data
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].client == client) {
      nodeName = nodes[i].name;
      nodeIndex = i;
      break;
    }
  }

  if (length < 1) return;

  // ==========================================
  // DATA PACKET: Expected format -> ['d', seq_counter, val1_L, val1_H, ...]
  // ==========================================
  if (pData[0] == 'd' && length >= 20) {
    uint8_t seq_counter = pData[1]; // Extract the 1-byte counter
    
    Serial.print("[DATA] ");
    Serial.print(nodeName);
    Serial.printf(" %u ", seq_counter); // Print node's timestamp immediately after name

    // Print 16-bit samples (starting at index 2)
    for (int i = 2; i + 1 < length; i += 2) {
      uint16_t v = pData[i] | (pData[i + 1] << 8);
      Serial.printf("%06u ", v);
    }
    Serial.println();
    return;
  }
}

// Scan for NODE 0–NODE 6
void scanNodesByUUID() {
  nodeCount = 0;

  Serial.println("=== Scan Start ===");

  BLEScanResults* results = pBLEScan->start(4, false);

  for (int i = 0; i < results->getCount(); i++) {
    
    // Bounds Check to prevent memory overflow 
    if (nodeCount >= MAX_NODES) {
        Serial.println("[SCAN] Max node limit reached.");
        break; 
    }

    BLEAdvertisedDevice dev = results->getDevice(i);

    // Print debug info (optional)
    //Serial.println(dev.toString().c_str());

    // Key: Identify device by UUID
    if (!dev.isAdvertisingService(serviceUUID)) continue;

    nodes[nodeCount].device = dev;
    nodes[nodeCount].name = "";  // Name read later
    nodeCount++;

    Serial.printf("[SCAN] Found device with NUS UUID: %s\n",
                  dev.getAddress().toString().c_str());
  }

  Serial.printf("[SCAN] Total nodes found: %d\n", nodeCount);
}

bool connectToNode(NodeClient& node) {
  // Memory Leak Fix (reusing existing client if available) 
  if (node.client == nullptr) {
      node.client = BLEDevice::createClient();
  }

  if (!node.client->connect(&node.device)) {
    Serial.printf("[Connect Failed] %s\n", node.device.getAddress().toString().c_str());
    return false;
  }

  node.service = node.client->getService(serviceUUID);
  if (!node.service) {
    Serial.println("[Error] NUS service not found");
    return false;
  }

  node.txChar = node.service->getCharacteristic(txUUID);
  node.rxChar = node.service->getCharacteristic(rxUUID);

  if (!node.txChar || !node.rxChar) {
    Serial.println("[Error] TX/RX characteristic missing");
    return false;
  }

  if (node.rxChar->canNotify()) {
    node.rxChar->registerForNotify(notifyCallback);
  }

  node.syncService = node.client->getService(syncServiceUUID);
  if (node.syncService) {
    node.syncRxChar = node.syncService->getCharacteristic(syncRxUUID);
    
    if (node.syncRxChar && node.syncRxChar->canNotify()) {
      node.syncRxChar->registerForNotify(syncNotifyCallback);
      Serial.println("[Setup] Sync characteristic found and registered.");
    } else {
      Serial.println("[Warning] Sync characteristic missing or cannot notify.");
    }
  } else {
    Serial.println("[Warning] Sync service not found on this node.");
  }

  node.connected = true;

  Serial.printf("[Connected] %s\n", node.device.getAddress().toString().c_str());
  return true;
}

String readDeviceName(BLEClient* client) {
  BLERemoteService* gap = client->getService(BLEUUID((uint16_t)0x1800));
  if (!gap) return "";

  BLERemoteCharacteristic* devNameChar =
    gap->getCharacteristic(BLEUUID((uint16_t)0x2A00));
  if (!devNameChar) return "";

  String name = devNameChar->readValue();
  return name;  // ← Correct conversion
}

void connectAndFilterNodes() {
  int filteredCount = 0;

  for (int i = 0; i < nodeCount; i++) {

    Serial.printf("[CONNECT] Connecting to %s...\n",
                  nodes[i].device.getAddress().toString().c_str());

    if (!connectToNode(nodes[i])) {
      Serial.println("[CONNECT] Failed.");
      continue;
    }

    // Read device name after successful connection
    String devName = readDeviceName(nodes[i].client);
    nodes[i].name = devName;

    Serial.printf("[NAME] %s\n", devName.c_str());

    // Key filtering logic 
    if (!devName.startsWith("DEMO NODE")) {
      Serial.println("[FILTER] Name not matched. Disconnecting...");
      nodes[i].client->disconnect();
      nodes[i].connected = false;
      continue;
    }

    // Name matches -> save to the front
    nodes[filteredCount] = nodes[i];
    filteredCount++;

    Serial.printf("[ACCEPT] %s accepted.\n", devName.c_str());
  }

  nodeCount = filteredCount;
  Serial.printf("[RESULT] Total accepted nodes: %d\n", nodeCount);
}


void setup() {
  Serial.begin(921600);

  pinMode(PIN_T, OUTPUT);
  pinMode(PIN_E, OUTPUT);

  digitalWrite(PIN_E, LOW);
  digitalWrite(PIN_T, HIGH);


  BLEDevice::init("ESP32S3 Multi-NUS Client");


  pBLEScan = BLEDevice::getScan();
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(1300);
  pBLEScan->setAdvertisedDeviceCallbacks(nullptr);

  delay(1000);
  Serial.println("Serial commands: '?'=identify, 'a'=start sampling, 's'=stop sampling");
  Serial.println("Loop alive");
}

void loop() {
  int64_t current_us = esp_timer_get_time();

  // ============================
  // 1. PING-PONG SYNC DISPATCHER (Every 3 seconds per node)
  // ============================
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].connected && nodes[i].txChar) {
      
      // If we've waited more than 500ms for a Pong, assume it was lost and reset
      if (nodes[i].waitingForPong && (current_us - nodes[i].t1_departure > 500000ULL)) {
         nodes[i].waitingForPong = false; 
      }

      // If less than 30 seconds have passed since start, use 3s interval. Otherwise, use 10s.
      int64_t elapsedSinceStart = current_us - nodes[i].recordingStartUs;
      int64_t currentIntervalUs = (elapsedSinceStart < 30000000ULL) ? 3000000ULL : 10000000ULL;
      //int64_t currentIntervalUs = 1500000ULL;
      // If it's been 3,000,000 us (3 seconds) since last ping, ping again
      if (!nodes[i].waitingForPong && (current_us - nodes[i].lastPingSentUs > currentIntervalUs)) {
         nodes[i].lastPingSentUs = current_us;
         nodes[i].t1_departure = esp_timer_get_time(); // Record Exact T1
         nodes[i].waitingForPong = true;
         
         uint8_t pingCmd[1] = {'p'}; 
         nodes[i].txChar->writeValue(pingCmd, 1, false); // Send Ping
      }
    }
  }

  // ============================
  // 2. Continuously send ESP32-ALIVE in idle state
  // ============================
  static unsigned long lastAlive = 0;
  if (aliveEnabled && millis() - lastAlive > 1000) {
    Serial.println("ESP32-ALIVE");
    lastAlive = millis();
  }

  // ============================
  // 3. Serial command processing
  // ============================
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') continue;

    //  Stop sending ALIVE upon receiving any command 
    aliveEnabled = false;

    if (c == '?') {
      Serial.println("ESP32-READY");
      continue;
    }

    if (c == 'c') {
      Serial.println("[CMD] Connect request received.");
      nodeCount = 0;
      scanNodesByUUID();
      connectAndFilterNodes();
      Serial.println("[READY] Connect sequence finished.");
      continue;
    }

    // ==========================================
    // START RECORDING ('a')
    // ==========================================
    if (c == 'a') {
      // --- NEW: Poll battery BEFORE starting ---
      Serial.println("[PC] Pre-recording battery poll initiated...");
      uint8_t battCmd[1] = {'b'};
      for (int i = 0; i < nodeCount; i++) {
        if (nodes[i].connected && nodes[i].txChar) {
          nodes[i].txChar->writeValue(battCmd, 1, false);
        }
      }
      delay(200); // Give nodes 200ms to measure VDD and reply before starting ADC
      
      int64_t start_time = esp_timer_get_time();
      // Original start command
      uint8_t cmd[2] = { 'a', 0x00 };
      for (int i = 0; i < nodeCount; i++) {
        if (nodes[i].connected && nodes[i].txChar) {
          nodes[i].recordingStartUs = start_time;
          nodes[i].txChar->writeValue(cmd, 2, false);
        }
      }
      Serial.println("[PC] Start command received.");
      delay(100);
      digitalWrite(PIN_T, LOW);
      Serial.println("[LOCAL] Start Triggered");
      delay(100);
      digitalWrite(PIN_T, HIGH);
      continue;
    }

    // ==========================================
    // STOP RECORDING ('s')
    // ==========================================
    if (c == 's') {
      digitalWrite(PIN_T, LOW);
      Serial.println("[LOCAL] Stop Triggered"); 
      delay(100);
      digitalWrite(PIN_T, HIGH);
      
      // Original stop command
      uint8_t cmd[2] = { 's', 0x00 };
      for (int i = 0; i < nodeCount; i++) {
        if (nodes[i].connected && nodes[i].txChar) {
          nodes[i].txChar->writeValue(cmd, 2, false);
        }
      }
      Serial.println("[PC] Stop command received.");
      
      // --- NEW: Poll battery AFTER stopping ---
      delay(200); // Wait 200ms to ensure nodes have completely stopped their ADC interrupts
      Serial.println("[PC] Post-recording battery poll initiated...");
      uint8_t battCmd[1] = {'b'};
      for (int i = 0; i < nodeCount; i++) {
        if (nodes[i].connected && nodes[i].txChar) {
          nodes[i].txChar->writeValue(battCmd, 1, false);
        }
      }

      aliveEnabled = true;
      continue;
    }

    // ============================
    // Local GPIO control commands (not forwarded)
    // ============================

    if (c == 'e') {  // Pull PIN_E high
      pinMode(PIN_E, OUTPUT);
      digitalWrite(PIN_E, HIGH);
      Serial.println("[LOCAL] Discharge Enabled");
      continue;
    }

    if (c == 'd') {  // Set PIN_E as INPUT (disables discharge)
      pinMode(PIN_E, INPUT);
      Serial.println("[LOCAL] Discharge Disabled");
      continue;
    }

    if (c == 't') {  // Pull PIN_T high then low
      digitalWrite(PIN_T, HIGH);
      Serial.println("[LOCAL] Start Triggered");
      delay(10);
      digitalWrite(PIN_T, LOW);
      continue;
    }
  }

  // ============================
  // 4. BLE Auto-reconnect
  // ============================
  for (int i = 0; i < nodeCount; i++) {
    if (nodes[i].client == nullptr) continue;

    if (nodes[i].connected && !nodes[i].client->isConnected()) {
      Serial.printf("[Disconnected] %s. Reconnecting...\n", nodes[i].name.c_str());
      nodes[i].connected = false;

      if (connectToNode(nodes[i])) {
        Serial.printf("[Reconnected] %s. Current connections: %d/%d\n",
                      nodes[i].name.c_str(),
                      getConnectedCount(),
                      nodeCount);
      }
    }
  }

  delay(5); // Replaced 10ms with 5ms 
}
