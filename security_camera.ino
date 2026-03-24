#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <FS.h>
#include <SPIFFS.h>

#include <RTClib.h>

#include <ArduCAM.h>

#ifdef swap
#undef swap
#endif

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

#include <esp_coexist.h>

// WiFi 
const char* ssid = "network_name";
const char* password = "password";

IPAddress local_IP(10, 40, 51, 200);
IPAddress gateway(10, 40, 51, 1);
IPAddress subnet(255, 255, 255, 0);

// RTC 
RTC_DS1307 rtc;
// Second I2C bus on different pins
TwoWire I2C_RTC = TwoWire(1);  // Bus number 1

// Pins
#define PIR_PIN 13
#define CS_PIN 15
//#define CAM_POWER 4
ArduCAM myCAM(OV2640, CS_PIN);

// BLEs
#define SECURITY_SERVICE_UUID             "f1a10000-0000-4000-b000-000000000010"
#define INTRUSION_CHARACTERISTIC_UUID     "f1a10000-0000-4000-b000-000000000011"
#define BATTERY_CHARACTERISTIC_UUID       "f1a10000-0000-4000-b000-000000000012"
#define TIME_CHARACTERISTIC_UUID          "f1a10000-0000-4000-b000-000000000013"

BLECharacteristic *intrusionChar;
BLECharacteristic *batteryChar;
BLECharacteristic *timeChar;

bool deviceConnected = false;   

// Globals
volatile bool motionDetected = false;
volatile int intrusionCount = 0; 

SemaphoreHandle_t timeMutex = NULL;

SemaphoreHandle_t spiffsMutex = NULL;

float batteryVolt = 0;
String currentTime = ""; 

// 3x3 grid weights — center weighted
const float GRID_WEIGHTS[9] = {
  0.5f, 0.75f, 0.5f,
  0.75f, 1.5f, 0.75f,
  0.5f, 0.75f, 0.5f
};

struct FrameSignature {
  uint32_t zoneSums[9];
  uint32_t zoneSamples[9];
  uint32_t totalSize;
};

// Task Handlers 
TaskHandle_t MotionDetectionHandler = NULL;
TaskHandle_t ImageCapHandler = NULL;
TaskHandle_t ImageProcHandler = NULL;
TaskHandle_t BLEHandler = NULL;
TaskHandle_t RTCHandler = NULL;
TaskHandle_t PowerHandler = NULL;
TaskHandle_t BatteryHandler = NULL;
TaskHandle_t OTAHandler = NULL;

// BLE Callbacks
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    BLEDevice::startAdvertising();
  }
};

// Init camera function
void initCamera() {

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  delay(1000);

  // Retry SPI test up to 5 times
  bool spiOk = false;
  for (int i = 0; i < 5; i++) {
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    if (myCAM.read_reg(ARDUCHIP_TEST1) == 0x55) {
      spiOk = true;
      break;
    }
    Serial.println("SPI retry...");
    delay(200);
  }

  if (!spiOk) {
    Serial.println("SPI communication failed!");
    while(true) {
      delay(1000); // stop here instead of continuing broken
    }
  }

  // Reset the camera
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_160x120);
  delay(1000);

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();

  Serial.println("Camera initialized.");
}

// Initialize the BLE
void initBLE(){
  BLEDevice::init("KatSecurityCam");

  // Slow down BLE advertising interval to reduce WiFi interference
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setMinInterval(0x100);  // 160ms
  pAdvertising->setMaxInterval(0x200);  // 320ms


  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *securityService = pServer->createService(SECURITY_SERVICE_UUID);

  intrusionChar = securityService->createCharacteristic(
    INTRUSION_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_READ
  );
  intrusionChar->addDescriptor(new BLE2902());

  batteryChar = securityService->createCharacteristic(
      BATTERY_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_READ);

  batteryChar->addDescriptor(new BLE2902());

  timeChar = securityService->createCharacteristic(
      TIME_CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_READ);

  timeChar->addDescriptor(new BLE2902());

  securityService->start();

  BLEAdvertising *advertising =
      BLEDevice::getAdvertising();

  advertising->addServiceUUID(SECURITY_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("BLE Ready");
}

void cameraWake(){
  Serial.println("Waking camera");
  // digitalWrite(CAM_POWER, HIGH);  // turn camera ON
  // delay(300);                     // let it boot

  myCAM.wrSensorReg8_8(0xFF, 0x01);
  myCAM.wrSensorReg8_8(0x09, 0x00);
  delay(100);
  initCamera();
  delay(1500);

  // discard warmup frame
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
  unsigned long start = millis();
  while(!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)){
    if(millis() - start > 3000) break;
    delay(10);
  }
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  delay(500);
}

void cameraSleep(){
  Serial.println("Putting camera into standby");
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.wrSensorReg8_8(0xFF, 0x01);
  myCAM.wrSensorReg8_8(0x09, 0x10);
  digitalWrite(CS_PIN, HIGH);

  //digitalWrite(CAM_POWER, LOW);   // PN2222 cuts ground (camera off)
  delay(50);                     
}

bool buildSignature(File &file, FrameSignature &sig){
  memset(&sig, 0, sizeof(sig));

  uint32_t fileSize = file.size();
  if(fileSize < 300) return false;

  sig.totalSize = fileSize;

  // Skip JPEG header
  const uint32_t HEADER_SKIP = 200;
  const uint32_t dataSize = fileSize - HEADER_SKIP;
  const uint32_t MAX_SAMPLES = 800;

  // Step through the file at even intervals
  uint32_t step = max((uint32_t)1, dataSize / MAX_SAMPLES);

  file.seek(HEADER_SKIP);

  uint32_t pos = 0;
  while(file.available() && pos < dataSize){
    uint8_t val = file.read();

    // Determine which grid zone this byte falls in (0-8)
    // Map byte position to a 3x3 grid
    uint32_t zoneX = (pos * 3) / dataSize;  // 0,1,2
    // Approximate Y zone using byte value bands (rough but valid heuristic)
    uint32_t zoneY = (val < 85) ? 0 : (val < 170) ? 1 : 2;
    uint32_t zone = zoneY * 3 + zoneX;

    if(zone < 9){
      sig.zoneSums[zone] += val;
      sig.zoneSamples[zone]++;
    }

    pos += step;
    // Seek forward by step-1 since we already read 1 byte
    if(step > 1) file.seek(HEADER_SKIP + pos);
  }

  return true;
}

float compareSignatures(FrameSignature &a, FrameSignature &b){
  float weightedDiff = 0;
  float totalWeight = 0;

  for(int i = 0; i < 9; i++){
    if(a.zoneSamples[i] == 0 || b.zoneSamples[i] == 0) continue;

    float avgA = (float)a.zoneSums[i] / a.zoneSamples[i];
    float avgB = (float)b.zoneSums[i] / b.zoneSamples[i];

    float diff = fabsf(avgA - avgB);
    weightedDiff += diff * GRID_WEIGHTS[i];
    totalWeight += GRID_WEIGHTS[i];
  }

  // Also factor in file size difference as secondary signal
  float sizeDiff = fabsf((float)a.totalSize - (float)b.totalSize) / max(a.totalSize, b.totalSize) * 100.0f;

  Serial.printf("Weighted zone diff: %.2f, Size diff: %.2f%%\n", weightedDiff / totalWeight, sizeDiff);

  if(totalWeight == 0){
    return 0;
  }
  return (weightedDiff / totalWeight) + (sizeDiff * 0.3f);
}

bool detectIntrusion(){
  if(!xSemaphoreTake(spiffsMutex, portMAX_DELAY)) return false;

  if(!SPIFFS.exists("/capture.jpg")){
    Serial.println("capture.jpg missing");
    xSemaphoreGive(spiffsMutex);
    return false;
  }

  if(!SPIFFS.exists("/previous.jpg")){
    Serial.println("No previous image. Creating baseline.");
    File cur = SPIFFS.open("/capture.jpg", FILE_READ);
    File prev = SPIFFS.open("/previous.jpg", FILE_WRITE);
    if(!cur || !prev){
      Serial.println("Failed to create baseline");
      if(cur) cur.close();
      if(prev) prev.close();
      xSemaphoreGive(spiffsMutex);
      return false;
    }
    while(cur.available()) prev.write(cur.read());
    cur.close();
    prev.close();
    Serial.println("Baseline created");
    xSemaphoreGive(spiffsMutex);
    return false;
  }

  File cur = SPIFFS.open("/capture.jpg", FILE_READ);
  File prev = SPIFFS.open("/previous.jpg", FILE_READ);

  if(!cur || !prev){
    Serial.println("Failed to open images");
    if(cur) cur.close();
    if(prev) prev.close();
    xSemaphoreGive(spiffsMutex);
    return false;
  }

  FrameSignature sigCur, sigPrev;
  bool okCur = buildSignature(cur, sigCur);
  bool okPrev = buildSignature(prev, sigPrev);

  cur.close();
  prev.close();

  if(!okCur || !okPrev){
    Serial.println("Signature build failed");
    xSemaphoreGive(spiffsMutex);
    return false;
  }

  float score = compareSignatures(sigCur, sigPrev);
  Serial.printf("Motion score: %.2f\n", score);

  bool intrusion = score > 5.0f;

  // Update baseline
  SPIFFS.remove("/previous.jpg");
  File newPrev = SPIFFS.open("/previous.jpg", FILE_WRITE);
  File newCur = SPIFFS.open("/capture.jpg", FILE_READ);
  if(newPrev && newCur){
    while(newCur.available()) newPrev.write(newCur.read());
  }
  if(newPrev) newPrev.close();
  if(newCur) newCur.close();

  xSemaphoreGive(spiffsMutex);
  return intrusion;
}

void MotionDetectionManager(void *parameters){
  // Motion detection 
  pinMode(PIR_PIN, INPUT);
  unsigned long lastTrigger = 0;

  while(true){
    if(digitalRead(PIR_PIN) == HIGH && millis() - lastTrigger > 5000){
      Serial.println("Motion detected");
      motionDetected = true;
      lastTrigger = millis();
      xTaskNotifyGive(ImageCapHandler);
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void ImageCapManager(void *parameters){
  while(true){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("Capturing Image");

    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    delay(50);
    myCAM.start_capture();

    // Wait for capture to complete
    unsigned long start = millis();
    while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)){
      if(millis() - start > 8000){
        Serial.println("Capture timeout");
        break;
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // Skip if capture didn't finish
    if(!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)){
      myCAM.clear_fifo_flag();
      continue;
    }

    uint32_t length = myCAM.read_fifo_length();

    Serial.print("Image size (bytes): ");
    Serial.println(length);

    if(length == 0 || length > 500000){
      Serial.println("Invalid image size");
      myCAM.clear_fifo_flag();
      continue;
    }

    if(xSemaphoreTake(spiffsMutex, portMAX_DELAY)){

      File file = SPIFFS.open("/capture.jpg", FILE_WRITE);

      if(!file){
        Serial.println("Failed to open file for writing");
        xSemaphoreGive(spiffsMutex);
        myCAM.clear_fifo_flag();
        continue;
      }

      const size_t CHUNK = 256;
      uint8_t buf[CHUNK];

      myCAM.CS_LOW();
      myCAM.set_fifo_burst();

      uint32_t remaining = length;
      while(remaining > 0){
        size_t toRead = (remaining > CHUNK) ? CHUNK : remaining;
        for(size_t i = 0; i < toRead; i++){
          buf[i] = SPI.transfer(0x00);
        }
        file.write(buf, toRead);
        remaining -= toRead;
        vTaskDelay(1 / portTICK_PERIOD_MS);
      }

      myCAM.CS_HIGH();
      file.close();
      xSemaphoreGive(spiffsMutex);
    }

    myCAM.clear_fifo_flag();
    Serial.println("Image saved");
    vTaskDelay(50 / portTICK_PERIOD_MS); // let SPIFFS flush
    xTaskNotifyGive(ImageProcHandler);
  }
}

void ImageProcManager(void *parameters){
  // Image processing -> recognition functions
  while(true){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("Processing Image");

    // Intrusion count
    bool intrusion = detectIntrusion();

    if(intrusion){
      intrusionCount++;

      Serial.print("Intrusions: "); // will send this over BLE
      Serial.println(intrusionCount);
      
      xTaskNotifyGive(BLEHandler);
    }
  }
}

void BLEManager(void *parameters){
  const int statusInterval = 10000; // 10 seconds
  unsigned long lastStatusTime = 0;

  while(true){
    // Wait for notification OR timeout
    uint32_t notification = ulTaskNotifyTake(
        pdTRUE,
        statusInterval / portTICK_PERIOD_MS
    );

    unsigned long now = millis();

    String timeCopy = "";
    if(xSemaphoreTake(timeMutex, 100 / portTICK_PERIOD_MS)){
      timeCopy = currentTime;
      xSemaphoreGive(timeMutex);
    }
    char buffer[16];

    if(notification > 0){
      // if there is an intruder, update right away
      if(deviceConnected && intrusionChar != nullptr){
        sprintf(buffer,"%d",intrusionCount);
        intrusionChar->setValue(buffer);
        intrusionChar->notify();
      }
      if(deviceConnected && batteryChar != nullptr){
        sprintf(buffer,"%.2f",batteryVolt);
        batteryChar->setValue(buffer);
        batteryChar->notify();
      }
      if(deviceConnected && timeChar != nullptr){
        timeChar->setValue(currentTime.c_str());
        timeChar->notify();       
      }
    }

    // Normal updates every 10 seconds
    if(now - lastStatusTime > statusInterval){
      if(deviceConnected && intrusionChar != nullptr){
        sprintf(buffer,"%d",intrusionCount);
        intrusionChar->setValue(buffer);
        intrusionChar->notify();
      }
      if(deviceConnected && batteryChar != nullptr){
        sprintf(buffer,"%.2f",batteryVolt);
        batteryChar->setValue(buffer);
        batteryChar->notify();
      }
      if(deviceConnected && timeChar != nullptr){
        timeChar->setValue(currentTime.c_str());
        timeChar->notify();       
      }
      lastStatusTime = now;
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void RTCManager(void *parameters){
  while(true){
    DateTime now = rtc.now();

    String newTime = String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
    if(xSemaphoreTake(timeMutex, portMAX_DELAY)){
      currentTime = newTime;
      xSemaphoreGive(timeMutex);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void PowerManager(void *parameters){
  while(true){
    static unsigned long lastMotion = millis();
    if(motionDetected){
      lastMotion = millis();
      motionDetected = false;
    }
    // after 30 seconds of nothing, 
    if(millis() - lastMotion > 30000){
      // Suspend all other tasks before sleeping
      vTaskSuspend(MotionDetectionHandler);
      vTaskSuspend(ImageCapHandler);
      vTaskSuspend(ImageProcHandler);
      vTaskSuspend(BLEHandler);
      vTaskSuspend(RTCHandler);
      vTaskSuspend(BatteryHandler);

      // Set wakeup sources right before sleep
      esp_sleep_enable_ext0_wakeup((gpio_num_t)PIR_PIN, 1);
      esp_sleep_enable_timer_wakeup(60000000ULL); // 60 seconds

      cameraSleep(); // turn camera off 
      
      // Disconnect Wifi when it goes to sleep for power 
      ArduinoOTA.end();
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      Serial.println("WiFi off"); 

      Serial.println("Entering light sleep...");
      Serial.flush();        // wait for serial to finish transmitting
      delay(100);            // give it extra time 

      esp_light_sleep_start();

      cameraWake(); // turn camera back on after it wakes

      // Reconnect WiFi after wake
      WiFi.config(local_IP, gateway, subnet);
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      unsigned long wifiStart = millis();
      while(WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000){
        delay(200);
      }
      if(WiFi.status() == WL_CONNECTED){
        Serial.println("WiFi reconnected");
        MDNS.end();
        MDNS.begin("Kat-Cam");
        MDNS.addService("arduino", "tcp", 3232);
        ArduinoOTA.begin();
      } else {
        Serial.println("WiFi reconnect failed");
      }
      

      esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
      if(reason == ESP_SLEEP_WAKEUP_EXT0){
        Serial.println("Woke up: motion");
        motionDetected = true;
        lastMotion = millis();
      } else {
        Serial.println("Woke up: timer");
      }
      motionDetected = false;
      lastMotion = millis();

      vTaskResume(MotionDetectionHandler);
      vTaskResume(ImageCapHandler);
      vTaskResume(ImageProcHandler);
      vTaskResume(BLEHandler);
      vTaskResume(RTCHandler);
      vTaskResume(BatteryHandler);
    }
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void BatteryManager(void *parameters){
  while(true){
    batteryVolt = 5.0; // always 100% since USB powered

    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }
}

void OTAManager(void *parameters){
  ArduinoOTA.setHostname("Kat-Cam"); 
  ArduinoOTA.begin();

  while(true){

    ArduinoOTA.handle();

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Force CS high before SPI starts
  // pinMode(CAM_POWER, OUTPUT);
  // digitalWrite(CAM_POWER, HIGH);
  // delay(200);
  SPI.begin(18, 19, 23, CS_PIN);
  SPI.setFrequency(1000000);  // 1 MHz
  delay(200);

  Wire.begin();
  delay(200);

  // initialize camera
  initCamera();
  delay(3000);

  esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  delay(2000);

  if (!MDNS.begin("Kat-Cam")) {
    Serial.println("mDNS failed");
  }

  MDNS.addService("arduino", "tcp", 3232);

  I2C_RTC.begin(14, 27);  // SDA=14, SCL=27

  // Initialize RTC 
  if (!rtc.begin(&I2C_RTC)) {
    Serial.println("Couldn't find RTC!");
    while (1);
  }

  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  // Only format if stale data is present from a previous session
  if(SPIFFS.exists("/capture.jpg")) SPIFFS.remove("/capture.jpg");
  if(SPIFFS.exists("/previous.jpg")) SPIFFS.remove("/previous.jpg");

  Serial.printf("SPIFFS total: %d, used: %d\n", SPIFFS.totalBytes(), SPIFFS.usedBytes());

  // inititalize BLE 
  initBLE();

  timeMutex = xSemaphoreCreateMutex();
  spiffsMutex = xSemaphoreCreateMutex();

  // Motion detection task 
  xTaskCreatePinnedToCore(
    MotionDetectionManager,
    "MotionDetectionManager",
    4096, 
    NULL,
    5, // want this to be high priority 
    &MotionDetectionHandler, 
    1
  );

  // Image Capturing Task 
  xTaskCreatePinnedToCore(
    ImageCapManager,
    "ImageCapManager",
    8192, 
    NULL, 
    4,  // High priority, below motion 
    &ImageCapHandler, 
    1
  );

  // Image Processing Task 
  xTaskCreatePinnedToCore(
    ImageProcManager,
    "ImageProcManager",
    4096, 
    NULL,
    1, 
    &ImageProcHandler, 
    1
  );

  // BLE Task 
  xTaskCreatePinnedToCore(
    BLEManager,
    "BLEManager",
    8192, 
    NULL,
    2,  
    &BLEHandler, 
    0
  );

  // RTC Task 
  xTaskCreatePinnedToCore(
    RTCManager,
    "RTCManager",
    2048, 
    NULL,
    1,
    &RTCHandler, 
    1
  );

  // Power Task 
  xTaskCreatePinnedToCore(
    PowerManager,
    "Poweranager",
    4096, 
    NULL,
    2, 
    &PowerHandler, 
    1
  );

  // Battery Task 
  xTaskCreatePinnedToCore(
    BatteryManager,
    "BatteryManager",
    2048, 
    NULL,
    1,
    &BatteryHandler, 
    1
  );

  //OTA Task 
  xTaskCreatePinnedToCore(
    OTAManager,
    "OTAManager",
    8192, 
    NULL,
    3,  
    &OTAHandler, 
    0
  );  
}

void loop() {
  vTaskDelay(portMAX_DELAY); 
}