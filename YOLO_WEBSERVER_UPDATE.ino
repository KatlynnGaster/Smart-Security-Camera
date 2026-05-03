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

#include <WiFi.h>

#include "SparkFunBME280.h"
#include "SparkFunCCS811.h"

#include <WebServer.h>

WebServer server(80);

// WiFi 
const char* ssid = "Katlynn_Galaxy22";
const char* password = "katlynn04";


// RTC 
RTC_DS1307 rtc;
// Second I2C bus on different pins
TwoWire I2C_RTC = TwoWire(1);  // Bus number 1

// ENV
#define CCS811_ADDR 0x5B

BME280 bme;               // env sensor
CCS811 ccs(CCS811_ADDR);  // env sensor

// Pins
#define PIR_PIN 13
#define CS_PIN 15
#define CAM_PWR 33
//#define CAM_POWER 4
ArduCAM myCAM(OV2640, CS_PIN);

// Globals
volatile bool motionDetected = false;
volatile int intrusionCount = 0; 

// Mutexs
SemaphoreHandle_t timeMutex = NULL;
SemaphoreHandle_t spiffsMutex = NULL;
SemaphoreHandle_t envMutex = NULL;

float batteryVolt = 0;
String currentTime = ""; 

float temperature = 0;
float humidity = 0;
float co2 = 0;

// LED Variables 
#define WHITE_LED 2 
#define RED_LED 4

volatile bool motionState = false;
volatile bool intrusionActive = false;
unsigned long lastMotionTime = 0;
const unsigned long MOTION_TIMEOUT = 5000; 
unsigned long intrusionStartTime = 0;

// Task Handlers 
TaskHandle_t MotionDetectionHandler = NULL;
TaskHandle_t ImageCapHandler = NULL;
TaskHandle_t ImageProcHandler = NULL;
TaskHandle_t RTCHandler = NULL;
TaskHandle_t PowerHandler = NULL;
TaskHandle_t EnvHandler = NULL;
TaskHandle_t BatteryHandler = NULL;

// Init camera function
void initCamera() {

  myCAM.write_reg(ARDUCHIP_FIFO, 0x00); // ensure FIFO idle
  myCAM.clear_fifo_flag();
  myCAM.flush_fifo();
  delay(100);

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

void cameraHardReset() {
  Serial.println("Hard resetting camera state...");

  // Ensure CS is safe
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  delay(50);

  // Reset SPI completely
  SPI.end();
  delay(200);

  SPI.begin(18, 19, 23, CS_PIN);
  SPI.setFrequency(400000); // slow for stability after wake
  delay(200);

  // Power cycle camera
  digitalWrite(CAM_PWR, LOW);
  delay(300);
  digitalWrite(CAM_PWR, HIGH);
  delay(1500);

  // Flush any ghost state
  for (int i = 0; i < 3; i++) {
    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    delay(100);
  }
}

void cameraWake(){
  
  Serial.println("Waking camera");
  cameraHardReset();

  // POWER STABILIZATION
  digitalWrite(CAM_PWR, HIGH);
  delay(2000);  // critical after hard power cycle

  // CLEAN SPI RESTART
  SPI.end();
  delay(300);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  delay(100);

  SPI.begin(18, 19, 23, CS_PIN);

  // slow SPI after cold boot
  SPI.setFrequency(400000);
  delay(300);

  // re-sync bus timing after power loss
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  delay(50);
  myCAM.write_reg(ARDUCHIP_TEST1, 0xAA);
  delay(50);

  // BASIC CAMERA REINIT
  initCamera();
  delay(2500);  // sensor + PLL settle time

  // SENSOR WAKE COMMAND
  myCAM.wrSensorReg8_8(0xFF, 0x01);
  myCAM.wrSensorReg8_8(0x09, 0x00);
  delay(300);


  // DUMMY CAPTURE WARMUP
  for(int i = 0; i < 4; i++){
    myCAM.flush_fifo();
    myCAM.clear_fifo_flag();
    myCAM.start_capture();

    unsigned long start = millis();
    while(!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)){
      if(millis() - start > 4000) break;
      delay(10);
    }

    delay(200);
  }

  Serial.println("Camera stabilized after wake");
}

void cameraSleep(){
  Serial.println("Putting camera into standby");
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.wrSensorReg8_8(0xFF, 0x01);
  myCAM.wrSensorReg8_8(0x09, 0x10);
  digitalWrite(CS_PIN, HIGH);

  // digitalWrite(CAM_PWR, LOW);   // PN2222 cuts ground (camera off)
  delay(50);  
  // Stop clocking FIFO activity
  myCAM.write_reg(ARDUCHIP_FIFO, 0x01); // stop FIFO write (safe idle state)

}

void handleRoot(){
  // building the webserver page here in html
  // display the intrustion count, battery, time, CO2, humidity, temp
  // display the images of intruders/when there is motion
  // alert message for intruders 

  String timeCopy;
  if(xSemaphoreTake(timeMutex, portMAX_DELAY)){
    timeCopy = currentTime;
    xSemaphoreGive(timeMutex);
  }

  float batt = batteryVolt;
  int intrusions = intrusionCount;

  // HTML PAGE

  String html = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <meta http-equiv="refresh" content="5">

    <style>
      body {
        font-family: Arial;
        background-color: #ffffff;
        color: #222;
        text-align: center;
      }

      h1 {
        color: #ff4fa3;
      }

      .card {
        background: #f7fff9;
        padding: 15px;
        margin: 10px;
        border-radius: 14px;
        box-shadow: 0 2px 10px rgba(0,0,0,0.08);
        border: 1px solid #d7f5df;
        transition: transform 0.2s ease, box-shadow 0.2s ease;
      }

      .card:hover {
        transform: scale(1.03);
        box-shadow: 0 6px 18px rgba(0,0,0,0.15);
      }

      .grid {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
        gap: 10px;
      }

      img {
        width: 90%;
        border-radius: 10px;
        margin-top: 10px;
        border: 2px solid #ffb6d9;
      }

      .alert {
        background: #ff4fa3;
        color: white;
        padding: 10px;
        font-weight: bold;
        border-radius: 10px;
        margin: 10px;
      }

      p {
        color: #2f6f3e;
        font-weight: 500;
      }
    </style>
  </head>

  <body>

    <h1>Kat Security Cam</h1>
  )rawliteral";

  // ALERT 
  if(intrusionActive){
    html += "<div class='alert'>INTRUSION DETECTED</div>";
  }

  float t, h, c;

  if(xSemaphoreTake(envMutex, portMAX_DELAY)){
    t = temperature;
    h = humidity;
    c = co2;
    xSemaphoreGive(envMutex);
  }

  // STATS 
  html += "<div class='grid'>";

  html += "<div class='card'><h3>Intrusions</h3><p>" + String(intrusions) + "</p></div>";
  html += "<div class='card'><h3>Battery</h3><p>" + String(batt, 2) + " V</p></div>";
  html += "<div class='card'><h3>Time</h3><p>" + timeCopy + "</p></div>";

  html += "<div class='card'><h3>Temp</h3><p>" + String(t,1) + "&deg;C</p></div>";
  html += "<div class='card'><h3>Humidity</h3><p>" + String(h,1) + "%</p></div>";
  html += "<div class='card'><h3>CO2</h3><p>" + String(c,0) + " ppm</p></div>";

  html += "</div>";

  // IMAGE 
  html += "<div class='card'>";
  html += "<h3>AI Detection Feed</h3>";
  html += "<img src='/annotated.jpg?rand=" + String(millis()) + "'>";  html += "</div>";
  html += R"rawliteral(
  </body>
  </html>
  )rawliteral";

  server.send(200, "text/html; charset=utf-8", html);
}

void handleAnnotated() {
  if(xSemaphoreTake(spiffsMutex, portMAX_DELAY)) {
    File file = SPIFFS.open("/annotated.jpg", FILE_READ);

    if(!file) {
      xSemaphoreGive(spiffsMutex);
      server.send(404, "text/plain", "No image");
      return;
    }

    server.streamFile(file, "image/jpeg");
    file.close();
    xSemaphoreGive(spiffsMutex);
  }
}

void handleImage() {
  if(xSemaphoreTake(spiffsMutex, portMAX_DELAY)){
    File file = SPIFFS.open("/capture.jpg", FILE_READ);

    if(!file){
      xSemaphoreGive(spiffsMutex);
      server.send(404, "text/plain", "No image");
      return;
    }

    server.streamFile(file, "image/jpeg");
    file.close();
    xSemaphoreGive(spiffsMutex);
  }
}

// Add YOLO to check if there was a person or not in py script
// If person == intrusion; else == no intrusion 
bool detectIntrusion() {
  WiFiClient client;

  if (!client.connect("10.203.169.215", 5000)) {
    Serial.println("YOLO server down");
    return false;
  }

  bool personDetected = false;

  // SEND IMAGE
  if (xSemaphoreTake(spiffsMutex, portMAX_DELAY)) {

    File file = SPIFFS.open("/capture.jpg", FILE_READ);
    if (!file) {
      xSemaphoreGive(spiffsMutex);
      return false;
    }

    client.println("POST /detect HTTP/1.1");
    client.println("Host: 10.203.169.215");
    client.println("Content-Type: image/jpeg");
    client.print("Content-Length: ");
    client.println(file.size());
    client.println();

    uint8_t buf[256];
    while (file.available()) {
      size_t n = file.read(buf, sizeof(buf));
      client.write(buf, n);
      delay(1);
    }

    file.close();
    xSemaphoreGive(spiffsMutex);
  }

  // READ RESPONSE HEADERS 
  unsigned long start = millis();

  while (millis() - start < 3000 && (client.connected() || client.available())) {
    String line = client.readStringUntil('\n');

    if (line == "\r") break;

    if (line.indexOf("X-Person: 1") >= 0) {
      personDetected = true;
    }
  }

  // SAVE IMAGE
  if (xSemaphoreTake(spiffsMutex, portMAX_DELAY)) {

    File out = SPIFFS.open("/annotated.jpg", FILE_WRITE);
    if (out) {

      unsigned long imgStart = millis();

      while ((millis() - imgStart < 3000) && client.connected()) {
        while (client.available()) {
          out.write(client.read());
        }
        delay(1);
      }
      out.close();
    }
    xSemaphoreGive(spiffsMutex);
  }
  return personDetected;
}

void MotionDetectionManager(void *parameters){
  // Motion detection 
  pinMode(PIR_PIN, INPUT);
  unsigned long lastTrigger = 0;

  while(true){
    if(digitalRead(PIR_PIN) == HIGH && millis() - lastTrigger > 5000){
      Serial.println("Motion detected");
      motionDetected = true;
      motionState = true;
      lastTrigger = millis();
      lastMotionTime = millis();
      xTaskNotifyGive(ImageCapHandler);
    }
    if (digitalRead(PIR_PIN) == LOW) {
      motionState = false;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void ImageCapManager(void *parameters){
  while(true){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("Capturing Image");

    bool captureOK = false;
    uint32_t length = 0;

    // CAPTURE RETRY LOOP
    for(int attempt = 0; attempt < 3; attempt++){

      myCAM.flush_fifo();
      myCAM.clear_fifo_flag();
      myCAM.start_capture();

      unsigned long start = millis();
      while(!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)){
        if(millis() - start > 4000) break;
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }

      if(myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)){
        captureOK = true;
        break;
      }

      Serial.println("Capture retry...");
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }

    // FAIL SAFE
    if(!captureOK){
      Serial.println("Capture failed after retries");
      myCAM.clear_fifo_flag();
      continue;
    }

    length = myCAM.read_fifo_length();

    Serial.print("Image size (bytes): ");
    Serial.println(length);

    // VALIDATION FILTER
    if(length == 0 || length > 500000 || length == 0x7FFFFF){
      Serial.println("Invalid FIFO detected → resetting camera");

      cameraHardReset();
      initCamera();
      continue;
    }

    // SAVE IMAGE
    bool savedOK = false;

    if(xSemaphoreTake(spiffsMutex, portMAX_DELAY)){

      File file = SPIFFS.open("/capture.jpg", FILE_WRITE);

      if(file){

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
        savedOK = true;
      }

      xSemaphoreGive(spiffsMutex);
    }

    myCAM.clear_fifo_flag();

    // ONLY NOTIFY IF SUCCESSFUL
    if(savedOK){
      Serial.println("Image saved → notifying processor");
      xTaskNotifyGive(ImageProcHandler);
    } else {
      Serial.println("Image save failed");
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void ImageProcManager(void *parameters){
  // Image processing -> recognition functions
  while(true){
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("Processing Image");

    // Intrusion count
    Serial.print("Free heap: ");
    Serial.println(ESP.getFreeHeap());

    bool intrusion = detectIntrusion();

    Serial.print("YOLO intrusion result: ");
    Serial.println(intrusion);

    if (intrusion) {

      intrusionCount++;

      intrusionActive = true;
      intrusionStartTime = millis();

      Serial.print("INTRUSION COUNT UPDATED: ");
      Serial.println(intrusionCount);

      Serial.println("INTRUSION DETECTED (PERSON CONFIRMED)");
    }
    if (intrusionActive && (millis() - intrusionStartTime > 5000)) {
      intrusionActive = false;
    }
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
      vTaskSuspend(RTCHandler);
      vTaskSuspend(BatteryHandler);
      vTaskSuspend(EnvHandler);
      
      // Set wakeup sources right before sleep
      esp_sleep_enable_ext0_wakeup((gpio_num_t)PIR_PIN, 1);
      esp_sleep_enable_timer_wakeup(60000000ULL); // 60 seconds

      cameraSleep(); // turn camera off 
      
      // Disconnect Wifi when it goes to sleep for power 
      WiFi.disconnect(true);
      WiFi.mode(WIFI_OFF);
      Serial.println("WiFi off"); 

      ccs.setDriveMode(0);  // idle

      Serial.println("Entering light sleep...");
      Serial.flush();        // wait for serial to finish transmitting
      delay(100);            // give it extra time 

      esp_light_sleep_start();

      cameraWake(); // turn camera back on after it wakes

      // Reconnect WiFi after wake
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      unsigned long wifiStart = millis();
      while(WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000){
        delay(200);
      }
      if(WiFi.status() == WL_CONNECTED){
        Serial.println("WiFi reconnected");
      } else {
        Serial.println("WiFi reconnect failed");
      }

      ccs.setDriveMode(1);  // 1s sampling (or 2/10 sec mode)

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
      vTaskResume(RTCHandler);
      vTaskResume(BatteryHandler);
      vTaskResume(EnvHandler);
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

void EnvManager(void *parameters){
  while(true){

    float t = bme.readTempC();
    float h = bme.readFloatHumidity();
    float c = 0;

    if (ccs.dataAvailable()) {
      ccs.readAlgorithmResults();
      c = ccs.getCO2();
    } else {
      Serial.println("CCS811 not ready...");
    }

    // STORE SAFELY 
    if(xSemaphoreTake(envMutex, portMAX_DELAY)){
      temperature = t;
      humidity = h;
      co2 = c;
      xSemaphoreGive(envMutex);
    }
  vTaskDelay(2000 / portTICK_PERIOD_MS); // every 2 sec
  }
}

void setup() {
  Serial.begin(115200);
  
  SPI.begin(18, 19, 23, CS_PIN);
  SPI.setFrequency(1000000);  // 1 MHz
  delay(200);

  Wire.begin();
  delay(200);

  pinMode(WHITE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);

  digitalWrite(WHITE_LED, LOW);
  digitalWrite(RED_LED, LOW);

  // pinMode(CAM_PWR, OUTPUT);
  // digitalWrite(CAM_PWR, HIGH);
  // delay(1000); 

  // initialize camera
  initCamera();
  delay(3000);

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

  // Mutex makes sure only one task can access something at a time 
  timeMutex = xSemaphoreCreateMutex(); // makes sure the web server reads a clean currentTime
  spiffsMutex = xSemaphoreCreateMutex(); // makes sure only one task can access the images so they arent being corrupted 
  envMutex = xSemaphoreCreateMutex(); // makes sure the sensor values match the same moment 

  // Initialize BME280
  if (!bme.begin()) {
    Serial.println("BME280 not detected!");
  } else {
    Serial.println("BME280 initialized!");
  }
  // Initialize CCS
  if (!ccs.begin()) {
    Serial.println("CCS811 not detected!");
  } else {
    Serial.println("CCS811 initialized!");
  }

  // Wait a bit for CCS811 to stabilize
  delay(1000);

  // Webserver 
  server.on("/", handleRoot);
  server.on("/image", handleImage);
  server.on("/annotated.jpg", handleAnnotated);

  server.begin();

  // Motion detection task 
  xTaskCreatePinnedToCore(
    MotionDetectionManager,
    "MotionDetectionManager",
    4096, 
    NULL,
    5, // want this to be high priority 
    &MotionDetectionHandler, 
    0
  );

  // Image Capturing Task 
  xTaskCreatePinnedToCore(
    ImageCapManager,
    "ImageCapManager",
    12288, 
    NULL, 
    4,  // High priority, below motion 
    &ImageCapHandler, 
    0
  );

  // Image Processing Task 
  xTaskCreatePinnedToCore(
    ImageProcManager,
    "ImageProcManager",
    12288, 
    NULL,
    5, 
    &ImageProcHandler, 
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

  // Env Sensor Task 
  xTaskCreatePinnedToCore(
    EnvManager,
    "EnvManager",
    2048, 
    NULL,
    1,
    &EnvHandler, 
    1
  ); 
}

void loop() {
  server.handleClient();

  // WHITE LED = motion 
  digitalWrite(WHITE_LED, (millis() - lastMotionTime < MOTION_TIMEOUT) ? HIGH : LOW);
  // RED LED = intrusion 
  digitalWrite(RED_LED, intrusionActive ? HIGH : LOW);

  // turn off intrusion after 5 seconds
  if (intrusionActive && (millis() - intrusionStartTime > 5000)) {
    intrusionActive = false;
  }
}
