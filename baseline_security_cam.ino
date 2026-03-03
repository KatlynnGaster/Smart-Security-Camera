// Code by: Katlynn Gaster 

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h> // For web server
#include <WebServer.h>
#include <SPI.h>
#include <FS.h>
#include <SD.h>
#include <iostream> 
#include <vector>
#include <string>
#include "memorysaver.h"
#include "FS.h"
#include "SPIFFS.h"
#include "esp_bt.h"
#include <ArduCAM.h>
#undef swap

WebServer server(80);

const char* ssid = "ESP32-CAM";
const char* password = "12345678";
bool maintenanceMode = false;

#define PIR_PIN 13
#define BUTTON_PIN 14
#define CS_PIN 5
ArduCAM myCAM(OV2640, CS_PIN);

// Counter for the amount of pictures 
RTC_DATA_ATTR int photoCounter = 0;

// Countdown timers
RTC_DATA_ATTR uint32_t minuteCounter = 0;
RTC_DATA_ATTR uint64_t lastCaptureMinute = UINT32_MAX;

// Making the pir motion after web server capture image if motion
RTC_DATA_ATTR bool forceNextCapture = false;

// web server function
void startWebServer() {
  WiFi.softAP(ssid, password);
    Serial.print("AP IP address: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, []() {
      String html = "<h1>ESP32-CAM Photos</h1><ul>";

      File root = SPIFFS.open("/");
      File file = root.openNextFile();

      while (file) {
          if (!file.isDirectory()) {
              String name = file.name();
              html += "<li><a href=\"" + name + "\">" + name + "</a></li>";
          }
          file = root.openNextFile();
      }

      html += "</ul>";
      server.send(200, "text/html", html);
    });

      server.onNotFound([]() {
        String path = server.uri();
        if (SPIFFS.exists(path)) {
            File f = SPIFFS.open(path, FILE_READ);
            server.streamFile(f, "image/jpeg");
            f.close();
        } else {
            server.send(404, "text/plain", "Not found");
        }
      });

    server.begin();
    Serial.println("Web server running.");

    unsigned long start = millis();
    const unsigned long duration = 60UL * 1000UL; // 2 minutes

    while (millis() - start < duration) {
        server.handleClient();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    Serial.println("Maintenance window expired. Going back to sleep...");
    delay(100);

    forceNextCapture = true; 

    // Re-enable wake sources before sleeping again
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIR_PIN, 1);
    esp_sleep_enable_ext1_wakeup((1ULL << BUTTON_PIN), ESP_EXT1_WAKEUP_ALL_LOW);
    esp_sleep_enable_timer_wakeup(60000000ULL);

    // Go back to deep sleep
    esp_deep_sleep_start();
}


// Init camera function
void initCamera() {
  Wire.begin(21, 22);
  SPI.begin(18, 19, 23, CS_PIN);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // Test SPI communication
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  if (myCAM.read_reg(ARDUCHIP_TEST1) != 0x55) {
    Serial.println("SPI communication failed!");
    return;
  }

  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  myCAM.OV2640_set_JPEG_size(OV2640_640x480);

  Serial.println("Camera initialized.");
}

// Capture picture function 
void capturePhoto() {
  Serial.println("Capturing image...");

  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();

  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));

  Serial.print("Image size (bytes): ");
  Serial.println(myCAM.read_fifo_length());
}

bool savePhotoToSPIFFS(const char* filename) {
  uint32_t length = myCAM.read_fifo_length();

  if (length == 0) {
    Serial.println("No image captured!");
    return false;
  }

  Serial.print("Saving image of size ");
  Serial.print(length);
  Serial.print(" bytes to ");
  Serial.println(filename);

  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file in SPIFFS");
    return false;
  }

  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  uint8_t buf[256];
  uint32_t bytesRemaining = length;

  while(bytesRemaining) {
    uint32_t bytesToRead = (bytesRemaining > 256) ? 256 : bytesRemaining;
    SPI.transferBytes(0, buf, bytesToRead);
    file.write(buf, bytesToRead);
    bytesRemaining -= bytesToRead;
  }

  myCAM.CS_HIGH();
  file.close();

  Serial.println("Image saved successfully!");
  return true;
}


void printWakeReason() {
  esp_sleep_wakeup_cause_t reason = esp_sleep_get_wakeup_cause();
  
  Serial.print("Wake reason: ");

  switch (reason) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("PIR motion detected");

      
      if (forceNextCapture) {
        Serial.println("Post-maintenance capture.");
        forceNextCapture = false;   // reset flag

        initCamera();
        capturePhoto();

        char filename[32];
        sprintf(filename, "/photo%d.jpg", photoCounter++);
        savePhotoToSPIFFS(filename);

        lastCaptureMinute = minuteCounter;
      }
      else if (minuteCounter - lastCaptureMinute >= 1) {
        Serial.println("Cooldown expired. Capturing image.");
        initCamera();
        capturePhoto();

        char filename[32];
        sprintf(filename, "/photo%d.jpg", photoCounter++);
        savePhotoToSPIFFS(filename);

        lastCaptureMinute = minuteCounter;
      } else {
        Serial.println("Cooldown active: ignoring motion");
      }
      break;
    
    case ESP_SLEEP_WAKEUP_EXT1:
      maintenanceMode = true;
      Serial.println("Button pressed: Maintenance mode activated");
      break;

    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("RTC Timer wake");
      minuteCounter++;
      break;

    default:
      Serial.println("Normal boot");
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  if(!SPIFFS.begin(true)){ 
    Serial.println("SPIFFS Mount Failed");
    return;
  }

  pinMode(PIR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  printWakeReason();

  if(maintenanceMode){
    startWebServer();
    return;
  }

  // Disable wifi and bluetooth 
  WiFi.mode(WIFI_OFF);
  btStop();

  // RTC Wake 
  esp_sleep_enable_timer_wakeup(60000000ULL);
  
  // PIR Motion Wake 
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIR_PIN, 1);

  // Maintenance button wake
  esp_sleep_enable_ext1_wakeup((1ULL << BUTTON_PIN), ESP_EXT1_WAKEUP_ALL_LOW);

  // Wait for PIR to go LOW so we don't instantly retrigger
  if (digitalRead(PIR_PIN) == HIGH) {
    Serial.println("Waiting for PIR to reset...");
    delay(50);  // allow sensor to go LOW
  }

  Serial.println("Entering deep sleep...");
  delay(100);

  esp_deep_sleep_start();
}

void loop() {}

// keeping in mind the camera is always on and using power
// When the web server is running it is around 0.3A
// When the ESP32 is in deep sleep, it is around 0.183A
// When the ESP32 is on it is around 0.224A - 0.250A
