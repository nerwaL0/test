#define BLYNK_AUTH_TOKEN "VMwyM2cKI5_QcqlMCMZIielA_gnXsI5-"
#define BLYNK_TEMPLATE_ID "TMPL6sUD1_VVQ"
#define BLYNK_TEMPLATE_NAME "SmartRemoteAC"
#define BLYNK_PRINT Serial

#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <driver/i2s.h>
#include <Blynk/BlynkApi.h>
#include "Blynk/BlynkConfig.h"
#include "Blynk/BlynkApi.h"

// Pin definitions for INMP441
#define I2S_WS 32   // Word Select (WS) pin
#define I2S_SCK 33  // Serial Clock (SCK) pin
#define I2S_SD 34   // Serial Data (SD) pin

//wifi credentials
const char* ssid = "jejeje";
const char* password = "gataubro";

// IR Codes (replace these with your actual AC IR codes)
unsigned long acOn = 0x10EFD200;  
unsigned long acOff = 0x10EFD100;  
unsigned long tempUp = 0x10EF900;  
unsigned long tempDown = 0x10EF800;

// IR and sensor setup
IRsend irsend(14);  // GPIO 13 for IR sending
IRrecv irrecv(13);  // GPIO 15 for IR receiving (VS1838 connected to GPIO 15)
decode_results results;  // To store the received IR data
DHT dht(4, DHT22);  // DHT22 sensor on pin 4

unsigned long lastIRCode = 0; // To store the last IR code received
unsigned long debounceDelay = 200; // Debounce delay for IR signals
BlynkTimer timer;
// I2S Configuration
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,  // 16 kHz sample rate
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,  // Updated to use the recommended format
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  // Install and start I2S driver
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}

// Function to read audio samples and detect loud sounds
bool detectLoudSound() {
  uint32_t buffer[1024]; // Buffer to store audio data
  size_t bytesRead = 0;
  int32_t sum = 0;

  // Read data from the I2S microphone
  i2s_read(I2S_NUM_0, &buffer, sizeof(buffer), &bytesRead, portMAX_DELAY);

  // Calculate the average magnitude
  for (size_t i = 0; i < bytesRead / sizeof(uint32_t); i++) {
    int32_t sample = buffer[i] >> 14; // Downscale 32-bit to 16-bit equivalent
    sum += abs(sample);
  }

  // Compute average magnitude
  int32_t avgMagnitude = sum / (bytesRead / sizeof(uint32_t));

  // If the average magnitude exceeds a threshold, return true
  const int threshold = 2000; // Adjust this value as needed
  return avgMagnitude > threshold;
}

// // This function sends sound value to Virtual Pin 1
// void sendSoundData() {
//   soundValue = analogRead(soundPin);  // Read the sound sensor
//   Blynk.virtualWrite(V1, soundValue);  // Send the sound value to Blynk App (Virtual Pin V1)
  
//   // If sound exceeds threshold, send IR command to turn on AC
//   if (soundValue > threshold) {
//     irsend.sendNEC(acOn, 32);  // Send IR code for AC ON
//     Serial.println("Loud sound detected, AC turned ON!");
//     Blynk.virtualWrite(V3, "AC ON");  // Update Blynk button or widget status to show AC ON
//   }
//   else {
//     Blynk.virtualWrite(V3, "AC OFF");  // Update Blynk button or widget status to show AC OFF
//   }
// }

// This function sends temperature data to Virtual Pin 2
void sendTemperatureData() {
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  // Check if the sensor readings are valid
  if (isnan(temp) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  // Print the temperature and humidity to the Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.print(" °C, Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  
  // Send temperature and humidity to Virtual Pins on Blynk
  Blynk.virtualWrite(V2, temp);     // Send temperature data to Virtual Pin V2
  Blynk.virtualWrite(V4, humidity); // Send humidity data to Virtual Pin V4
}

// Function to handle IR reception
void handleIRReception() {
  if (irrecv.decode(&results)) {  // Check if an IR signal is received
    unsigned long irCode = results.value;  // Get the IR code received
    irrecv.resume();  // Receive the next value

    // Check if the IR code is different from the last one received and debounce
    if (irCode != lastIRCode || millis() - lastIRCode > debounceDelay) {
      Serial.print("IR Code received: ");
      Serial.println(irCode, HEX);  // Print the IR code in hex format
      lastIRCode = irCode;  // Update the last received IR code

      // Check the received code and map it to corresponding actions
      if (irCode == acOn) {
        Serial.println("AC Turned ON via IR");
        Blynk.virtualWrite(V3, "AC ON");
      } 
      else if (irCode == acOff) {
        Serial.println("AC Turned OFF via IR");
        Blynk.virtualWrite(V3, "AC OFF");
      } 
      else if (irCode == tempUp) {
        Serial.println("Temperature Up command received");
        irsend.sendNEC(tempUp, 32);  // Send the tempUp IR command to the AC
        Blynk.virtualWrite(V3, "Temp Up");
      } 
      else if (irCode == tempDown) {
        Serial.println("Temperature Down command received");
        irsend.sendNEC(tempDown, 32);  // Send the tempDown IR command to the AC
        Blynk.virtualWrite(V3, "Temp Down");
      }
    }
  }
}

void setup() {
  // Start serial communication for debugging
  Serial.begin(115200);

  // Set up WiFi connection
  WiFi.begin(ssid, password);
  
  // Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Initialize IR send, receive, DHT sensor, and I2S
  irsend.begin();
  irrecv.enableIRIn();  // Start the IR receiver
  dht.begin();
  setupI2S();  // Set up the I2S microphone

  // Initialize Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  // Set a timer to send temperature data every 2 seconds
  timer.setInterval(2000L, sendTemperatureData);  // Every 2 seconds, read and send temperature
}

void loop() {
  Blynk.run();  // Run Blynk tasks
  timer.run();  // Run the timer

  // Detect loud sound and send IR command if threshold exceeded
  if (detectLoudSound()) {
    irsend.sendNEC(acOn, 32);  // Send IR code for AC ON
    Serial.println("Loud sound detected, AC turned ON!");
    Blynk.virtualWrite(V3, "AC ON");  // Update Blynk button or widget status to show AC ON
  }

  handleIRReception();  // Check and handle incoming IR signals
}