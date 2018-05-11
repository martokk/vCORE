/*
vCORE Universal IoT Device v1.4.28
  - mike.villarreal@outlook.com

*/

// #define FASTLED_ALLOW_INTERRUPTS 0
// #define FASTLED_INTERRUPT_RETRY_COUNT 1
// #define FASTLED_INTERRUPT_RETRY_COUNT 0

/************ INCLUDES ****************/
#include <Arduino.h>          // Required. Using .cpp file insteal .ino
#include "config.h"           // Required. Device Configureation File (Edit before final build)
#include <ESP8266WiFi.h>      // Required. WiFi
#include <ArduinoOTA.h>       // Required. OTA Updates via PlatformIO (platformio.ini)
#include <ESP8266mDNS.h>      // UNKNOWN! // TODO: Reasearch if ESP8266mDNS.h is needed
#include <PubSubClient.h>     // Required. MQTTT
#include <ArduinoJson.h>      // Required. JSON
#include <FastLED.h>          // Required. LED Strips
#include <WS2812FX.h>         // Required. LED Strips
#include <Adafruit_Sensor.h>  // Optional. DHT Sensors
#include <DHT.h>              // Optional. DHT Sensors
#include <IRsend.h>           // Optional. IR Transmitter

/************ DEFINE FUNCITONS ****************/
void SetupWifi();
void SetupOta();
void ReconnectMqtt();
void MqttCallback(char* topic, byte* payload, unsigned int length);
bool ProcessMqttJson(char* message);
void ProcessColorPalette();
void ConvertTempToRGB(unsigned int kelvin);
void SendMqttState();
void ShowLedState();

void CheckDeviceReset();
void CheckButtons();
void CheckDht();
void CheckPir();

uint16_t CustomEffect_fillnoise8();
uint16_t CustomEffect_bpm();
void Effect_fillnoise8();
void Effect_bpm();

/************ SET VARIABLES ****************/
// WIFI, MQTT, JSON
WiFiClient espClient;
PubSubClient client(espClient);
const int BUFFER_SIZE = JSON_OBJECT_SIZE(20);

// JSON States
bool stateOn;
int r_color = -1;
int g_color = -1;
int b_color = -1;
int color_temp = -1;
int brightness = -1;
int speed = 5000;
int transitionTime;
int effect = -1;
const char* effect_char;
String effect_string;
const char* palette_char;
String palette;
bool paletteOn = false;

// LED Strips
WS2812FX led1 = WS2812FX(LED1_COUNT, LED1_PIN, NEO_RGB + NEO_KHZ800);
CRGB leds[LED1_COUNT];
CRGBPalette16 currentPalette;
CRGBPalette16 targetPalette;
static uint16_t x = random16();
static uint16_t y = random16();
static uint16_t z = random16();
uint8_t maxPaletteBlendChanges = 48;
uint8_t colorLoop = 1;
uint8_t noise[LED1_COUNT];
uint16_t fastled_speed;
uint16_t scale;
static uint16_t dist = random(12345);
uint8_t gHue = 0;

// Buttons
int reset_button_state = 0;
bool device_reset = true;
bool device_ready = false;
int button_loop_interval = 0;

// PIR
bool pir1_motion_state = false;
bool pir2_motion_state = false;
bool pir3_motion_state = false;
int pir_state = 0;
int mqtt_rearm_time = 0;
int pir1_rearm_time = 0;
int pir3_rearm_time = 0;
int pir_interval_time = 0;

// DHT
DHT dht(DHT_PIN, DHT_TYPE);
int dht_interval_time = 0;
bool dht_reconnect = true;
float dht_rolling_temperature_total = 0;
float dht_rolling_humidity_total = 0;
int dht_rolling_counter = 0;
int dht_rolling_counter_max = DHT_MQTT_PUB_INTERVAL / DHT_INTERVAL_LOOP;
float dht_average_temperature = 0;
float dht_average_humidity = 0;

// IR TRANSMITTER
IRsend irsend(IR_TRANSMITTER1_PIN);





/******************************************************************************/





/************ SETUP & MAIN LOOP ****************/
void setup() {
  // Start Serial
  // Serial.begin(115200);
  // Serial.println();
  // Serial.println();
  // Serial.println("Device: Booting");

  // Initialize Reset Button
  if (TOTAL_RESET_BUTTONS > 0 ) {
    pinMode(RESET_BUTTON_PIN, INPUT);
  }

  // Start WIFI
  SetupWifi();

  // Start OTA
  SetupOta();

  // Start MQTT
  client.setServer(MQTT_SERVER, 1883); //CHANGE PORT HERE IF NEEDED
  client.setCallback(MqttCallback);

  // Start PIR
  switch(TOTAL_PIR_SENSORS) {
    case 0:
      break;
    case 1:
      pinMode(PIR1_PIN, INPUT);
      break;
    case 2:
      pinMode(PIR2_PIN, INPUT);
      break;
    case 3:
      pinMode(PIR3_PIN, INPUT);
      break;
  }

  // Start DHT
  if (TOTAL_DHT_SENSORS > 0) {
    dht.begin();
  }

  // Start IR Transmitter
  if (TOTAL_IR_TRANSMITTERS > 0) {
      irsend.begin();
  }

}

void SetupWifi() {
  delay(10);
  // Serial.println();
  // Serial.print("WiFi: ");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
WiFi.setSleepMode(WIFI_NONE_SLEEP);
  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  // Serial.print("Connected to ");
  // Serial.println(WIFI_SSID);
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());
}

void ReconnectMqtt() {
  // Loop until we're reconnected
  int retry = 0;
  while (!client.connected()) {
    ArduinoOTA.handle();
    // Serial.print("MQTT: ");
    // Attempt to connect
    if (client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS)) {
      // Serial.println("Connected");

      // client.publish(PUB_LED1_POWER, "ON");
      // client.publish(PUB_LED1_COLOR, "255,255,255");

      //send 'Device Restarted' notification to MQTT server
      if (device_reset == true) {
        client.publish(PUB_DEVICE, "restarted");
      }

      // send 'Device Connected' notification to MQTT server
      client.publish(PUB_DEVICE, "connected");
      client.publish(SUB_LED1, "connected");

      // led1.setColor(255,255,255);
      // led1.service();

      // NOTE: Comment out if device keeps restarting.
      client.subscribe(PUB_LED1);
      client.loop();
      client.unsubscribe(PUB_LED1);

      client.subscribe(SUB_LED1);
      client.subscribe(SUB_LED1_SPEED);
      client.subscribe(SUB_LED1_PALETTE);
      client.subscribe(SUB_IR_SEND);

          // Device is now Ready!
      if (device_ready == false) {
        device_ready = true;
        // Serial.println("------------ DEVICE CONNECTED ------------");
        // Serial.println();
      }
      retry = 0;
    } else {
      // Serial.print("FAILED!; rc=");
      // Serial.print(client.state());
      // Serial.print("; retry=");
      // Serial.print(retry);

      if (WiFi.status() != WL_CONNECTED) {
        SetupWifi();
      }
      // led1.setColor(255,0,0);
      // led1.service();

      retry = retry + 1;
      delay(5000);
      if (retry >= 10) {
        // Serial.println("MQTT Failed: Resetting Device");

        // led1.stop();
        // led1.service();

        ESP.reset();
      }
    }
  }
}

void SetupOta() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Set Device/Host Name
  // ArduinoOTA.setHostname(DEVICE_NAME);

  // // Set Password
  // ArduinoOTA.setPassword(PASS); // same as wifi (via credentials.h)

  ArduinoOTA.onStart([]() {
    // Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    // Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    // Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    // Serial.printf("Error[%u]: ", error);
    // if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    // else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    // else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    // else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    // else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  // Serial.println("OTA Update: Connected");
}

void loop() {
  if (!client.connected()) {
    ReconnectMqtt();
  }

  client.loop();

  // Check of OTA Intall Request
  ArduinoOTA.handle();

  // Check  Device Reset Button
  if (TOTAL_RESET_BUTTONS > 0 ) {
    CheckDeviceReset();
  }

  // Check MQTT Button
  if (TOTAL_MQTT_BUTTONS > 0 ) {
    CheckButtons();
  }

  //Check PIR State
  if (TOTAL_PIR_SENSORS > 0) {
    CheckPir();
  }

  // Check DHT
  if (TOTAL_DHT_SENSORS > 0) {
    CheckDht();
  }

  // Run WS8212FX service
  if (TOTAL_LED_STRIPS > 0) {

    led1.service();
  }

  if (WiFi.status() != WL_CONNECTED) {
    SetupWifi();
  }

}


/************ SENSORS / MODULES ****************/
void CheckDeviceReset() {
  reset_button_state = digitalRead(RESET_BUTTON_PIN);
  if (reset_button_state == HIGH ) {
    // Serial.println("Reset Button Pressed. Resetting device.");
    led1.setColor(255,0,255);
    led1.start();
    led1.service();
    delay(200);

    led1.stop();
    led1.service();
    delay(200);

    led1.setColor(255,0,255);
    led1.start();
    led1.service();
    delay(200);

    led1.stop();
    led1.service();
    delay(200);

    led1.setColor(255,0,255);
    led1.start();
    led1.service();
    delay(200);

    led1.stop();
    led1.service();
    ESP.reset();
  }
}

void CheckButtons() {
  int button1_state = LOW;

  button1_state = digitalRead(BUTTON1_PIN);
  if (button1_state == HIGH ) {
    if (millis() > button_loop_interval) {
      // Serial.println("Button1 was pressed!");
      client.publish(PUB_BUTTON1, "1", true);

      delay(50);
      client.publish(PUB_BUTTON1, "0", true);

      button_loop_interval = millis() + BUTTON_LOOP_INTERVAL;
    }
  }
}

void CheckPir() {
  if (millis() > pir_interval_time) {
    pir_state = 0;
    int pir1_state = LOW;
    int pir2_state = LOW;
    int pir3_state = LOW;
    if (TOTAL_PIR_SENSORS > 0) {
      pir1_state = digitalRead(PIR1_PIN);
      pir_state = pir_state + pir1_state;
    }
    if (TOTAL_PIR_SENSORS > 1) {
      pir2_state = digitalRead(PIR2_PIN);
      pir_state = pir_state + pir2_state;
    }
    if (TOTAL_PIR_SENSORS > 2) {
      pir3_state = digitalRead(PIR3_PIN);
      pir_state = pir_state + pir3_state;
    }

    if (pir_state > 0) {

      // Check PIR 1
      if (pir1_state == HIGH || pir2_state == HIGH) {
        if (pir1_motion_state == false) {
          pir1_motion_state = true;

          //IF PIR_MODE = 2, then Trigger LEDs
          if (PIR1_TRIGGER_MODE == 2) {
            if (pir3_motion_state == false && pir3_state == LOW ) {
              if (led1.getMode() != FX_MODE_RAINBOW_CYCLE ) {
                led1.setSpeed(215);
                led1.setBrightness(255);
                led1.setMode(FX_MODE_RAINBOW_CYCLE);
                if (led1.isRunning() == false) {
                  led1.start();
                }
              }
            }
          }

        }

        // Publish MQTT
        if (pir1_motion_state == false || millis() > mqtt_rearm_time) {
          client.publish(PUB_PIR1, "1", true);
          mqtt_rearm_time = millis() + PIR_MQTT_RETRIGGER_DELAY;
        }

        pir1_rearm_time = millis() + PIR1_OFF_DELAY;
      }

      // Check PIR 3
      if (pir3_state == HIGH) {
        if (pir3_motion_state == false ) {
          pir3_motion_state = true;
          client.publish(PUB_PIR1, "1", true);
          client.publish(PUB_PIR3, "1", true);
          mqtt_rearm_time = millis() + PIR_MQTT_RETRIGGER_DELAY;

          //PIR3 Trigger LEDs
          if (PIR3_TRIGGER_MODE == 2) {
            if (led1.getMode() != FX_MODE_STATIC) {
              led1.setColor(150,255,125); //GBR
              led1.setBrightness(255);
              led1.setMode(FX_MODE_STATIC);
              if (led1.isRunning() == false) {
                led1.start();
              }
            }
          }

        }
        if (pir3_motion_state == false || millis() > mqtt_rearm_time) {
          client.publish(PUB_PIR1, "1", true);
          client.publish(PUB_PIR3, "1", true);
          mqtt_rearm_time = millis() + PIR_MQTT_RETRIGGER_DELAY;
        }
        pir1_rearm_time = millis() + PIR1_OFF_DELAY;
        pir3_rearm_time = millis() + PIR3_OFF_DELAY;

      } else {
        if (pir3_motion_state == true && millis() > pir3_rearm_time) {
          pir3_motion_state = false ;

          //PIR1 Trigger LEDs
          if (PIR1_TRIGGER_MODE == 2) {
            led1.setSpeed(215);
            led1.setBrightness(255);
            led1.setMode(FX_MODE_RAINBOW_CYCLE);
            led1.start();
          }

          client.publish(PUB_PIR3, "0", true);
          pir3_rearm_time = millis() ;
          mqtt_rearm_time = millis() + PIR_MQTT_RETRIGGER_DELAY;
        }
      }

    // All PIR = LOW then Reset all
    } else {
      if (millis() > pir1_rearm_time && millis() > pir3_rearm_time && (pir1_motion_state == true || pir3_motion_state == true)) {
        pir1_motion_state = false;
        pir3_motion_state = false;
        client.publish(PUB_PIR3, "0", true);
        client.publish(PUB_PIR1, "0", true);
        pir1_rearm_time = millis() ;
        pir3_rearm_time = millis() ;
      }
    }

    pir_interval_time = millis() + PIR_LOOP_INERVAL;
  }
}

void CheckDht(){
  if (millis() > dht_interval_time) {
    // Get Values from DHT Sensor
    float temperature = dht.readTemperature(true) + DHT_TEMP_OFFSET;    //Fahrenheit
    // float temperature = dht.readTemperature(); + DHT_TEMP_OFFSET        // Celsius
    float humidity = dht.readHumidity();

    // Check if humidy & temperature are availble from DHT
    if (isnan(humidity) || isnan(temperature)) {
      if (dht_reconnect == false) {
        // Serial.println("Failed to read from DHT sensor!");
        client.publish(PUB_DEBUG, "DHT sensor  === OFFLINE ===");
        dht_reconnect = true;
      }
      return;
    }
    if (dht_reconnect == true) {
      client.publish(PUB_DEBUG, "DHT sensor  online");
      dht_reconnect = false;
    }

    // DEBUG
    // Serial.println("DEBUG: " + String(temperature));

    // Add current temp to rolling totals
    dht_rolling_temperature_total = dht_rolling_temperature_total + temperature;
    dht_rolling_humidity_total = dht_rolling_humidity_total + humidity;
    dht_rolling_counter++;

    // Get Average & print to Serial/MQTT
    if (dht_rolling_counter >= dht_rolling_counter_max) {
      dht_average_temperature = dht_rolling_temperature_total / dht_rolling_counter_max;
      dht_average_humidity = dht_rolling_humidity_total / dht_rolling_counter_max;

      client.publish(PUB_TEMPERATURE, String(dht_average_temperature).c_str(), true);
      client.publish(PUB_HUMIDITY, String(dht_average_humidity).c_str(), true);
      // Serial.println("Temperature: " + String(dht_average_temperature) + ";  Humidity: " + String(dht_average_humidity));

      dht_rolling_temperature_total = 0;
      dht_rolling_humidity_total = 0;
      dht_rolling_counter = 0;
    }

    dht_interval_time = millis() + DHT_INTERVAL_LOOP;
  }
}


/************ MQTT ****************/
void MqttCallback(char* topic, byte* payload, unsigned int length) {
  // Serial.print("Message arrived [");
  // Serial.print(topic);
  // Serial.print("] ");
  // client.publish(PUB_DEBUG, topic, true);

  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  // Serial.println(message);


  // Process MQTT Topic - PALETTE
  if (String(topic) == SUB_LED1_PALETTE) {
    palette = String(message);
  }

  // Process MQTT Topic - SPEED
  if (String(topic) == SUB_LED1_SPEED) {
    speed = String(message).toInt();
  }

  // Process JSON
  if (!ProcessMqttJson(message)) {

  }



  // startFade = true;
  // inFade = false; // Kill the current fade

  ShowLedState();
  SendMqttState();
}

bool ProcessMqttJson(char* message) {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.parseObject(message);
  // client.publish(PUB_DEBUG, "Parsing JSON", true);
  if (!root.success()) {
    // Serial.println("parseObject() failed");
    client.publish(PUB_DEBUG, "parseObject() failed", true);
    // client.publish(PUB_DEBUG, String(message).c_str(), true);
    return false;
  }


  // ********** STATE ***********
  if (root.containsKey("state")) {
    if (strcmp(root["state"], "ON") == 0) {
      stateOn = true;
    }
    else if (strcmp(root["state"], "OFF") == 0) {
      stateOn = false;
    }
  }


  // ********** COLOR ***********
  if (root.containsKey("color")) {
    r_color = (int)root["color"]["g"];
    g_color = (int)root["color"]["r"];
    b_color = (int)root["color"]["b"];

    effect = FX_MODE_STATIC;
  }


  // ********** COLOR TEMPERATURE ***********
  if (root.containsKey("color_temp")) {
    //temp comes in as mireds, need to convert to kelvin then to RGB
    color_temp = root["color_temp"];
    unsigned int kelvin  = 1000000 / color_temp;

    ConvertTempToRGB(kelvin);
    effect = FX_MODE_STATIC;
  }


  // ********** BRIGHTNESS ***********
  if (root.containsKey("brightness")) {
    brightness = root["brightness"];
  }

  // ********** SPEED ***********
  if (root.containsKey("speed")) {
    speed = root["speed"];
  }

  // ********** Palette ***********
  if (root.containsKey("palette")) {
    palette_char = root["palette"];
    palette = palette_char;
  }

  // ********** EFFECT ***********
  if (root.containsKey("effect")) {
    for (uint8_t i=0; i <= led1.getModeCount(); i++) {

      if (strcmp(root["effect"], String(led1.getModeName(i)).c_str()) == 0) {
        effect = i;
        effect_char = root["effect"];
        effect_string = effect_char;

      }
    }
  }


  // ********** TRANSITION ***********
  if (root.containsKey("transition")) {
    transitionTime = root["transition"];
  // }
  // else if ( effectString == "solid") {
  //   transitionTime = 0;
  }


  return true;
}

void SendMqttState() {
  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  // Device State
  root["state"] = (stateOn) ? "ON" : "OFF";

  if (stateOn) {
      // Color
      if ( r_color != -1  && g_color != -1 && b_color != -1 ) {
        JsonObject& color = root.createNestedObject("color");
        color["r"] = g_color;
        color["g"] = r_color;
        color["b"] = b_color;
      }

      // Color Temp
      if (color_temp != -1) {
        root["color_temp"] = color_temp;
      }

      // Brightness
      if (brightness != -1) {
        root["brightness"] = brightness;
      }


      // Effect
      if (effect != -1) {
        root["effect"] = led1.getModeName(led1.getMode());
        root["effect_id"] = effect;
      }

      // Publish Speed
      if (speed > 10 ) {
        client.publish(PUB_LED1_SPEED, String(speed).c_str() , true);
        root["speed"] = speed;
      }

      // Publish Color Palette
      if (palette != "") {
        client.publish(PUB_LED1_PALETTE, palette.c_str(), true);
        root["palette"] = palette;
      }
  }

  // Publish to MQTT Server
  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));
  client.publish(PUB_LED1, buffer, true);
}

void ShowLedState() {
  if (stateOn) {

    // Color Palette
    ProcessColorPalette();




    // Brightness
    led1.setBrightness(brightness);

    // Color
    led1.setColor(r_color,g_color,b_color);

    // Color Temp


    // Effect
    switch (effect) {
      case  56: led1.setCustomMode(CustomEffect_fillnoise8); break;
      case  57: led1.setCustomMode(CustomEffect_bpm); break;
    }
    led1.setMode(effect);

    // Speed
    led1.setSpeed(speed);

    // if (paletteOn == true) {
      led1.init();
      led1.start();
    // }


  } else {
    led1.stop();
  }
}


// ********** WS2812FX TO FASTLED HOOKS ***********
uint16_t CustomEffect_fillnoise8() {
  colorLoop = 1;

  // SPEED: Speed determines how fast time moves forward.
  // 1 = very slow moving effect
  // 60 = for something that ends up looking like water.
  fastled_speed = 1;

  // SCALE: Scale determines how far apart the pixels in our noise matrix are.
  // The higher the value of scale, the more "zoomed out" the noise will be.
  // 1 = zoomed in, you'll mostly see solid colors.
  scale = 2000 / LED1_COUNT;

  // Process Color Palette Changes
  ProcessColorPalette();

  // Transition Current Palette Towards Target Palettee
  //
  // Each time that nblendPaletteTowardPalette is called, small changes
  // are made to currentPalette to bring it closer to matching targetPalette.
  // You can control how many changes are made in each call:
  // - the default of 24 is a good balance
  // - meaningful values are 1-48. 1=veeeeeeeery slow, 48=quickest
  // - "0" means do not change the currentPalette at all; freeze
  EVERY_N_MILLISECONDS(1) { // Blend towards the target palette
    // if (paletteOn == true) {
      nblendPaletteTowardPalette(currentPalette, targetPalette, maxPaletteBlendChanges);
    // }
  }

  // Effect
  Effect_fillnoise8();

  delay_at_max_brightness_for_power(led1.getSpeed() / LED1_COUNT);
}

uint16_t CustomEffect_bpm() {
  colorLoop = 1;

  // Process Color Palette Changes
  ProcessColorPalette();

  // Transition Current Palette Towards Target Palettee
  //
  // Each time that nblendPaletteTowardPalette is called, small changes
  // are made to currentPalette to bring it closer to matching targetPalette.
  // You can control how many changes are made in each call:
  // - the default of 24 is a good balance
  // - meaningful values are 1-48. 1=veeeeeeeery slow, 48=quickest
  // - "0" means do not change the currentPalette at all; freeze
  EVERY_N_MILLISECONDS(10) { // Blend towards the target palette
    nblendPaletteTowardPalette(currentPalette, targetPalette, maxPaletteBlendChanges);
  }

  EVERY_N_MILLISECONDS( 20 ) { gHue++; }

  // Effect
  Effect_bpm();

  delay_at_max_brightness_for_power(led1.getSpeed() / LED1_COUNT);
}

// ********** FASTLED EFFECTS ***********
void Effect_fillnoise8() {
  static uint8_t ihue=0;
  // If we're runing at a low "speed", some 8-bit artifacts become visible
  // from frame-to-frame.  In order to reduce this, we can do some fast data-smoothing.
  // The amount of data smoothing we're doing depends on "speed".
  uint8_t dataSmoothing = 0;
  if( fastled_speed < 50) {
    dataSmoothing = 200 - (fastled_speed * 4);
  }

  for(int i = 0; i < LED1_COUNT; i++) {
    int ioffset = scale * i;


      uint8_t data = inoise8(i * scale, dist + i * scale) % 255;


      // The range of the inoise8 function is roughly 16-238.
      // These two operations expand those values out to roughly 0..255
      // You can comment them out if you want the raw noise data.
      data = qsub8(data,16);
      data = qadd8(data,scale8(data,39));

      if( dataSmoothing ) {
        uint8_t olddata = noise[i];
        uint8_t newdata = scale8( olddata, dataSmoothing) + scale8( data, 256 - dataSmoothing);
        data = newdata;
      }

      noise[i] = data;
      uint8_t index = noise[i];

      if( colorLoop) {
        index += ihue;
      }
      //
      // if( bri > 127 ) {
      //   bri = 255;
      // } else {
      //   bri = dim8_raw( bri * 2);
      // }

      CRGB color = ColorFromPalette( currentPalette, index, 255, LINEARBLEND);
      leds[i] = color;

      // Set WS2812FX to FASTLED COLORS
      led1.setPixelColor(i, leds[i].green, leds[i].red, leds[i].blue);

  }

  ihue+=1;
  dist += beatsin8(2, 1, 4);

}

void Effect_bpm() {
    // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 120;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < LED1_COUNT; i++) {
    leds[i] = ColorFromPalette(currentPalette, gHue+(i*2), beat-gHue+(i*10));

    // Set WS2812FX to FASTLED COLORS
    led1.setPixelColor(i, leds[i].green, leds[i].red, leds[i].blue);
  }
}

/************ FASTLED: FUNCTIONS ****************/
void ProcessColorPalette() {
  // Example Color using Hue, Saturation, Value (Brightness)
  // CRGB red = CHSV( 0, 255, 255);
  // Example Color using HTML Color Name
  // CRGB red = CRGB::Red;

  CRGB red = CRGB::Red; // Main color
  CRGB orange = CRGB( 255, 106, 0); // Main color
  CRGB yellow = CRGB( 255, 175, 0); // Main color
  CRGB green = CRGB::Green; // Main color
  CRGB cyan = CRGB( 1, 254, 207); // Main color
  CRGB blue = CRGB::Blue; // Main color
  CRGB purple = CRGB( 179, 0, 255); // Main color
  CRGB magenta = CRGB::Magenta; // Main color
  CRGB black  = CRGB::Black;
  CRGB coolwhite  = CRGB( 255, 175, 125);
  CRGB warmwhite  = CRGB( 255, 155, 55) ;
  CRGB random = CHSV( random8(), 255, 128);
  CRGB current  = CRGB( g_color, r_color, b_color );
  CHSV current_chsv = rgb2hsv_approximate(current);

  CHSV current_add5 = CHSV(current_chsv.hue + 10, current_chsv.sat, current_chsv.val);
  CHSV current_add10 = CHSV(current_chsv.hue + 20, current_chsv.sat, current_chsv.val);
  CHSV current_add15 = CHSV(current_chsv.hue + 30, current_chsv.sat, current_chsv.val);
  CHSV current_add20 = CHSV(current_chsv.hue + 40, current_chsv.sat, current_chsv.val);
  CHSV current_add25 = CHSV(current_chsv.hue + 50, current_chsv.sat, current_chsv.val);
  CHSV current_add30 = CHSV(current_chsv.hue + 60, current_chsv.sat, current_chsv.val);

  CHSV current_sub5 = CHSV(current_chsv.hue - 10, current_chsv.sat, current_chsv.val);
  CHSV current_sub10 = CHSV(current_chsv.hue - 20, current_chsv.sat, current_chsv.val);
  CHSV current_sub15 = CHSV(current_chsv.hue - 30, current_chsv.sat, current_chsv.val);
  CHSV current_sub20 = CHSV(current_chsv.hue - 40, current_chsv.sat, current_chsv.val);
  CHSV current_sub25 = CHSV(current_chsv.hue - 50, current_chsv.sat, current_chsv.val);
  CHSV current_sub30 = CHSV(current_chsv.hue - 60, current_chsv.sat, current_chsv.val);


  if (palette == "CloudColors_p") { targetPalette = CloudColors_p;}
  // Default
  else if (palette == "LavaColors_p") { targetPalette = LavaColors_p;}
  else if (palette == "OceanColors_p") { targetPalette = OceanColors_p;}
  else if (palette == "ForestColors_p") { targetPalette = ForestColors_p;}
  else if (palette == "RainbowColors_p") { targetPalette = RainbowColors_p;}
  else if (palette == "RainbowStripeColors_p") { targetPalette = RainbowStripeColors_p;}
  else if (palette == "PartyColors_p") { targetPalette = PartyColors_p;}
  else if (palette == "HeatColors_p") { targetPalette = HeatColors_p;}

  // Current Color
  else if (palette == "CurrentWithBlack_p") {
    CRGB color1 = current;
    CRGB color2 = current;
    CRGB color3 = black;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCoolWhite_p") {
    CRGB color1 = current;
    CRGB color2 = current;
    CRGB color3 = coolwhite;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithWarmWhite_p") {
    CRGB color1 = current;
    CRGB color2 = current;
    CRGB color3 = warmwhite;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithWhite_p") {
    CRGB color1 = current;
    CRGB color2 = coolwhite;
    CRGB color3 = warmwhite;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }

  else if (palette == "CurrentWithCurrentAdd5") {
    CRGB color1 = current;
    CRGB color2 = current_add5;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAdd10") {
    CRGB color1 = current;
    CRGB color2 = current_add10;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAdd15") {
    CRGB color1 = current;
    CRGB color2 = current_add15;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAdd20") {
    CRGB color1 = current;
    CRGB color2 = current_add20;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAdd25") {
    CRGB color1 = current;
    CRGB color2 = current_add25;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAdd30") {
    CRGB color1 = current;
    CRGB color2 = current_add30;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }

  else if (palette == "CurrentWithCurrentSub5") {
    CRGB color1 = current;
    CRGB color2 = current_sub5;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentSub10") {
    CRGB color1 = current;
    CRGB color2 = current_sub10;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentSub15") {
    CRGB color1 = current;
    CRGB color2 = current_sub15;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentSub20") {
    CRGB color1 = current;
    CRGB color2 = current_sub20;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentSub25") {
    CRGB color1 = current;
    CRGB color2 = current_sub25;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentSub30") {
    CRGB color1 = current;
    CRGB color2 = current_sub30;
    CRGB color3 = current;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }

  else if (palette == "CurrentWithCurrentAug5") {
    CRGB color1 = current;
    CRGB color2 = current_sub5;
    CRGB color3 = current_add5;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAug10") {
    CRGB color1 = current;
    CRGB color2 = current_sub10;
    CRGB color3 = current_add10;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAug15") {
    CRGB color1 = current;
    CRGB color2 = current_sub15;
    CRGB color3 = current_add15;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAug20") {
    CRGB color1 = current;
    CRGB color2 = current_sub20;
    CRGB color3 = current_add20;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAug25") {
    CRGB color1 = current;
    CRGB color2 = current_sub25;
    CRGB color3 = current_add25;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CurrentWithCurrentAug30") {
    CRGB color1 = current;
    CRGB color2 = current_sub30;
    CRGB color3 = current_add30;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }

  // Custom Themes
  else if (palette == "BlueWithPurple_p") {
    CRGB color1 = blue;
    CRGB color2 = purple;
    CRGB color3 = cyan;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CoolWhiteWithBlack_p") {
    CRGB color1 = coolwhite;
    CRGB color2 = black;
    CRGB color3 = warmwhite;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CoolWhiteWithCyan_p") {
    CRGB color1 = coolwhite;
    CRGB color2 = black;
    CRGB color3 = cyan;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CyanWithGreen_p") {
    CRGB color1 = cyan;
    CRGB color2 = green;
    CRGB color3 = yellow;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CyanWithPurple_p") {
    CRGB color1 = cyan;
    CRGB color2 = purple;
    CRGB color3 = cyan;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "CyanWithWhite_p") {
    CRGB color1 = cyan;
    CRGB color2 = coolwhite;
    CRGB color3 = blue;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "GreenWithPurple_p") {
    CRGB color1 = green;
    CRGB color2 = purple;
    CRGB color3 = purple;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "MagentaWithWhite_p") {
    CRGB color1 = magenta;
    CRGB color2 = coolwhite;
    CRGB color3 = black;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "OrangeWithRed_p") {
    CRGB color1 = orange;
    CRGB color2 = red;
    CRGB color3 = yellow;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "Party_p") {
    CRGB color1 = CHSV( random8(), 255, 128);
    CRGB color2 = CHSV( random8(), 255, 128);
    CRGB color3 = CHSV( random8(), 255, 128);

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "MagentaWithYellow_p") {
    CRGB color1 = magenta;
    CRGB color2 = yellow;
    CRGB color3 = orange;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "PurpleWithBlack_p") {
    CRGB color1 = purple;
    CRGB color2 = black;
    CRGB color3 = blue;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "PurpleWithCyan_p") {
    CRGB color1 = purple;
    CRGB color2 = cyan;
    CRGB color3 = blue;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "PurpleWithGreen_p") {
    CRGB color1 = purple;
    CRGB color2 = green;
    CRGB color3 = green;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "RedWithBlack_p") {
    CRGB color1 = red;
    CRGB color2 = CRGB::DarkRed;
    CRGB color3 = black;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "RedWithOragnge_p") {
    CRGB color1 = red;
    CRGB color2 = orange;
    CRGB color3 = black;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "WarmWhiteWithBlack_p") {
    CRGB color1 = warmwhite;
    CRGB color2 = black;
    CRGB color3 = coolwhite;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }
  else if (palette == "YellowWithBlack_p") {
    CRGB color1 = yellow;
    CRGB color2 = black;
    CRGB color3 = coolwhite;

    targetPalette = CRGBPalette16(
      color1,   color3,   color2,   black,
      color1,   color1,   color1,   black,
      color1,   color2,   color1,   black,
      color1,   color1,   color1,   black );
  }

  // if (paletteOn == false) {
  //   currentPalette = targetPalette;
  //   paletteOn = true;
  // }
}

void ConvertTempToRGB(unsigned int kelvin) {
    int tmp_internal = kelvin / 100.0;
    int r_color_tmp = 0;

    // red
    if (tmp_internal <= 66) {
        r_color = 255;
    } else {
        float tmp_red = 329.698727446 * pow(tmp_internal - 60, -0.1332047592);
        if (tmp_red < 0) {
            r_color = 0;
        } else if (tmp_red > 255) {
            r_color = 255;
        } else {
            r_color = tmp_red;
        }
    }

    // green
    if (tmp_internal <=66){
        float tmp_green = 99.4708025861 * log(tmp_internal) - 161.1195681661;
        if (tmp_green < 0) {
            g_color = 0;
        } else if (tmp_green > 255) {
            g_color = 255;
        } else {
            g_color = tmp_green;
        }
    } else {
        float tmp_green = 288.1221695283 * pow(tmp_internal - 60, -0.0755148492);
        if (tmp_green < 0) {
            g_color = 0;
        } else if (tmp_green > 255) {
            g_color = 255;
        } else {
            g_color = tmp_green;
        }
    }

    // blue
    if (tmp_internal >=66) {
        b_color = 255;
    } else if (tmp_internal <= 19) {
        b_color = 0;
    } else {
        float tmp_blue = 138.5177312231 * log(tmp_internal - 10) - 305.0447927307;
        if (tmp_blue < 0) {
            b_color = 0;
        } else if (tmp_blue > 255) {
            b_color = 255;
        } else {
            b_color = tmp_blue;
        }
    }
  r_color_tmp = r_color;
  r_color = g_color;
  g_color = r_color_tmp;

}

// void BlendTowardsTargetPalette(){
//   if (ledMode != 999) {
//     switch (ledMode) {
//       case  1: all_off(); break;
//       case  2: SetupRandomPalette(); fastled_speed = 3; scale = 25; colorLoop = 1; break; //2-color palette
//       case  3: SetupRandomPalette_g(); fastled_speed = 3; scale = 25; colorLoop = 1; break; //3-color palette
//       case  4: SetupPurpleAndGreenPalette(); fastled_speed = 3; scale = 60; colorLoop = 1; break;
//       case  5: SetupGreenAndPurplePalette(); fastled_speed = 3; scale = 60; colorLoop = 1; break;
//       case  6: SetupBlackAndWhiteStripedPalette(); fastled_speed = 4; scale = 20; colorLoop = 1; ; break;
//       case  7: targetPalette = ForestColors_p; fastled_speed = 3; scale = 20; colorLoop = 0; break;
//       case  8: targetPalette = CloudColors_p; fastled_speed =  4; scale = 20; colorLoop = 0; break;
//       case  9: targetPalette = LavaColors_p;  fastled_speed =  8; scale = 19; colorLoop = 0; break;
//       case  10: targetPalette = OceanColors_p; fastled_speed = 6; scale = 25; colorLoop = 0;  break;
//       case  11: targetPalette = PartyColors_p; fastled_speed = 3; scale = 20; colorLoop = 1; break;
//     }
//   }
// }
//
// void MapNoiseToLEDsUsingPalette(){
//   static uint8_t ihue=0;
//
//   for(int i = 0; i < LED1_COUNT; i++) {
//
//     // We use the value at the (i,j) coordinate in the noise
//     // array for our brightness, and the flipped value from (j,i)
//     // for our pixel's index into the color palette.
//
//     uint8_t index = noise[0][i];
//     uint8_t bri =   noise[i][0];
//
//     // if this palette is a 'loop', add a slowly-changing base value
//     if( colorLoop) {
//       index += ihue;
//     }
//
//     // brighten up, as the color palette itself often contains the
//     // light/dark dynamic range desired
//     if( bri > 127 ) {
//       bri = 255;
//     } else {
//       bri = dim8_raw( bri * 2);
//     }
//
//     CRGB color = ColorFromPalette( currentPalette, index, bri);
//     leds[i] = color;
//
//     // Set WS2812FX to FASTLED COLORS
//     led1.setPixelColor(i, leds[i].green, leds[i].red, leds[i].blue);
//   }
//
//
// }
