/*
vCORE Universal IoT Device v0.7.10
  - mike.villarreal@outlook.com

*/

#define FASTLED_ALLOW_INTERRUPTS 0

/********** INITALIZATION *****************************************/
#include <Arduino.h>          // Required. Using .cpp file insteal .ino
#include "config.h"           // Required. Device Configureation File (Edit before final build)
#include <FastLED.h>          // Required. LED Strips
#include <WS2812FX.h>         // Required. LED Strips
#include <ESP8266WiFi.h>      // Required. WiFi
#include <ArduinoOTA.h>       // Required. OTA Updates via PlatformIO (platformio.ini)
#include <ESP8266mDNS.h>      // UNKNOWN! // TODO: Reasearch if ESP8266mDNS.h is needed
#include <PubSubClient.h>     // Required. MQTTT
#include <Adafruit_Sensor.h>  // Optional. DHT Sensors
#include <DHT.h>              // Optional. DHT Sensors
#include <IRsend.h>           // Optional. IR Transmitter

void SetupWifi();
void SetupOta();
void ReconnectMqtt();
void MqttCallback(char* topic, byte* payload, unsigned int length);
uint8_t LimitLedBrightness(uint8_t _red, uint8_t _green, uint8_t _blue, uint8_t _brightness);
void CheckDeviceReset();
void CheckButtons();
void CheckDht();
void CheckPir();
void CheckLedsOn();

int reset_button_state = 0;
bool device_reset = true;
bool device_ready = false;
int button_loop_interval = 0;

/********** INITALIZE FASTLED ***************************************/
uint16_t myCustomEffect();
void fillnoise8();
void mapNoiseToLEDsUsingPalette();
void ChangePaletteAndSettingsPeriodically();
void all_off();
void SetupRandomPalette();
void SetupRandomPalette_g();
void SetupPurpleAndGreenPalette();
void SetupBlackAndWhiteStripedPalette();
uint16_t XY( uint8_t x, uint8_t y);
void FillLEDsFromPaletteColors( uint8_t colorIndex);

/********** INITALIZE WS2812FX ***************************************/
WS2812FX led1 = WS2812FX(LED1_COUNT, LED1_PIN, NEO_RGB + NEO_KHZ800);

String set_color = "0,0,0";
String set_power = "OFF";
String set_brightness = "50";
String set_speed = "255";
int set_effect = FX_MODE_STATIC;

int r_color = 0;
int g_color = 0;
int b_color = 0;

/********** LIMIT LED BRIGHTNESS ********************************/
int brightness = 50;
int last_brightness = 50;
int new_brightness = 50;
int brightness_total_max = (255 * 3) * LED_BRIGHTNESS_LIMIT;
int brightness_total_actual = 0;
int brightness_total_delta = 0;
double brightness_total_delta_ratio = 0;

/********** INITALIZE WIFI ***************************************/
WiFiClient espClient;

/********** INITALIZE MQTT ***************************************/
PubSubClient client(espClient);
char message_buff[100];

/********** INITALIZE PIR ***************************************/
bool pir1_motion_state = false;
bool pir2_motion_state = false;
bool pir3_motion_state = false;
int pir_state = 0;
int mqtt_rearm_time = 0;
int pir1_rearm_time = 0;
int pir3_rearm_time = 0;
int pir_interval_time = 0;

/********** INITALIZE DHT ***************************************/
DHT dht(DHT_PIN, DHT_TYPE);

int dht_interval_time = 0;
bool dht_reconnect = true;

float dht_rolling_temperature_total = 0;
float dht_rolling_humidity_total = 0;
int dht_rolling_counter = 0;
int dht_rolling_counter_max = DHT_MQTT_PUB_INTERVAL / DHT_INTERVAL_LOOP;
float dht_average_temperature = 0;
float dht_average_humidity = 0;

/********** INITALIZE IR TRANSMITTER *******************************/
IRsend irsend(IR_TRANSMITTER1_PIN);

/******************************************************************/
/********** SETUP *************************************************/
/******************************************************************/
void setup() {
  // Start Serial
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println("Device: Booting");

  // Initialize Reset Button
  if (TOTAL_RESET_BUTTONS > 0 ) {
    pinMode(RESET_BUTTON_PIN, INPUT);
  }

  // FastLED.addLeds<WS2812B, LED1_PIN, RGB>(leds, 1); // init one FastLED led
  // FastLED.setBrightness(50);

  // led1.setBrightness(80);
  // led1.setColor(0,255,255); // Purple
  // led1.setSpeed(235);
  // led1.setMode(FX_MODE_STATIC);
  // led1.start();
  // led1.service();
  // delay(500);

  // Start WIFI
  // led1.setColor(255,0,0); // Green
  // led1.service();
  SetupWifi();
  // delay(1000);

  // Start OTA
  // led1.setColor(0,255,0); // Red
  // led1.service();
  SetupOta();
  // delay(5000);

  // Start MQTT
  // led1.setColor(0,0,255); // Blue
  // led1.service();
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



/******************************************************************/
/********** MAIN LOOP *********************************************/
/******************************************************************/
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



/*****************************************************************/
/********** SENSORS / MODULES ************************************/
/*****************************************************************/

/********** RESET BUTTON *****************************************/
void CheckDeviceReset() {
  reset_button_state = digitalRead(RESET_BUTTON_PIN);
  if (reset_button_state == HIGH ) {
    Serial.println("Reset Button Pressed. Resetting device.");
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


/********** CHECK BUTTONS *****************************************/
void CheckButtons() {
  int button1_state = LOW;

  button1_state = digitalRead(BUTTON1_PIN);
  if (button1_state == HIGH ) {
    if (millis() > button_loop_interval) {
      Serial.println("Button1 was pressed!");
      client.publish(PUB_BUTTON1, "1", true);

      delay(50);
      client.publish(PUB_BUTTON1, "0", true);

      button_loop_interval = millis() + BUTTON_LOOP_INTERVAL;
    }
  }
}

/********** PIR CODE *******************************************/
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


/********** DHT CODE ***************************************/
void CheckDht(){
  if (millis() > dht_interval_time) {
    // Get Values from DHT Sensor
    float temperature = dht.readTemperature(true) + DHT_TEMP_OFFSET;    //Fahrenheit
    // float temperature = dht.readTemperature(); + DHT_TEMP_OFFSET        // Celsius
    float humidity = dht.readHumidity();

    // Check if humidy & temperature are availble from DHT
    if (isnan(humidity) || isnan(temperature)) {
      if (dht_reconnect == false) {
        Serial.println("Failed to read from DHT sensor!");
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
    Serial.println("DEBUG: " + String(temperature));

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
      Serial.println("Temperature: " + String(dht_average_temperature) + ";  Humidity: " + String(dht_average_humidity));

      dht_rolling_temperature_total = 0;
      dht_rolling_humidity_total = 0;
      dht_rolling_counter = 0;
    }

    dht_interval_time = millis() + DHT_INTERVAL_LOOP;
  }
}

/********** LIMIT LED BRIGHTNESS *******************************/
uint8_t LimitLedBrightness(uint8_t _red, uint8_t _green, uint8_t _blue, uint8_t _brightness) {
  uint8_t _new_brightness = _brightness;
  brightness_total_actual = ( _red + _green + _blue) * _brightness;
  brightness_total_delta = brightness_total_actual - LED_BRIGHTNESS_LIMIT;
  brightness_total_delta_ratio = double(brightness_total_max) / double(brightness_total_actual);

  if (brightness_total_delta_ratio > 0 && brightness_total_delta_ratio < 1 ){
    int _new_brightness = int( (_brightness *  brightness_total_delta_ratio) - 0.5 );
    Serial.println("brightness_total_delta_ratio: " + String(brightness_total_delta_ratio) + "  input: " + _red + "," + _green + "," + _blue + " " + _brightness);
    Serial.println("Brightness reduced from " + String(last_brightness) + " to " + String(_new_brightness));
  }
  return _new_brightness;
}



/*****************************************************************/
/********** WIFI/OTA/MQTT CODE ***********************************/
/*****************************************************************/

/********** WIFI ***************************************/
void SetupWifi() {
  delay(10);
  Serial.println();
  Serial.print("WiFi: ");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  Serial.print("Connected to ");
  Serial.println(WIFI_SSID);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/********** OTA ***************************************/
void SetupOta() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Set Device/Host Name
  // ArduinoOTA.setHostname(DEVICE_NAME);

  // // Set Password
  // ArduinoOTA.setPassword(PASS); // same as wifi (via credentials.h)

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA Update: Connected");
}

/********** MQTT ***************************************/
void ReconnectMqtt() {
  // Loop until we're reconnected
  int retry = 0;
  while (!client.connected()) {
    ArduinoOTA.handle();
    Serial.print("MQTT: ");
    // Attempt to connect
    if (client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS)) {
      Serial.println("Connected");

      // client.publish(PUB_LED1_POWER, "ON");
      // client.publish(PUB_LED1_COLOR, "255,255,255");

      //send 'Device Restarted' notification to MQTT server
      if (device_reset == true) {
        client.publish(PUB_DEVICE, "restarted");
      }

      // send 'Device Connected' notification to MQTT server
      client.publish(PUB_DEVICE, "connected");

      // led1.setColor(255,255,255);
      // led1.service();

      client.subscribe(SUB_LED1_BRIGHTNESS);
      client.subscribe(SUB_LED1_COLOR);
      client.subscribe(SUB_LED1_POWER);
      client.subscribe(SUB_LED1_EFFECT);
      client.subscribe(SUB_LED1_SPEED);
      client.subscribe(SUB_IR_SEND);

          // Device is now Ready!
      if (device_ready == false) {
        device_ready = true;
        Serial.println("------------ DEVICE CONNECTED ------------");
        Serial.println();
      }
      retry = 0;
    } else {
      Serial.print("FAILED!; rc=");
      Serial.print(client.state());
      Serial.print("; retry=");
      Serial.print(retry);

      if (WiFi.status() != WL_CONNECTED) {
        SetupWifi();
      }
      // led1.setColor(255,0,0);
      // led1.service();

      retry = retry + 1;
      delay(5000);
      if (retry >= 10) {
        Serial.println("MQTT Failed: Resetting Device");

        // led1.stop();
        // led1.service();

        ESP.reset();
      }
    }
  }
}



/******************************************************************/
/********** MQTT CALLBACK *************************************************/
/******************************************************************/
void MqttCallback(char* topic, byte* payload, unsigned int length) {
  int i = 0;

  /********** POWER ****************/
  if (String(topic) == SUB_LED1_POWER) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    set_power = String(message_buff);

  }

  /********** EFFECT ****************/
  if (String(topic) == SUB_LED1_EFFECT) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    set_effect = String(message_buff).toInt();
    uint8_t _set_effect = set_effect;
    uint8_t _get_mode = led1.getMode();

    if (_get_mode != _set_effect) {
      if (_set_effect == 56) {
        led1.setCustomMode(myCustomEffect);
      }

      set_power == "ON";
      led1.setMode(_set_effect);


      Serial.println("Set Effect: " + String(message_buff));

      client.publish(PUB_HUMIDITY, String(dht_average_humidity).c_str(), true);
      client.publish(PUB_DEBUG, String("get: " + String(_get_mode) + ";  set: " + String(_set_effect)).c_str(), true);

      client.publish(PUB_LED1_EFFECT, message_buff);
    }
  }

  /********** BRIGHTNESS ****************/
  if (String(topic) == SUB_LED1_BRIGHTNESS) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    set_brightness = String(message_buff);
    brightness = set_brightness.toInt();


    new_brightness = LimitLedBrightness(r_color, b_color, g_color, brightness);
    uint8_t _get_brightness = led1.getBrightness();

    if (last_brightness != new_brightness) {
      set_power == "ON";
      led1.setBrightness(new_brightness);
      Serial.println("Set Brightness: " + set_brightness);
      client.publish(PUB_DEBUG, String("get: " + String(_get_brightness) + ";  set: " + String(set_brightness)).c_str(), true);
      if ( new_brightness > 0 ) {
        client.publish(PUB_LED1_BRIGHTNESS, String(new_brightness).c_str());
      }
      last_brightness = brightness;
    }
  }

  /********** COLOR ****************/
  if (String(topic) == SUB_LED1_COLOR) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    set_color = String(message_buff);

    g_color = set_color.substring(0, set_color.indexOf(',')).toInt();
    r_color = set_color.substring(set_color.indexOf(',') + 1, set_color.lastIndexOf(',')).toInt();
    b_color = set_color.substring(set_color.lastIndexOf(',') + 1).toInt();

    uint32_t _set_color = ((uint32_t)r_color << 16) | ((uint32_t)g_color << 8) | b_color;
    uint32_t _get_color = led1.getColor();
    uint8_t _get_brightness = led1.getBrightness();

    if (_get_color != _set_color) {
      set_power == "ON";
      client.publish(PUB_DEBUG, String("get: " + String(_get_color) + ";  set: " + String(_set_color)).c_str(), true);

      new_brightness = LimitLedBrightness(r_color, b_color, g_color, brightness);
      if (_get_brightness != new_brightness) {
        led1.setBrightness(new_brightness);
        if ( new_brightness > 0 ) {
          client.publish(PUB_LED1_BRIGHTNESS, String(new_brightness).c_str());
        }
      }
      led1.setMode(FX_MODE_STATIC);
      led1.setColor(r_color,g_color,b_color);

      Serial.println("Set Color: " + set_color);
      client.publish(PUB_LED1_COLOR, message_buff);
    }
  }

  /********** ANIMATION SPEED ****************/
  if (String(topic) == SUB_LED1_SPEED) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    set_speed = String(message_buff);

    uint8_t _set_speed = set_speed.toInt();
    uint8_t _get_speed = led1.getSpeed();

    if (_get_speed != _set_speed) {
      set_power == "ON";
      led1.setSpeed(_set_speed);
      client.publish(PUB_LED1_SPEED, message_buff);
    }
  }

  /********** IR_SEND ****************/
  if (String(topic) == SUB_IR_SEND) {
    for (i = 2; i < length; i++) {
      message_buff[i-2] = payload[i];
    }
    message_buff[i] = '\0';
    unsigned long msgInt = strtoul(String(message_buff).c_str(),0,16);
    irsend.sendNEC(msgInt, 32);
    client.publish(PUB_DEBUG, String("irsend : " + String(msgInt)).c_str(), true);
  }

  /********** Check if LEDS On ****************/
  CheckLedsOn();
}

void CheckLedsOn(){
  if (set_power == "OFF") {
    led1.stop();
    client.publish(PUB_LED1_POWER, "OFF");
  }
  if (set_power == "ON") {
    led1.init();
    led1.start();
    client.publish(PUB_LED1_POWER, "ON");
  }
  Serial.println("Set Power: " + set_power);
}





/******************************************************************/
/********** FASTLED ***********************************************/
/******************************************************************/




/********** INITALIZE FASTLED ***************************************/
// declare some parameters for the FastLED functions
#define MILLI_AMPERE      15000    // IMPORTANT: set here the max milli-Amps of your power supply 5V 2A = 2000
#define FRAMES_PER_SECOND  60    // here you can control the speed.
int ledMode = 4;                  // this is the starting palette
int BRIGHTNESS =           128;   // this is half brightness
int new_BRIGHTNESS =       128;   // shall be initially the same as brightness

static uint16_t x = random16();
static uint16_t y = random16();
static uint16_t z = random16();
uint8_t colorLoop = 1;
uint8_t noise[LED1_COUNT][LED1_COUNT];

// SPEED: Speed determines how fast time moves forward.
// 1 = very slow moving effect
// 60 = for something that ends up looking like water.
uint16_t speed; // speed is set dynamically by 'ChangePaletteAndSettingsPeriodically()'

// SCALE: Scale determines how far apart the pixels in our noise matrix are.
// The higher the value of scale, the more "zoomed out" the noise will be.
// 1 = zoomed in, you'll mostly see solid colors.
uint16_t scale; // scale is set dynamically by 'ChangePaletteAndSettingsPeriodically()'

CRGB leds[LED1_COUNT];
CRGBPalette16 currentPalette( CRGB::Black );
CRGBPalette16 targetPalette( CRGB::Black );

// 1 = 5 sec per palette
// 2 = 10 sec per palette
// etc
#define HOLD_PALETTES_X_TIMES_AS_LONG 2

/********** WS2812FX HOOK ***************************************/
uint16_t myCustomEffect() {
  BRIGHTNESS = led1.getBrightness();
  ChangePaletteAndSettingsPeriodically();
  // Crossfade current palette slowly toward the target palette
  //
  // Each time that nblendPaletteTowardPalette is called, small changes
  // are made to currentPalette to bring it closer to matching targetPalette.
  // You can control how many changes are made in each call:
  // - the default of 24 is a good balance
  // - meaningful values are 1-48. 1=veeeeeeeery slow, 48=quickest
  // - "0" means do not change the currentPalette at all; freeze
  //  uint8_t maxChanges = 7;
  //  nblendPaletteTowardPalette( currentPalette, targetPalette, maxChanges);

  // generate noise data
  fillnoise8();

  // convert the noise data to colors in the LED array
  // using the current palette
  mapNoiseToLEDsUsingPalette();


  show_at_max_brightness_for_power();
  delay_at_max_brightness_for_power(1000/FRAMES_PER_SECOND);
}

/********** FASTLED FUNCTIONS ***************************************/
uint16_t XY( uint8_t x, uint8_t y) {
  uint16_t i;
    i = (y * LED1_COUNT) + x;
  return i;
}

void FillLEDsFromPaletteColors( uint8_t colorIndex){
  uint8_t brightness = 255;

  for( int i = 0; i < LED1_COUNT; i++) {
    leds[i] = ColorFromPalette( currentPalette, colorIndex + sin8(i*16), brightness);
    led1.setPixelColor(i, leds[i].red, leds[i].green, leds[i].blue);
    colorIndex += 3;
  }
}

/********** EFFECT - FILLNOISE8 ***************************************/
// Fill the x/y array of 8-bit noise values using the inoise8 function.
void fillnoise8() {
  // If we're runing at a low "speed", some 8-bit artifacts become visible
  // from frame-to-frame.  In order to reduce this, we can do some fast data-smoothing.
  // The amount of data smoothing we're doing depends on "speed".
  uint8_t dataSmoothing = 0;
  if( speed < 50) {
    dataSmoothing = 200 - (speed * 4);
  }

  for(int i = 0; i < LED1_COUNT; i++) {
    int ioffset = scale * i;
    for(int j = 0; j < LED1_COUNT; j++) {
      int joffset = scale * j;

      uint8_t data = inoise8(x + ioffset,y + joffset,z);

      // The range of the inoise8 function is roughly 16-238.
      // These two operations expand those values out to roughly 0..255
      // You can comment them out if you want the raw noise data.
      data = qsub8(data,16);
      data = qadd8(data,scale8(data,39));

      if( dataSmoothing ) {
        uint8_t olddata = noise[i][j];
        uint8_t newdata = scale8( olddata, dataSmoothing) + scale8( data, 256 - dataSmoothing);
        data = newdata;
      }

      noise[i][j] = data;
    }
  }

  z += speed;

  // apply slow drift to X and Y, just for visual variation.
  x += speed / 8;
  y -= speed / 16;
}

void mapNoiseToLEDsUsingPalette(){
  static uint8_t ihue=0;

  for(int i = 0; i < LED1_COUNT; i++) {

    // We use the value at the (i,j) coordinate in the noise
    // array for our brightness, and the flipped value from (j,i)
    // for our pixel's index into the color palette.

    uint8_t index = noise[0][i];
    uint8_t bri =   noise[i][0];

    // if this palette is a 'loop', add a slowly-changing base value
    if( colorLoop) {
      index += ihue;
    }

    // brighten up, as the color palette itself often contains the
    // light/dark dynamic range desired
    if( bri > 127 ) {
      bri = 255;
    } else {
      bri = dim8_raw( bri * 2);
    }

    CRGB color = ColorFromPalette( currentPalette, index, bri);
    leds[i] = color;
    led1.setPixelColor(i, leds[i].green, leds[i].red, leds[i].blue);
  }

  ihue+=1;
}

/********** FASTLED - ANIMATIONS ***************************************/
void all_off() {  fill_solid( targetPalette, 16, CRGB::Black);}

void ChangePaletteAndSettingsPeriodically(){
  uint8_t maxChanges = 10;
  nblendPaletteTowardPalette( currentPalette, targetPalette, maxChanges);
  //uint8_t secondHand = ((millis() / 1000) / HOLD_PALETTES_X_TIMES_AS_LONG) % 60; //not used with webserver
  //static uint8_t lastSecond = 99;                                                 //not used with webserver

    if (ledMode != 999) {
      switch (ledMode) {
      case  1: all_off(); break;
      case  2: SetupRandomPalette(); speed = 3; scale = 25; colorLoop = 1; break; //2-color palette
      case  3: SetupRandomPalette_g(); speed = 3; scale = 25; colorLoop = 1; break; //3-color palette
      case  4: SetupPurpleAndGreenPalette(); speed = 3; scale = 15; colorLoop = 1; break;
      case  5: SetupBlackAndWhiteStripedPalette(); speed = 4; scale = 20; colorLoop = 1; ; break;
      case  6: targetPalette = ForestColors_p; speed = 3; scale = 20; colorLoop = 0; break;
      case  7: targetPalette = CloudColors_p; speed =  4; scale = 20; colorLoop = 0; break;
      case  8: targetPalette = LavaColors_p;  speed =  8; scale = 19; colorLoop = 0; break;
      case  9: targetPalette = OceanColors_p; speed = 6; scale = 25; colorLoop = 0;  break;
      case  10: targetPalette = PartyColors_p; speed = 3; scale = 20; colorLoop = 1; break;
      }
  }
}

// This function generates a random palette that's a gradient
// between four different colors.  The first is a dim hue, the second is
// a bright hue, the third is a bright pastel, and the last is
// another bright hue.  This gives some visual bright/dark variation
// which is more interesting than just a gradient of different hues.
void SetupRandomPalette() {
  EVERY_N_MILLISECONDS( 8000 ){ //new random palette every 8 seconds. Might have to wait for the first one to show up
  CRGB black  = CRGB::Black;
  CRGB random1 = CHSV( random8(), 255, 255);
  CRGB random2 = CHSV( random8(), 255, 255);
  targetPalette = CRGBPalette16(
                      random1,random1,black, black,
                      random2,random2,black, black,
                      random1,random1,black, black,
                      random2,random2,black, black);
  }
}

void SetupRandomPalette_g(){
  EVERY_N_MILLISECONDS( 8000 ){ //new random palette every 8 seconds
  CRGB black  = CRGB::Black;
  CRGB random1 = CHSV( random8(), 255, 255);
  CRGB random2 = CHSV( random8(), 200, 100);
  CRGB random3 = CHSV( random8(), 150, 200);
  targetPalette = CRGBPalette16(
                      random1,random1,black, black,
                      random2,random2,black, random3,
                      random1,random1,black, black,
                      random2,random2,black, random3);
 }
}

// This function sets up a palette of purple and green stripes.
void SetupPurpleAndGreenPalette() {
  CRGB purple = CHSV( HUE_PURPLE, 255, 255);
  CRGB green  = CHSV( HUE_GREEN, 255, 255);
  CRGB blue  = CHSV( HUE_BLUE, 255, 255);
  CRGB aqua  = CHSV( HUE_AQUA, 255, 255);
  CRGB black  = CRGB::Black;

  targetPalette = CRGBPalette16(
   blue,  green,  black,  purple,
   purple, blue , purple,  purple,
   green,  aqua,  black,  black,
   purple, purple, black,  purple );
}

// This function sets up a palette of black and white stripes,
// using code.  Since the palette is effectively an array of
// sixteen CRGB colors, the various fill_* functions can be used
// to set them up.
void SetupBlackAndWhiteStripedPalette(){
  // 'black out' all 16 palette entries...
  fill_solid( targetPalette, 16, CRGB::Black);
  // and set every eighth one to white.
  currentPalette[0] = CRGB::White;
  // currentPalette[4] = CRGB::White;
  currentPalette[8] = CRGB::White;
  //  currentPalette[12] = CRGB::White;
}
