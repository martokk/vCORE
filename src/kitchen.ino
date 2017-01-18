

/********** DEVICE ***************************************/
  #define DEVICE_NAME "kitchen" //Must be unique on Network.
  #define devicepub "device/kitchen"
  #define debugpub "device/kitchen/debug"

/********** LEDs ***************************************/
  #include <WS2812FX.h>

  #define setcolorsub "device/kitchen/setcolor"
  #define setpowersub "device/kitchen/setpower"
  #define seteffectsub "device/kitchen/seteffect"
  #define setbrightnesssub "device/kitchen/setbrightness"
  #define setanimationspeedsub "device/kitchen/setanimationspeed"

  #define setcolorpub "device/kitchen/setcolorpub"
  #define setpowerpub "device/kitchen/setpowerpub"
  #define seteffectpub "device/kitchen/seteffectpub"
  #define setbrightnesspub "device/kitchen/setbrightnesspub"
  #define setanimationspeedpub "device/kitchen/setanimationspeedpub"

  #define LED_COUNT 86
  #define LED_PIN D1
  int powerMax = 255; // 0 to 255

  /********** PIR MOTION SENSORS ***************************************/
    #define motion_topic "device/kitchen/motion"

    int pirPin1 = D5;
    int pirPin2 = D6;
    int pirPin3 = D7;
    int calibrationTime = 15; // in Seconds
    int pirDelay = 500; // Delay between loop()
    int pirRearmDelay = 500;  // in miliSeconds; Number of seconds without movement before the sensor will re-arm
    int pir3RearmDelay = 5000;  // in miliSeconds; Number of seconds without movement before the sensor will re-arm
    int mqttRearmDelay = 8000; // in millisecond; Will re-send mqtt status when PIR active

  /********** DHT SENSORS ***************************************/
    #include <DHT.h>

    #define humidity_topic "device/kitchen/humidity"
    #define temperature_topic "device/kitchen/temperature"

    #define DHTPIN D2     // what digital pin we're connected to
    #define DHTTYPE DHT11
    int dhtDelay = 12000;
    int dhtOffset = 6; //degress +/- to offest sensor. Use to calibrate DHT sensor to known temp.

    DHT dht(DHTPIN, DHTTYPE);








/******************************************************************/
/********** START *************************************************/
/******************************************************************/
#include <credentials.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h> // is this needed?
#include <PubSubClient.h>

int now = millis();
WiFiClient espClient;
char message_buff[100];
PubSubClient client(espClient);

/********** LED CODE ***************************************/
String setColor = "255,0,0";
String setPower;
String setBrightness = "50";
String setAnimationSpeed = "255";
String setColorTemp;

int setEffect = FX_MODE_RAINBOW;
int brightness = 50;
int animationspeed = 255;
int Rcolor = 0;
int Gcolor = 0;
int Bcolor = 0;

int lastBrightness = 50;
int newBrightness = 50;

int powerMaxTotal = (255 * 3) * powerMax;
int powerActualTotal = 0;
int powerDelta = 0;
double powerDeltaRatio = 0;

WS2812FX ws2812fx = WS2812FX(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

/********** SETUP ***************************************/
void setup() {
  // Start Serial
  Serial.begin(115200);
  Serial.println("Booting");

  // Start WS8212FX
  ws2812fx.init();
  ws2812fx.setBrightness(80);
  ws2812fx.setSpeed(235);
  ws2812fx.setColor(0,255,255); // Purple
  ws2812fx.setMode(FX_MODE_STATIC);
  ws2812fx.start();
  ws2812fx.service();

  // Start PIR
  pinMode(pirPin1, INPUT);
  pinMode(pirPin2, INPUT);
  pinMode(pirPin3, INPUT);
  setup_pir();
  delay(3000);

  // Start WIFI
  ws2812fx.setColor(255,0,0); // Green
  ws2812fx.service();
  setup_wifi();
  delay(1000);

  // Start OTA
  ws2812fx.setColor(0,255,0); // Red
  ws2812fx.service();
  setup_ota();
  delay(1000);

  // Start MQTT
  ws2812fx.setColor(0,0,255); // Blue
  ws2812fx.service();
  delay(1000);
  client.setServer(MQTT_SERVER, 1883); //CHANGE PORT HERE IF NEEDED
  client.setCallback(callback);

  // Start DHT
  dht.begin();

}

/***************** MAIN LOOP ********************************/
void loop() {
  now = millis();

  // Check of OTA Intall Request
  ArduinoOTA.handle();

  // Check PIR State
  checkPIR();

  // Check DHT
  checkDHT();

  // Run WS8212FX service
  ws2812fx.service();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

}









/******************************************************************/
/********** MQTT CALLBACK *************************************************/
/******************************************************************/
void callback(char* topic, byte* payload, unsigned int length) {
  int i = 0;

  /********** POWER ****************/
  if (String(topic) == setpowersub) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    setPower = String(message_buff);

    Serial.println("Set Power: " + setPower);

    if (setPower == "OFF") {
      ws2812fx.stop();
      ws2812fx.setMode(FX_MODE_STATIC);
      client.publish(setpowerpub, "OFF");
    }

    if (setPower == "ON") {
      ws2812fx.start();
      client.publish(setpowerpub, "ON");
    }
  }

  /********** EFFECT ****************/
  if (String(topic) == seteffectsub) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    setEffect = String(message_buff).toInt();

    ws2812fx.setMode(setEffect);
    ws2812fx.service();

    Serial.println("Set Effect: " + setEffect);
    // client.publish(seteffectpub, message_buff);
  }

  /********** BRIGHTNESS ****************/
  if (String(topic) == setbrightnesssub) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    setBrightness = String(message_buff);
    Serial.println("Set Brightness: " + setBrightness);
    brightness = setBrightness.toInt();
    lastBrightness = brightness;

    powerActualTotal = ( Rcolor + Bcolor + Gcolor) * brightness;
    powerDelta = powerActualTotal - powerMaxTotal;
    powerDeltaRatio = double(powerMaxTotal) / double(powerActualTotal);
    Serial.println("powerDeltaRatio: " + String(powerDeltaRatio));

    newBrightness = brightness;
    if (powerDelta > 0){
      newBrightness = int( (brightness *  powerDeltaRatio) - 0.5 );
      Serial.println("Brightness reduced from " + String(lastBrightness) + " to " + String(newBrightness));
    }

    ws2812fx.setBrightness(newBrightness);

    // client.publish(setbrightnesspub, message_buff);
  }

  /********** COLOR ****************/
  if (String(topic) == setcolorsub) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    client.publish(setcolorpub, message_buff);
    setColor = String(message_buff);

    Bcolor = setColor.substring(0, setColor.indexOf(',')).toInt();
    Rcolor = setColor.substring(setColor.indexOf(',') + 1, setColor.lastIndexOf(',')).toInt();
    Gcolor = setColor.substring(setColor.lastIndexOf(',') + 1).toInt();

    powerActualTotal = ( Rcolor + Bcolor + Gcolor) * brightness;
    powerDelta = powerActualTotal - powerMaxTotal;
    powerDeltaRatio = double(powerMaxTotal) / double(powerActualTotal);
    Serial.println("powerDeltaRatio: " + String(powerDeltaRatio));

    newBrightness = brightness;
    if (powerDelta > 0){
      newBrightness = int( (brightness *  powerDeltaRatio) - 0.5 );
      Serial.println("Brightness reduced from " + String(lastBrightness) + " to " + String(newBrightness));
    }

    ws2812fx.setBrightness(newBrightness);
    ws2812fx.setColor(Rcolor,Bcolor,Gcolor);
    ws2812fx.setMode(FX_MODE_STATIC);

    Serial.println("Set Color: " + setColor);
    }

  /********** ANIMATION SPEED ****************/
  if (String(topic) == setanimationspeedsub) {
    for (i = 0; i < length; i++) {
      message_buff[i] = payload[i];
    }
    message_buff[i] = '\0';
    setAnimationSpeed = String(message_buff);
    animationspeed = setAnimationSpeed.toInt();
    ws2812fx.setSpeed(animationspeed); //map
    // client.publish(setanimationspeedpub, message_buff);
  }
}




/*****************************************************************/
/********** SENSORS / MODULES ************************************/
/*****************************************************************/

/***************** PIR CODE ********************************/
bool motionState = false;
bool motionState3 = false;
int mqttRearmTime = 0;
int pirRearmTime = 0;
int pir3RearmTime = 0;

void setup_pir(){
  // PIR CALIBRATION
  delay(10);

  digitalWrite(pirPin1, LOW);
  digitalWrite(pirPin2, LOW);
  digitalWrite(pirPin3, LOW);

  Serial.print("PIR Calibrating");
  for (int i = 0; i < calibrationTime; i++) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("  COMPLETE.");
  Serial.println("SENSOR ACTIVE");
}

void checkPIR() {
  if (now > pirDelay) {

    int pir1State = digitalRead(pirPin1);
    int pir2State = digitalRead(pirPin2);
    int pir3State = digitalRead(pirPin3);
    int pirState = pir1State + pir2State + pir3State;
    // Serial.println(pirState);


    if (pirState > 0) {
      if (motionState == false) {
        motionState = true;

        // Serial.print("motion detected at ");
        // Serial.print(millis() / 1000);
        // Serial.println(" sec");
        if (motionState3 == false) {
          if (ws2812fx.getBrightness() < 100){
            ws2812fx.setBrightness(180);
          }
          ws2812fx.setSpeed(245);
          ws2812fx.setMode(FX_MODE_COLOR_WIPE_RANDOM);
        }
        client.publish(motion_topic, "1", true);
        mqttRearmTime = millis() + mqttRearmDelay;
        ws2812fx.start();
        client.publish(setpowerpub, "ON");
      } else {
        if (millis() > mqttRearmTime) {
          client.publish(motion_topic, "1", true);
          mqttRearmTime = millis() + mqttRearmDelay;
        }
      }
      // Kitchen Light Only (3rd Motion Sensor)
      if (pir3State == HIGH) {
        if (motionState3 == false) {
          motionState3 = true;
          ws2812fx.setMode(FX_MODE_COLOR_WIPE_TO_WHITE);
          ws2812fx.service();
        }
      pir3RearmTime = millis() + pir3RearmDelay;
      } else {
        if (millis() > pir3RearmTime) {
          if (motionState3 == true) {
            ws2812fx.setMode(FX_MODE_COLOR_WIPE_RANDOM);
            motionState3 = false;
          }
        }
      }
      pirRearmTime = millis() + pirRearmDelay;
    } else {
      if (motionState == true) {
        if (millis() > pirRearmTime) {
          motionState = false;
          motionState3 = false;

          Serial.print("Motion Detection Re-armed at ");
          Serial.print(millis() / 1000);
          Serial.println(" sec");
          client.publish(motion_topic, "0", true);
        }
      }
    }
    pirDelay = now + 200;
  }
}

/********** DHT CODE ***************************************/
int dhtTime = 0;

void checkDHT(){
  if (millis() > dhtTime) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    float f = dht.readTemperature(true) + dhtOffset;

    if (isnan(h) || isnan(t) || isnan(f)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    // Compute heat index in Fahrenheit (the default)
    float hif = dht.computeHeatIndex(f, h);
    // Compute heat index in Celsius (isFahreheit = false)
    float hic = dht.computeHeatIndex(t, h, false);

    Serial.print("Humidity: ");
    Serial.print(h);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(f);
    Serial.print(" *F");


    client.publish(humidity_topic, String(h).c_str(), true);
    client.publish(temperature_topic, String(f).c_str(), true);

    dhtTime = millis() + dhtDelay;
  }
}








/*****************************************************************/
/********** WIFI/OTA/MQTT CODE ***********************************/
/*****************************************************************/

/********** WIFI ***************************************/
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  // WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected (kitchen)");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

/********** OTA ***************************************/
void setup_ota() {
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Set Device/Host Name
  // ArduinoOTA.setHostname("esp8266-kitchen");

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
  Serial.println("Ready for OTA (kitchen)");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/********** MQTT ***************************************/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection (kitchen)...");
    // Attempt to connect
    if (client.connect(DEVICE_NAME, MQTT_USER, MQTT_PASS)) {
      Serial.println("connected");

      client.subscribe(setcolorsub);
      client.subscribe(setbrightnesssub);
      //client.subscribe(setcolortemp);
      client.subscribe(setpowersub);
      client.subscribe(seteffectsub);
      client.subscribe(setanimationspeedsub);

      client.publish(setpowerpub, "ON");
      client.publish(setcolorpub, "255,255,255");

      delay(1000);

      ws2812fx.setColor(175, 255, 110); // White (Color-Corrected)
      ws2812fx.setMode(FX_MODE_STATIC);
      ws2812fx.service();

      //send 'Device Connected' notification to MQTT server (handled by Home Assistant automation)
      client.publish(debugpub, "connected");
      
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      ws2812fx.setColor(0,0,255); // Blue
      ws2812fx.setMode(FX_MODE_STATIC);
      ws2812fx.service();

      delay(5000);
      // ESP.reset();
    }
  }
}
