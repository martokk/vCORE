/*
vCORE Universal IoT Device v0.1.1
  - mike.villarreal@outlook.com

  REQUIREMENTS
    - LED Strips (min=1, max=5)
    - MQTT
    - WIFI
  OPTIONAL
    - PIR (min=0, max=3)
    - DHT (min=0, max = 1)

*/


/********** DEVICE ***************************************/
  #define DEVICE_NAME               "change_me"   // Must be unique on Network.

/********** PUSH BUTTON ***************************************/
  #define TOTAL_RESET_BUTTONS       0         // 0 or 1 Buttons to reset device
  #define RESET_BUTTON_PIN          D0        // Reset Button Pin

/********** LEDs ***************************************/
  #define TOTAL_LED_STRIPS          0         // Supports up to 5 seperate LED Strips
  #define LED_BRIGHTNESS_LIMIT      50       // Limit Brightnes Level (Max = 255)

  #define LED1_PIN                  D0        // Arduno pin
  #define LED1_COUNT                0         // Total LEDs
  // #define LED1_MQTT_MODE            0      // 0 = Use LED1; 1 = Use own

/********** PIR ***************************************/
  #define TOTAL_PIR_SENSORS         0
  #define PIR_LOOP_INERVAL          250       // Interval Between loops
  #define PIR_MQTT_RETRIGGER_DELAY  4000      // Re-send PIR HIGH status to MQTT
  #define PIR_CALIBRATION_DELAY     15000     // Calibration time

  #define PIR1_PIN                  D0        // Arduiuno Pin
  #define PIR1_MODE                 0         // 1 = Sensor Only; 2 = Sensor & LEDs
  #define PIR1_REARM_MODE           0         // 0 = Use PIR1; 1 = Use own
  #define PIR1_REARM_DELAY          3000      // Time before PIR triggers back to 0 ("off")

  #define PIR2_PIN                  D0        // Arduno pin
  #define PIR2_MODE                 0         // 1 = Sensor Only; 2 = Sensor & LEDs
  #define PIR2_REARM_MODE           0         // 0 = Use PIR1; 1 = Use own
  #define PIR2_REARM_DELAY          3000      // Time before PIR triggers back to 0 ("off")

  #define PIR3_PIN                  D0        // Arduno pin
  #define PIR3_MODE                 0         // 1 = Sensor Only; 2 = Sensor & LEDs
  #define PIR3_REARM_MODE           1         // 0 = Use PIR1; 1 = Use own
  #define PIR3_REARM_DELAY          5000      // Time before PIR triggers back to 0 ("off")

/********** DHT ***************************************/
  #define TOTAL_DHT_SENSORS         0
  #define DHT_PIN                   D0        // Arduno pin
  #define DHT_TYPE                  DHT11     // DH11; DH22; DHT21; AM2301
  #define DHT_INTERVAL_LOOP         3000      // DHT11=1000; DHT22=2000
  #define DHT_MQTT_PUB_INTERVAL     120000    // Delay between MQTT Publish (Avereged every PUB_INTERVAL)
  #define DHT_TEMP_OFFSET           0         // Calibration offset

/********** MQTT ***************************************/
  /*--------- DEVICE --------------*/
    #define PUB_DEVICE              "device/change_me/device"
    #define PUB_DEBUG               "device/change_me/debug"

  /*--------- PIR --------------*/
    #define PUB_PIR1                "device/change_me/pir1"
    #define PUB_PIR2                "device/change_me/pir2"
    #define PUB_PIR3                "device/change_me/pir3"

  /*--------- DHT --------------*/
    #define PUB_HUMIDITY            "device/change_me/humidity"
    #define PUB_TEMPERATURE         "device/change_me/temperature"

  /*--------- LED STRIP 1 --------------*/
    #define SUB_LED1_POWER          "device/change_me/led1/power"
    #define SUB_LED1_COLOR          "device/change_me/led1/color"
    #define SUB_LED1_BRIGHTNESS     "device/change_me/led1/brightness"
    #define SUB_LED1_EFFECT         "device/change_me/led1/effect"
    #define SUB_LED1_SPEED          "device/change_me/led1/speed"

    #define PUB_LED1_POWER          "device/change_me/led1/power/pub"
    #define PUB_LED1_COLOR          "device/change_me/led1/color/pub"
    #define PUB_LED1_BRIGHTNESS     "device/change_me/led1/brightness/pub"
    #define PUB_LED1_EFFECT         "device/change_me/led1/effect/pub"
    #define PUB_LED1_SPEED          "device/change_me/led1/speed/pub"
