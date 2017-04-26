/*
vCORE Universal IoT Device v0.1.2
  - mike.villarreal@outlook.com

*/


/********** DEVICE ***************************************/
  #define DEVICE_NAME               "change_me"   // Must be unique on Network.

/********** PUSH BUTTON ***************************************/
  #define TOTAL_RESET_BUTTONS       0         // 0 or 1 Buttons to reset device
  #define RESET_BUTTON_PIN          0         // Reset Button Pin

/********** PUSH BUTTON ***************************************/
  #define TOTAL_MQTT_BUTTONS        0         // 0 or 1 Buttons to reset device
  #define BUTTON_LOOP_INTERVAL      000       // Interval Between loops

  #define BUTTON1_PIN               0         // Reset Button Pin

/********** LEDs ***************************************/
  #define TOTAL_LED_STRIPS          0         // Supports up to 5 seperate LED Strips
  #define LED_BRIGHTNESS_LIMIT      0         // Limit Brightnes Level (Max = 255)

  #define LED1_PIN                  0         // Arduno pin
  #define LED1_COUNT                0         // Total LEDs

/********** PIR ***************************************/
  #define TOTAL_PIR_SENSORS         0
  #define PIR_LOOP_INERVAL          0         // Interval Between loops (Default: 250)
  #define PIR_MQTT_RETRIGGER_DELAY  0         // Re-send PIR HIGH status to MQTT (Default: 4000)
  #define PIR_CALIBRATION_DELAY     0         // Calibration time (Default: 15000)

  #define PIR1_PIN                  0         // Arduiuno Pin
  #define PIR1_TRIGGER_MODE         0         // 1 = Sensor Only; 2 = Sensor & LEDs
  #define PIR1_OFF_DELAY            0000      // Time before PIR triggers back to 0 ("off")

  #define PIR2_PIN                  0         // Arduno pin
  #define PIR2_TRIGGER_MODE         0         // 1 = Sensor Only; 2 = Sensor & LEDs
  #define PIR2_OFF_DELAY            0000      // Time before PIR triggers back to 0 ("off")

  #define PIR3_PIN                  0         // Arduno pin
  #define PIR3_TRIGGER_MODE         0         // 1 = Sensor Only; 2 = Sensor & LEDs
  #define PIR3_OFF_DELAY            0000      // Time before PIR triggers back to 0 ("off")

/********** DHT ***************************************/
  #define TOTAL_DHT_SENSORS         0
  #define DHT_PIN                   0         // Arduno pin
  #define DHT_TYPE                  DHT11     // DH11; DH22; DHT21; AM2301
  #define DHT_INTERVAL_LOOP         3000      // DHT11=1000; DHT22=2000
  #define DHT_MQTT_PUB_INTERVAL     120000    // Delay between MQTT Publish (Avereged every PUB_INTERVAL)
  #define DHT_TEMP_OFFSET           0         // Calibration offset

/********** MQTT ***************************************/
  /*--------- DEVICE --------------*/
    #define PUB_DEVICE              "device/change_me/device"
    #define PUB_DEBUG               "device/change_me/debug"

  /*--------- BUTTON --------------*/
    #define PUB_BUTTON1            "device/change_me/button1"

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
