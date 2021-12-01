
#if defined ARDUINO_SAMD_NANO_33_IOT && defined BLEEncryption
// Those are only relavant on the board with bluetooth and only when we have encyption implermented
#define PAIR_BUTTON 17      // button for pairing
#define PAIR_LED 16         // LED used to signal pairing
#define PAIR_LED_ON LOW     // Blue LED on Nano BLE has inverted logic
#define PAIR_INTERVAL 30000 // interval for pairing after button press in ms

#define CTRL_LED LED_BUILTIN
#endif
/*_______________________________________________________________________________________________*/
//DHT Defintions
#define DHTPIN 15 // Digital pin connected to the DHT sensor
// TODO change to DHT22
// Uncomment the type of sensor in use:
#define DHTTYPE DHT11 // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)
/*_______________________________________________________________________________________________*/
// Interrupt for CCS 881
// Possible Interrupt pins for the Nano 33 IoT are: 2, 3, 9, 10, 11, 13, A1, A5, A7
#ifdef ARDUINO_SAMD_NANO_33_IOT
#define INTERRUPT_PIN 17
#endif

// Possible Interrupt pins for the Leonardo are: 0, 1, 2, 3, 7
#ifdef ARDUINO_AVR_LEONARDO
#define INTERRUPT_PIN 2
#endif

#ifdef ARDUINO_AVR_NANO_EVERY
#define INTERRUPT_PIN 2
#endif
/*_______________________________________________________________________________________________*/
//Dustsensor Definitons for Leonardo
#ifdef ARDUINO_AVR_LEONARDO
#define DUST_SENSOR_LED_PIN 12       // Arduino digital pin 7 connect to sensor LED.
#define DUST_SENSOR_ANALOG_IN_PIN A0 // Arduino analog pin 5 connect to sensor Vo.
#endif
/*_______________________________________________________________________________________________*/
// Uncomment the #define DEBUG line to enable the printing Statements
// Credits go to Andreas Spiess wher I saw this trick
// https://www.youtube.com/watch?v=mFprG2UuHrA&t=1111s
#define DEBUG //TODO comment this line in final build

#ifdef DEBUG
#define DEBUGPRINT(x) Serial.print(x)
#define DEBUGPRINTLN(x) Serial.println(x)
#define DEBUGSERIALBEGIN(x) Serial.begin(x)
#else
#define DEBUGPRINT(x)
#define DEBUGPRINTLN(x)
#define DEBUGSERIALBEGIN(x)
#endif
/*_______________________________________________________________________________________________*/
// NOTE ALL this tags have to be adjusted for final build
#define NODE_ID 1
#define NODE_LOCATION "Livingroom"
/*===============================================================================================*/
#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_CCS811.h>
#include <Wire.h>
#include <SPI.h>

/*_______________________________________________________________________________________________*/
#ifdef ARDUINO_SAMD_NANO_33_IOT
#include <ArduinoBLE.h>
#include <ble_definitions.h>
#include <JC_Button.h>
#endif

/*_______________________________________________________________________________________________*/
#ifdef ARDUINO_AVR_NANO_EVERY
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>
// TODO Add the E-Paper SPI Screen Library
// TODO Add the
#endif
/*_______________________________________________________________________________________________*/
#ifdef ARDUINO_AVR_LEONARDO
#include <GP2Y1010sensor.h>
#endif
/*===============================================================================================*/
//Global Variables
unsigned short eco2 = 0;
unsigned short tvoc = 0;
float temperature = 25.0;
float humidity = 50.0;
byte batteryPrecentage = 0;
/*===============================================================================================*/
// BLE Services only on the Arduino Nano 33 Iot
#ifdef ARDUINO_SAMD_NANO_33_IOT
BLEService enviormentalSensingService(SERVICE_ENVIRONMENTAL_SENSING_UUID);
BLEService batteryService(SERVICE_BATTERY_SERVICE_UUID);
BLEService deviceInformationService(SERVICE_DEVICE_INFORMATION_UUID);

/*_______________________________________________________________________________________________*/
// NOTE BLEDesciptor Could enable Notifications on encryption because Desciptor is CCCD
/* NOTE Example (not notification) 
   * BLEDescriptor batteryLevelDescriptor("2901", "millis");
   * batteryLevelChar.addDescriptor(batteryLevelDescriptor);
   * BLE Battery Level Characteristic with encryption when when I add the feature in the future
   */
#ifdef BLEEncryption
BLEShortCharacteristic temperatureValue(CHARACTERISTIC_TEMPERATURE_CELSIUS_UUID,
                                        BLERead | BLEEncryption);
BLEShortCharacteristic humidityValue(CHARACTERISTIC_HUMIDITY_UUID, BLERead | BLEEncryption);
BLEShortCharacteristic heatIndexValue(CHARACTERISTIC_HEAT_INDEX_UUID, BLERead | BLEEncryption);
BLEUnsignedShortCharacteristic eco2Value(CHARACTERISTIC_ECO2_UUID, BLERead | BLEEncryption);
BLEUnsignedShortCharacteristic tvocValue(CHARACTERISTIC_TVOC_UUID, BLERead | BLEEncryption);
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
BLEByteCharacteristic batteryLevelValue(CHARACTERISTIC_BATTERY_LEVEL_UUID,
                                        BLERead | BLEEncryption);
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
BLEStringCharacteristic sernsorLocationConst(CHARACTERISTIC_SENSOR_LOCATION_UUID, BLERead | BLEWrite | BLEEncryption, 31);
BLEUnsignedLongCharacteristic systemIDConst(CHARACTERISTIC_SYSTEM_ID_UUID,
                                            BLERead | BLEEncryption);
//TODO Use BLE_Desciptor to reenable Notification on BLE_Encryption
#else
/*___________________________________________________________________________________________*/

BLEShortCharacteristic temperatureValue(CHARACTERISTIC_TEMPERATURE_CELSIUS_UUID, BLERead);
BLEShortCharacteristic humidityValue(CHARACTERISTIC_HUMIDITY_UUID, BLERead);
BLEShortCharacteristic heatIndexValue(CHARACTERISTIC_HEAT_INDEX_UUID, BLERead);
BLEUnsignedShortCharacteristic eco2Value(CHARACTERISTIC_ECO2_UUID, BLERead);
BLEUnsignedShortCharacteristic tvocValue(CHARACTERISTIC_TVOC_UUID, BLERead);
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
BLEByteCharacteristic batteryPercentageValue(CHARACTERISTIC_BATTERY_LEVEL_UUID, BLERead);
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
BLEStringCharacteristic sernsorLocationConst(CHARACTERISTIC_SENSOR_LOCATION_UUID,
                                             BLERead | BLEWrite, 31);
BLEUnsignedLongCharacteristic systemIDConst(CHARACTERISTIC_SYSTEM_ID_UUID, BLERead);
#endif
#endif
/*===============================================================================================*/
void cbCcsDataReady()
{
  //ccs.setEnvironmentalData(humidity, temperature);
  if (!ccs.readData())
  {
    eco2 = ccs.geteCO2();
    tvoc = ccs.getTVOC();
  }
}
/*___________________________________________________________________________________________*/
//int oldBatteryLevel = 0;  // last battery level reading from analog input
//long previousMillis = 0;  // last time the battery level was checked, in ms

//void updateBatteryLevel() {
/* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
/*int battery = analogRead(A0);
  int batteryLevel = map(battery, 0, 1023, 0, 100);

  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    Serial.print("Battery Level % is now: "); // print it
    Serial.println(batteryLevel);
    batteryLevelChar.writeValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
}*/
/*===============================================================================================*/
// Object Definitions
Adafruit_CCS811 ccs;
DHT dht(DHTPIN, DHTTYPE);

#ifdef ARDUINO_AVR_LEONARDO
// Add the Dust Sensor library only on the leonardo (which is buildin to my latte panda)
GP2Y1010sensor DustSensor;
#endif
// TODO XIAO setup soil moustior sensor
/*===============================================================================================*/
void setup()
{
  DEBUGSERIALBEGIN(9600); // initialize serial communication
  // while (!Serial);
  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected
  digitalWrite(LED_BUILTIN, HIGH);
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  DEBUGPRINTLN("Starting CCS");
  if (!ccs.begin())
  {
    DEBUGPRINTLN("Failed to start CCS sensor! Please check your wiring.");
    delay(1000);
    while (true)
      ;
  }
  // Wait for the sensor to be ready
  while (!ccs.available())
    ;
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  DEBUGPRINTLN("Starting DHT");
  if (!dht.begin())
  {
    DEBUGPRINTLN("Failed to start DHT sensor! Please check your wiring.");
    delay(1000);
    while (true)
      ;
  }
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  css.setDriveMode(CCS811_DRIVE_MODE_10SEC); //For DEBUG only
  //TODO uncoment the line below, and comment the line above
  ccs.setDriveMode(CCS811_DRIVE_MODE_60SEC); // Measure only every 60 seconds
  // CCS has a lot of clockdrifing about a second every minute
  // ccs.enableInterrupt();
/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
// Attach Interrupt to interupt pin to wake up the mcu every time the ccs has a new value ready
#ifdef ARDUINO_SAMD_NANO_33_IOT
  LowPower.attachInterruptWakeup(INTERRUPT_PIN, cbCcsDataReady, FALLING);
  /*___________________________________________________________________________________________*/
  // begin initialization for the Arduino Nano IoT

  if (!BLE.begin())
  {
    DEBUGPRINTLN("starting BLE failed!");
    while (true)
      ;
  }
  /*___________________________________________________________________________________________*/
  /* Set a local name for the BLE device
      This name will appear in advertising packets
      and can be used by remote devices to identify this BLE device
      The name can be changed but maybe be truncated based on space left in advertisement packet
    */
  BLE.setDeviceName("Arduino Nano 33 IoT");
  BLE.setLocalName("EnviormentalMonitor");
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  BLE.setAdvertisedService(enviormentalSensingService); //TODO maybe remove all but device information from advertising if I see "3 devices" in nRF connect
  BLE.setAdvertisedService(batteryService);             //TODO maybe remove all but device information from advertising if I see "3 devices" in nRF connect
  BLE.setAdvertisedService(deviceInformationService);   // add the service UUID

  /*___________________________________________________________________________________________*/
  //Adding the Charcateristics to the coresponding services
  enviormentalSensingService.addCharacteristic(temperatureValue);
  enviormentalSensingService.addCharacteristic(humidityValue);
  enviormentalSensingService.addCharacteristic(heatIndexValue);
  enviormentalSensingService.addCharacteristic(eco2Value);
  enviormentalSensingService.addCharacteristic(tvocValue);
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  batteryService.addCharacteristic(batteryPercentageValue);
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  deviceInformationService.addCharacteristic(sernsorLocationConst);
  deviceInformationService.addCharacteristic(systemIDConst);
  /*___________________________________________________________________________________________*/
  //Adding Services
  BLE.addService(enviormentalSensingService);
  BLE.addService(batteryService);
  BLE.addService(deviceInformationService);
  /*___________________________________________________________________________________________*/
  //Writing Default and Constant Values to every characteristic
  temperatureValue.writeValue(25);
  humidityValue.writeValue(50);
  heatIndexValue.writeValue(25);
  eco2Value.writeValue(400);
  tvocValue.writeValue(0);
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  batteryPercentageValue.writeValue(100);
  /*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/
  sernsorLocationConst.writeValue(NODE_LOCATION);
  systemIDConst.writeValue(NODE_ID);

  /*___________________________________________________________________________________________*/
  /* Start advertising BLE.  It will start continuously transmitting BLE
      advertising packets and will be visible to remote BLE central devices
      until it receives a new connection */
  BLE.advertise();
  DEBUGPRINTLN("Bluetooth device active, waiting for connections...");
  /*___________________________________________________________________________________________*/
#endif
// begin initialization for the Arduino Leonardo
#ifdef ARDUINO_AVR_LEONARDO
  DustSensor.init(DUST_SENSOR_LED_PIN, DUST_SENSOR_ANALOG_IN_PIN, K, N);
  DEBUGPRINTLN("Initialized Dust Sensor");
#endif
  // TODO ARDUINO_AVR_NANO_EVERY Specific Setups likje E-Paper
  /*_____________________________________________________________________________________________*/
}

void loop()
{
#ifdef ARDUINO_SAMD_NANO_33_IOT
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central)
  {
    DEBUGPRINT("Connected to central: ");
    // print the central's BT address:
    DEBUGPRINTLN(central.address());
    // turn off the LED to indicate the connection, this conserves power
    digitalWrite(LED_BUILTIN, LOW);

    // check the battery level every 200ms
    // while the central is connected:
    //while (central.connected()) {
    //long currentMillis = millis();
    // if 200ms have passed, check the battery level:
    //if (currentMillis - previousMillis >= 200) {
    //previousMillis = currentMillis;
    //updateBatteryLevel();
    //}
    //}
    // when the central disconnects, turn on the LED:
    while (central.connected())
    { //TODO make it while Serial on leonardo and while /mqtt on xiao
      //TODO as a result, for this to be more efficent putt everything into a method.
      LowPower.sleep(); //Sleep indefinetly until an interrupt is triggered
      // We return here from the interrupt.
      humidity = dht.readHumidity();
      // Read temperature as Celsius (the default)
      temperature = dht.readTemperature();
      if (isnan(temperature) || isnan(humidity))
      {
        DEBUGPRINTLN("Error reading from DHT");
      }
      else
      {
        //Set enviornmental  Data to ccs to improve co2 estimates.
        ccs.setEnvironmentalData(humidity, temperature);
      }
      float hic = dht.computeHeatIndex(temperature, humidity, false);

      //TODO read battery (only on Nano 33 iot)
      //TODO read Dust sensor (only on leonardo)
      //TODO set the Values to the ble characteristics
    }
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    //TODO when no device is connected after 10 minutes go to deepSleep,
    //TODO if the bt gateway has many failed attemts at reconnecting, it will notify me.
    //TODO maybe a second button will be enabled as interrupt, reseting the 10 minute timer
    //TODO and starting the 10 minute waiting from the beginning.
  }
#endif
}
