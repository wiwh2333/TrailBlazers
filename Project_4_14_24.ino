
#include <Wire.h> 
#include <SPI.h>
#include <Arduino.h>
#include <math.h>
#include <Adafruit_Sensor.h>
//Sensor Libraries
#include <Adafruit_GPS.h> //GPS
#include "Adafruit_BME680.h"//Temp,Pressure,Humidity
#include "Adafruit_PM25AQI.h"//Particle Count
#include <Adafruit_INA219.h>//Voltage/Current
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#define GPSSerial Serial2 //GPS
#define RX_PIN    18 //GPS
#define TX_PIN    17 //GPS
#define GPSECHO true //GPS

#define I2C_SCL    9 //GPS
#define I2C_SDA    10 //GPS

#define wheelEncoderPin_forward 15 //Encoder
#define wheelEncoderPin_backward 16 //Encoder

#define           BNO08X_CS 4 //BNO
#define           BNO08X_INT 5 //BNO
#define           BNO08X_RESET 6 //BNO

//environmental sensors setup
#define SEALEVELPRESSURE_HPA (1013.25)

#define GPS_FLAG false
#define BNO true
#define HALL true
#define BME false
#define INA true
#define PM25AQI false

#define PUTTY false //Print "Acell Z: 0.00 Azell Y: 0.00" or "0.00, 0.00, "
#define NUM_READS 20


// Adafruit_BNO08x   bno08x(BNO08X_RESET); //Send Reset to BNO-085
BNO080 myIMU;
// BNO080 myIMU2;
Adafruit_GPS GPS(&GPSSerial); //Connect to the GPS on hardware

byte imuCSPin = 4;
byte imuWAKPin = 37;
byte imuINTPin = 5;
byte imuRSTPin = 6;


uint32_t timer = millis(); //GPS
//BNO
float defaultX=-2.0; //BNO X Vector
float defaultY=-2.0; //BNO Y Vector
float defaultZ=-2.0; //BNO Z Vector
bool ready_for_bno = true; 
float alpha;
float AccelZ;
float intermed;
//Wheel Encoder
int ticksPerRotation; //# Magnets on the wheel
float wheelDiameter;
int wheelTicks;
float totalDistance;
float prevDistance;
bool forwardFlag;
bool backwardFlag=false;
bool readFlag;
bool StoreFlag =0;
bool unitFlag= false;
float unitConversion;
bool usingInches;
int times_printed;
int test = 0;
unsigned long intTime = 0;
unsigned long lastTime = 0;
//GPS
float lat = 0;
float lon = 0;
char lat_string = 'X';
char lon_string = 'X';
//initialize all sensors
Adafruit_BME680 bme;
Adafruit_INA219 ina219;
PM25_AQI_Data data;
Adafruit_PM25AQI aqi = Adafruit_PM25AQI();
//sensors variables
int counter = 0;
float total_temp = 0;
float total_hum = 0;
float total_pressure = 0; 
float total_co2 = 0;
int co2_counter = 0;
float total_count = 0;
float total_loadvoltage = 0;
float total_current = 0;
float total_c = 0;
//INA Readings 
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power_mW = 0;
//PM25AQI
int particle_count;
//BME
float temp;
float pressure; 
float humidity; 
// interrupt variables
float station_distance = 0; // for automatic station creation
bool create_station = false;  // this will be sent from the tablet

float x = 1;
float y = 2;
float z = 3;
bool alert = 0;


//Bluetooth Setup
// having each piece of data be its own characteristic will simplify sending multiple packets.
BLECharacteristic *pCharacteristic_GPS;   // GPS data characteristic
BLECharacteristic *pCharacteristic_ACC;   // Accelerometer data characteristic
BLECharacteristic *pCharacteristic_DIST;  // Hall Effect data characteristic
BLECharacteristic *pCharacteristic_BATT;  // Battery voltage characteristic
BLECharacteristic *pCharacteristic_ENV;   // environmental sensors data characteristic (may have to split this up)
BLECharacteristic *pCharacteristic_RX; // characteristic for writing to ESP32
BLECharacteristic *pCharacteristic_ALERT; //characteristic for an alert

bool deviceConnected = false;
const int LED = 42;      // pin of the RGB LED

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9A"
#define CHARACTERISTIC_UUID_RX "6E400003-B5A3-F393-E0A9-E50E24DCCA9B"   // for tablet writes to ESP32
#define CHARACTERISTIC_UUID_GPS "6E400003-B5A3-F393-E0A9-E50E24DCCA9C"
#define CHARACTERISTIC_UUID_ACC "6E400003-B5A3-F393-E0A9-E50E24DCCA9D"
#define CHARACTERISTIC_UUID_DIST "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_ENV "6E400003-B5A3-F393-E0A9-E50E24DCCA9F"
#define CHARACTERISTIC_UUID_ALERT "6E400003-B5A3-F393-E0A9-E50E24DCCA90"

// Alert codes
#define DISTANCE_ALERT 1
#define GRADE_ALERT 2
#define X_SLOPE_ALERT 3
#define GPS_ALERT 4
#define TARE_COMPLETE_ALERT 5
//Threshold values to check for alerts
float grade_threshold = 5.0;
float x_slope_threshold = 5.0;

#define CHUNK_SIZE 20 // Define the chunk size

float bt_stationData[10]; // define arrary to transmit GPS, accelerometer, and wheel encoder data

void call_delay(int time);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.println("*********");
      Serial.print("Received Value: ");

      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]); // print receiving key
      }

      Serial.println();

      // Do stuff based on the command received from the app
      if (rxValue.find(1) != -1) { // get values from sensors and send back to the app
        Serial.println("Manual Read");
        
        shuntvoltage = ina219.getShuntVoltage_mV();
        busvoltage = ina219.getBusVoltage_V();
        loadvoltage = busvoltage + (shuntvoltage / 1000.0);

        current_mA = ina219.getCurrent_mA();

        particle_count = data.particles_03um + data.particles_05um + data.particles_10um;
        total_count += particle_count;
        
        x = 0; y = 0; z = 0;
    
        for (int i = 0; i < NUM_READS; i++){
          while(myIMU.dataAvailable() == false){delay(1);}
          
          x = x + myIMU.getRawAccelX();
          y = y + myIMU.getRawAccelY();
          z = z + myIMU.getRawAccelZ();
        }
        
        x = x/NUM_READS; y=y/NUM_READS; z=z/NUM_READS; 
        x = x - defaultX; y= -1*(y- defaultY); z= z-defaultZ;
        x = (x/16.0)*(90.0/256.0); y = (y/16.0)*(90.0/256.0); z = (z/16.0)*(90.0/256.0);
        
        if(y >= grade_threshold){
          sendAlert(GRADE_ALERT);
        }
        elif(x >= x_slope_threshold){
          sendAlert(X_SLOPE_ALERT);
        }
        if(lat_string == 'S') {
            bt_stationData[0] = -1 * lat;
        }  
        else {
            bt_stationData[0] = -1 * lat;
        }

        if(lon_string == 'W') {
            bt_stationData[1] = -1 * lon;
        }
        else {
          bt_stationData[1] = lon;
        }

        bt_stationData[2] = x; // need to adjust to do whatever math necessary to convert from ADU to percentage
        bt_stationData[3] = y;
        bt_stationData[4] = totalDistance;;
        bt_stationData[5] = loadvoltage;
        bt_stationData[6] = total_count; //particle count
        bt_stationData[7] = alert;//alert

        String acc_data = String(bt_stationData[2], 2) + "," +
                          String(bt_stationData[3], 2);
        pCharacteristic_ACC->setValue(acc_data.c_str());
        pCharacteristic_ACC->notify(); // Send the value to the app!
        Serial.print("*** Sent Accelerometer Data Value: ");
        Serial.print(acc_data);
        Serial.println(" ***");
    
        String dist_data = String(bt_stationData[4], 4);
        pCharacteristic_DIST->setValue(dist_data.c_str());
        pCharacteristic_DIST->notify(); // Send the value to the app!
        Serial.print("*** Sent Wheel Encoder Data Value: ");
        Serial.print(dist_data);
        Serial.println(" ***");
    
        String env_data = String(bt_stationData[5], 2) + "," + // battery voltage
                          String(bt_stationData[6], 2) + "," + // particle count
                          String(bt_stationData[7], 2);        // alert
        pCharacteristic_ENV->setValue(env_data.c_str());
        pCharacteristic_ENV->notify(); // Send the value to the app!
        Serial.print("*** Sent Env Data Value: ");
        Serial.print(env_data);
        Serial.println(" ***");
      
      }
      else if (rxValue.find(2) != -1) { // change direction
        if (backwardFlag == false){
          backwardFlag == true;
        }
        else{
          backwardFlag == false;
        }
      }
      else if (rxValue.find(3) != -1) {
        ESP.restart();
      }
      else if (rxValue.find(4) != -1) {
        if (unitFlag == false){
          unitFlag == true;
        }
        else{
          unitFlag == false;
        }
      }
      else if (rxValue.find(5) != -1) {
        //MANUAL TARE
        defaultX = 0.0; defaultY = 0.0; defaultZ = 0.0;
        Serial.print("BEGIN TARE");
        delay(1000);
        for (int i = 0; i < 50; i++){
          while(myIMU.dataAvailable() == false){delay(1);}   
          defaultX = defaultX + myIMU.getRawAccelX();
          defaultY = defaultY + myIMU.getRawAccelY();
          defaultZ = defaultZ + myIMU.getRawAccelZ();   
          Serial.print(i);
        }
        StoreFlag = 1;
        defaultX = defaultX/50.0; defaultY=defaultY/50.0; defaultZ=defaultZ/50.0; 
        Serial.println();
        Serial.println("*********");
        sendAlert(TARE_COMPLETE_ALERT);
      }
      else if (rxValue.find(6) != -1){
        wheelDiameter = rxValue[1];
        ticksPerRotation = rxValue[2];
      }
      else if (rxValue.find(7) != -1){
        grade_threshold = rxValue[1];
        x_slope_threshold = rxValue[2];
      }
    }
  }
};

void sendAlert(int alert_code){
  pCharacteristic_ALERT->setValue(alert_code.c_str());
  pCharacteristic_ACC->notify();
  return;
}

void call_delay(int time){
  delay(time);
  return;
}

void ForwardISR(){
  intTime = millis();
  //if front encoder triggers before back encoder
  if((digitalRead(wheelEncoderPin_forward) == LOW) &&  
        (intTime - lastTime > 150)) {
    if (!backwardFlag){
      wheelTicks++;
    }
    else{wheelTicks--;}
    if (wheelTicks % 8 == 0){
      readFlag = 1;
    }
    lastTime=intTime;
  }
}

void BackwardISR(){
//   intTime = millis();
//   //if back encoder triggers before front encoder
//   if((digitalRead(wheelEncoderPin_backward) == LOW) && 
//         (digitalRead(wheelEncoderPin_forward) == HIGH) && 
//         (intTime - lastTime > 150)) {
//     if (backwardFlag){
//       wheelTicks++;
//     }
//     else{
//       wheelTicks--;
//     }
//     if (wheelTicks % 8 == 0){
//       readFlag = 1;
//     }
//     lastTime=intTime;
//   }
}
// define wheel encoder interrupts
/* if forward wheel encoder is sensed first,
 * this pin (1) will go low first, then the 
 * backward pin will go low after, the cart
 * is moving forward in this case, and
 * distance should be increased */
// void IRAM_ATTR ForwardISR()
// {
//   intTime = millis();
//   if((digitalRead(wheelEncoderPin_forward) == LOW) && (intTime - lastTime > 150)){
//     if (!backwardFlag){wheelTicks++;}
//     else{wheelTicks--;}
//     if (wheelTicks % 8 == 0){
//       readFlag = 1;
//     }
    
//     lastTime=intTime;
//   }
// }

void setup() 
{
  // Set up communications 
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN); //Sets correct pins for GPS
  Wire.setPins(I2C_SDA,I2C_SCL);
  Wire.begin();

  //Encoder Setup
  #if HALL
    pinMode(wheelEncoderPin_forward, INPUT); //Sets Pin to INPUT
    attachInterrupt(digitalPinToInterrupt(wheelEncoderPin_forward), ForwardISR, FALLING); //Sets pin to call Forward service routine on Falling Edge
    pinMode(wheelEncoderPin_backward, INPUT); //Sets Pin to INPUT
    attachInterrupt(digitalPinToInterrupt(wheelEncoderPin_backward), BackwardISR, FALLING); //Sets pin to call Back service routine on Falling Edge
    ticksPerRotation = 8; //# mags
    wheelDiameter = 15.0;
    usingInches = true;
    wheelTicks = 0;   // could be a value if starting from a station
    unitConversion = 1;
    alpha = 1/ticksPerRotation;
    AccelZ = -2;
  #endif
  
  //GPS SETUP  
  #if GPS_FLAG
    // 9600 NMEA is recommended by Rylan
    if(GPS.begin(9600)){
      Serial.println("GPS Begin!");
    }
    else{
      Serial.println("GPS HALT!");
    }
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz
    delay(1000);
    // Ask for firmware version
    GPSSerial.println(PMTK_Q_RELEASE);
  #endif 

  //BNO SPARKY
  if(myIMU.beginSPI(imuCSPin, imuWAKPin, imuINTPin, imuRSTPin)){
    Serial.println("BNO08x Found!");
  }

  myIMU.enableLinearAccelerometer(10); //Send data update every 50ms
  myIMU.enableRawAccelerometer(10); //Send data update every 50ms
  
  //BME
  #if BME
    //start bme 
  #endif
  delay(100);

  //INA
  #if INA
    if (!ina219.begin()) 
    {
      Serial.println("Failed to find INA219 chip");
      while (1) 
      { 
        delay(10); 
      }
    }
  #endif

  //PM25Aqi
  #if PM25AQI
    if (! aqi.begin_I2C()) 
    {
      Serial.println("Could not find PM 2.5 sensor!");
      while (1) 
      {
        delay(10);
      }
    }
    delay(100);
  #endif

  // Blutooth Setup
  pinMode(LED, OUTPUT); // setting the RGB pin as the output

  // Create the BLE Device
  BLEDevice::init("TrailSense ESP32"); // Give the ESP32 a name
  //BLEDevice::setMTU(100);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create BLE Characteristics
  // Write to ESP Characteristic
  pCharacteristic_RX = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_RX,
                                        BLECharacteristic::PROPERTY_WRITE
                                      );
  pCharacteristic_RX->setCallbacks(new MyCallbacks());
  
  // Transmit GPS data Characteristic
  // pCharacteristic_GPS = pService->createCharacteristic(
  //                     CHARACTERISTIC_UUID_GPS,
  //                     BLECharacteristic::PROPERTY_NOTIFY
  //                   );

  // Transmit accelerometer data Characteristic
  pCharacteristic_ACC = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_ACC,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // // Transmit whel encoder data Characteristic
  pCharacteristic_DIST = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_DIST,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  //   // Transmit environmental sensors data Characteristic
  pCharacteristic_ENV = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_ENV,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic_ALERT = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_ALERT,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting for a client connection to notify...");
}


void loop() 
{
  //Encoder LOOP  
  if(forwardFlag) 
  { // reset flags
    forwardFlag = false;
    //backwardFlag = false;
  }
  if(unitFlag){
    unitConversion = 2.54;
  }

  #if HALL
    if(readFlag){
      prevDistance = totalDistance;
      totalDistance = ((wheelTicks / ticksPerRotation) * (wheelDiameter * PI) * unitConversion);
      station_distance += totalDistance - prevDistance;
      if(station_distance > 25){
        sendAlert(DISTANCE_ALERT);
      }
    } // read flag
  #endif  // hall
  
  //GPS LOOP
  #if GPS_FLAG
    char c = GPS.read(); //Read the data from the GPS
    if (GPSECHO){
      if (c) //Serial.print(c);
    }
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())){} // this also sets the newNMEAreceived() flag to false
        //return; // we can fail to parse a sentence in which case we should just wait for another
    }
    if (millis() - timer > 2000) {
      timer = millis(); // reset the timer
      if (GPS.fix) {
        lat = GPS.latitude;
        lon = GPS.longitude;
        lon_string = GPS.lon;
        lat_string = GPS.lat;
      }
      else{
        sendAlert(GPS_ALERT);
      }
    }
  #endif

  //BME readings 
  #if BME
    if (! bme.performReading()) 
    {
      // Serial.println("Failed to perform BME reading :(");
      return;
    }
    temp = (bme.temperature * (9/5)) + 32;    // will need option for f or c
    pressure = (bme.pressure / 100.0) * 30;
    humidity = bme.humidity;
  #endif

  //particle count
  #if PM25AQI 
    PM25_AQI_Data data;
    if (! aqi.read(&data)) 
    {    
      // Serial.println("Could not read from AQI");
      delay(500);  // try again in a bit!
      return;
    }
    particle_count = data.particles_03um + data.particles_05um + data.particles_10um;
    total_count += particle_count;
  #endif

  #if INA
    shuntvoltage = ina219.getShuntVoltage_mV();
    busvoltage = ina219.getBusVoltage_V();
    current_mA = ina219.getCurrent_mA();
    power_mW = ina219.getPower_mW();
    loadvoltage = busvoltage + (shuntvoltage / 1000);
    total_loadvoltage += loadvoltage;
    total_current += current_mA;
  #endif
}