
// SEE EXEMPLE_MPU6050 FOR THE CONFIG, THIS FILE IS A COPY PASTE OF IT MOSTLY, I'VE JUST ADDED MQTT STUFF AND REMOVED UNUSED LINE

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "WiFi.h"
#include <WiFiUdp.h>
#include <OSCBundle.h>

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Wifi & UDP configuration

const char *ssid = "SuperMango";
const char *password = "goodlife";
const char *topic = "GhostSong";
String client_id = "esp32-client-" + String(WiFi.macAddress());

const char * udpAddress = "192.168.8.101";
const int udpPort = 1234;
WiFiUDP udp;
WiFiClient espClient;

void setup() {
// join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties

  Serial.begin(9600);

 // Connecting to Wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to the Wi-Fi network");

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  // SEE IMU_ZERO to get these values (it's the last four of the final output)
  mpu.setXGyroOffset(55);
  mpu.setYGyroOffset(-4);
  mpu.setZGyroOffset(64);
  mpu.setZAccelOffset(2864);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();

    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {  
  OSCBundle bndl;
  int size;

  bndl.add("/rnbo/inst/0/params/play-button").add(0.5);

  udp.beginPacket(udpAddress, udpPort);  
  bndl.send(udp);
  udp.endPacket();

  delay(2000);
}
