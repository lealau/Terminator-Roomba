/*
#include <Arduino.h>
#include <MPU9250.h>

#define baudRate 115200

MPU9250 IMU(Wire, 0x68);
int imu_status;

void setup() {
    Serial.begin(baudRate);
    Serial.println("initializing IMU...");

    imu_status = IMU.begin();
    if (imu_status < 0) {
        Serial.print("unable to initialize IMU! Status code: ");
        Serial.println(imu_status);
    } else {
        Serial.print("initialized with status code: ");
        Serial.println(imu_status);
    }
}

void loop() {
    IMU.readSensor();
    
    Serial.print("X: ");
    Serial.print(IMU.getAccelX_mss(),6);
    Serial.print("\t");

    Serial.print("Y: ");
    Serial.print(IMU.getAccelY_mss(),6);
    Serial.print("\t");

    Serial.print("Z: ");
    Serial.println(IMU.getAccelZ_mss(),6);
    delay(1000);
}
*/