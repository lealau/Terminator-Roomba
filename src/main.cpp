#include <Arduino.h>
#include <MPU9250.h>
#include <Wire.h>

#define baudRate 115200

class Roomba {
private:
    // Pin assignments
    // Photodiode RIGHT to LEFT 0 to 3
    const uint8_t photodiode0 = A0;
    const uint8_t photodiode1 = A1;
    const uint8_t photodiode2 = A2;
    const uint8_t photodiode3 = A3;
    const uint8_t leftChannel = 9;
    const uint8_t rightChannel = 10;

    // Default bot configurations
    uint8_t m_speed = 100;
    MPU9250* IMU;

    int beaconAngle = 0;

protected:
    void updateIMU() { IMU->readSensor(); }

    void forwardRight()  { digitalWrite(6, LOW);  digitalWrite(7, HIGH); }
    void forwardLeft()   { digitalWrite(4, HIGH); digitalWrite(5, LOW); }
    void backwardRight() { digitalWrite(6, HIGH); digitalWrite(7, LOW); }
    void backwardLeft()  { digitalWrite(4, LOW);  digitalWrite(5, HIGH); }

    void forwardConfig() {
        forwardRight();
        forwardLeft();
    }

    void backwardConfig() {
        backwardRight();
        backwardLeft();
    }

public:
    /* Constructor */
    Roomba(MPU9250* imu) {
        IMU = imu;
    }

    /* setup() Function */
    void begin() {
        // Initialize the serial port
        Serial.begin(baudRate);
        
        
        // // Initialize and configure IMU
        // Serial.println("initializing IMU...");
        // int imu_status = IMU->begin();
        // if (imu_status < 0) {
        //     Serial.print("unable to initialize IMU! Status code: ");
        //     Serial.println(imu_status);
        //     while(1);
        // } else {
        //     Serial.print("IMU initialized with status code: ");
        //     Serial.println(imu_status);
        // }
        // IMU->setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    }

    /* Motor Controls */
    void moveForward() {
        forwardConfig();
        analogWrite(rightChannel, m_speed);
        analogWrite(leftChannel, m_speed+4);
    }

    void moveBackward() {
        backwardConfig();
        analogWrite(rightChannel, m_speed);
        analogWrite(leftChannel, m_speed-4);
    }

    void stopTurn() {
        analogWrite(rightChannel, 0);
        analogWrite(leftChannel, 0);
        
        forwardRight();
        backwardLeft();
        analogWrite(rightChannel, 100);
        analogWrite(leftChannel, 100);
    }
    

    bool beaconFound() {
        int threshold = 50;
        int i0 = analogRead(photodiode0);
        int i1 = analogRead(photodiode1);
        int i2 = analogRead(photodiode2);
        int i3 = analogRead(photodiode3);

        if (i0 > threshold || i1 > threshold || i2 > threshold || i3 > threshold) {
            return true;
        }
        return false;
    }


    float readCenterIR() {
        int i1 = analogRead(photodiode1);
        int i2 = analogRead(photodiode2);

        return (i1 + i2) / 2;
    }

    int calculateBeaconAngle() {
        int c = readCenterIR();
        int r = readRightIR();

        return ((r - c) / ((r + c) / 2)) * 1024;
    }


    float readRightIR() {
        int i0 = analogRead(photodiode0);
        return i0;
    }
    
    float readLeftIR() {
        int i2 = analogRead(photodiode2);
        int i3 = analogRead(photodiode3);
    }


    float readIR() {
        float right = readRightIR();
        float left = readLeftIR();
        return right - left;
    }

    // setSpeed sets the target straight-line motor speed between 0-255.
    void setSpeed(uint8_t speed) {
        if (speed < 80) {
            m_speed = 0;
        } else if (speed > 200) {
            m_speed = 200;
        } else {
            m_speed = speed;
        }
    }
};

MPU9250 IMU(Wire, 0x68);
Roomba Terminator(&IMU);

void setup() {
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);

    Terminator.begin();
}

void loop() {
    Serial.print("L: ");
    Serial.print(Terminator.readLeftIR());
    Serial.print(" \t");

    Serial.print("R: ");
    Serial.print(Terminator.readRightIR());
    Serial.print(" \t");

    Serial.print("C: ");
    Serial.println(Terminator.readIR());

    delay(100);
}

