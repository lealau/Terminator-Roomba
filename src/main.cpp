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

protected:
    void updateIMU() {
        IMU->readSensor();
    }

    void forwardRight() {
        digitalWrite(6, LOW);
        digitalWrite(7, HIGH);
    }

    void forwardLeft() {
        digitalWrite(4, HIGH);
        digitalWrite(5, LOW);
    }

    void backwardRight() {
        digitalWrite(6, HIGH);
        digitalWrite(7, LOW);
    }

    void backwardLeft() {
        digitalWrite(4, LOW);
        digitalWrite(5, HIGH);
    }

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
        
        // Initialize and configure IMU
        Serial.println("initializing IMU...");
        int imu_status = IMU->begin();
        if (imu_status < 0) {
            Serial.print("unable to initialize IMU! Status code: ");
            Serial.println(imu_status);
            while(1);
        } else {
            Serial.print("IMU initialized with status code: ");
            Serial.println(imu_status);
        }
        IMU->setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    }

    /* Motor Controls */
    void forward() {
        forwardConfig();
        analogWrite(rightChannel, m_speed);
        analogWrite(leftChannel, m_speed+4);
    }

    void backward() {
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

    float readRightIR() {
        int i0 = analogRead(photodiode0);
        int i1 = analogRead(photodiode1);
        if (i1 > i0) {
            return i1 / 2.0F;
        } else {
            return i0;
        }
        
    }
    
    float readLeftIR() {
        int i2 = analogRead(photodiode2);
        int i3 = analogRead(photodiode3);
        if (i2 > i3) {
            return i2 / 2.0F;
        } else {
            return i3;
        }
    }


    float readIR() {
        float right = readRightIR();
        float left = readLeftIR() * -1;

        return right + left;
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

    // getSpeed returns the current straight-line motor speed
    unsigned getSpeed() {
        return (unsigned)m_speed;
    }

    // Print with `Serial.print(Roomba.getAccelX(), 6)`
    float getAccelX() {
        updateIMU();
        return IMU->getAccelX_mss();
    }

    // Print with `Serial.print(Roomba.getAccelY(), 6)`
    float getAccelY() {
        updateIMU();
        return IMU->getAccelY_mss();
    }

    // Print with `Serial.print(Roomba.getAccelZ(), 6)`
    float getAccelZ() {
        updateIMU();
        return IMU->getAccelZ_mss();
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
    Serial.print(Terminator.readLeftIR() * -1);
    Serial.print("\t");

    Serial.print("R: ");
    Serial.print(Terminator.readRightIR());
    Serial.print("\t");

    Serial.print("C: ");
    Serial.println(Terminator.readIR());

    delay(100);
}

