// #include <Arduino.h>
// #include <PID_v1.h>
// // #include <MPU9250.h>

// #define baudRate 115200
// #define diode0 A0
// #define diode1 A1
// #define diode2 A2
// #define diode3 A3
// #define leftChannel 9
// #define rightChannel 10

// int offset0, offset1, offset2, offset3;
// int rightSpeed = 100;
// int leftSpeed = 100;
// double setpoint, input, output;
// double Kp = 1, Ki = 0, Kd = 0;

// // MPU9250 IMU(Wire, 0x68);
// PID pid(&input, &output, &setpoint, Kp, Ki, Kd, REVERSE);

// #pragma region Movement
// void forwardRight()  { digitalWrite(6, LOW);  digitalWrite(7, HIGH); }
// void forwardLeft()   { digitalWrite(4, HIGH); digitalWrite(5, LOW); }
// void backwardRight() { digitalWrite(6, HIGH); digitalWrite(7, LOW); }
// void backwardLeft()  { digitalWrite(4, LOW);  digitalWrite(5, HIGH); }

// void forwardConfig() {
//     forwardRight();
//     forwardLeft();
// }

// void backwardConfig() {
//     backwardRight();
//     backwardLeft();
// }

// void moveForward(int rightSpeed, int leftSpeed) {
//     forwardConfig();
//     analogWrite(rightChannel, rightSpeed);
//     analogWrite(leftChannel, leftSpeed);
// }

// void moveBackward(int rightSpeed, int leftSpeed) {
//     backwardConfig();
//     analogWrite(rightChannel, rightSpeed);
//     analogWrite(leftChannel, leftSpeed);
// }

// void stop() {
//     forwardConfig();
//     analogWrite(rightChannel, 0);
//     analogWrite(leftChannel, 0);
// }

// void hardTurnLeft() {
//     forwardRight();
//     backwardLeft();
//     analogWrite(rightChannel, rightSpeed);
//     analogWrite(leftChannel, leftSpeed);
// }

// void hardTurnRight() {
//     forwardLeft();
//     backwardRight();
//     analogWrite(rightChannel, rightSpeed);
//     analogWrite(leftChannel, leftSpeed);
// }
// #pragma endregion Movement

// #pragma region Calibrated Reading
// int readLeftIR() {
//     return analogRead(diode3) - offset3;
// }

// int readCenterLeftIR() {
//     return analogRead(diode2) - offset2;
// }

// int readCenterRightIR() {
//     return analogRead(diode1) - offset1;
// }

// int readRightIR() {
//     return analogRead(diode0) - offset0;
// }

// float leftSideAvg() {
//     return (readCenterLeftIR() + readLeftIR()) / 2.F;
// }

// float rightSideAvg() {
//     return (readCenterRightIR() + readRightIR()) / 2.F;
// }

// float deviation() {
//     return rightSideAvg() - leftSideAvg();
// }
// #pragma endregion Calibrated Reading

// #pragma region Beacon Finding
// void calibrate() {
//     // IR Calibration
//     offset0 = analogRead(diode0);
//     offset1 = analogRead(diode1);
//     offset2 = analogRead(diode2);
//     offset3 = analogRead(diode3);
//     Serial.println("IR Photodiodes Calibrated.");
// }

// bool beaconFound() {
//     int left        = readLeftIR();
//     int centerLeft  = readCenterLeftIR();
//     int centerRight = readCenterRightIR();
//     int right       = readRightIR();

//     int threshold = 5;

//     if (left > threshold || centerLeft > threshold || centerRight > threshold || right > threshold) {
//         return true;
//     }
//     return false;
// }

// bool beaconRight() {
//     return deviation() > 0;
// }

// bool beaconLeft() {
//     return deviation() < 0;
// }
// #pragma endregion Beacon Finding

// void setup() {
//     Serial.begin(baudRate);
    
    
//     // PID setup
//     input = deviation();
//     setpoint = 0;
//     pid.SetOutputLimits(-40, 40);
//     pid.SetMode(AUTOMATIC);
//     Serial.println("PID Controller Initialized.");

//     // Move to Center
//     Serial.println("Bot Moving");
//     moveBackward(rightSpeed, leftSpeed);
//     delay(1500);
//     calibrate();
//     hardTurnLeft();
//     delay(500);
//     unsigned startTime = millis();
//     while (millis() - startTime < 2700 && !beaconFound()) {
//         moveForward(rightSpeed, leftSpeed);
//     }
//     Serial.println("Bot Stopping");
// }

// void loop() {
//     input = deviation();
//     pid.Compute();

//     #pragma region Beacon Testing
//     Serial.print("L:  "); Serial.print(readLeftIR()); Serial.print("\t");
//     Serial.print("CL: "); Serial.print(readCenterLeftIR()); Serial.print("\t");
//     Serial.print("CR: "); Serial.print(readCenterRightIR()); Serial.print("\t");
//     Serial.print("R:  "); Serial.print(readRightIR()); Serial.print("\t");
//     Serial.print("D:  "); Serial.print(deviation()); Serial.print("\t");
//     Serial.print("O:  "); Serial.print(output); Serial.print("\t");
//     if (beaconFound()) {
//         Serial.print("BEACON");
//         if (beaconRight()) {
//             Serial.println(" RIGHT --> ");
//         } else if (beaconLeft()) {
//             Serial.println(" <-- LEFT ");
//         } else {
//             Serial.println(" -- CENTER -- ");
//         }
//     } else {  
//         Serial.println();
//     }
//     #pragma endregion Beacon Testing

//     moveForward(255, 240);
//     if (beaconFound()) {
//         moveForward(rightSpeed - output, leftSpeed + output);
//     }

// //     if (!beaconFound()) {
// //         hardTurnLeft();
// //         delay(250);
// //         stop();
// //     }

// //     while (!beaconFound()) {
// //         moveForward(rightSpeed, leftSpeed);
// //         delay(2000);
// //         hardTurnRight();
// //         delay(500);
// //     }

// //     if (beaconFound()) {
// //         moveForward(rightSpeed - output, leftSpeed + output);
// //     } else {
// //         stop();
// //     }


//     delay(10);
// }


