/*
#include <Arduino.h>

#define rightChannel 10
#define leftChannel 9

void forward(unsigned n) {
    digitalWrite(7, HIGH);
    digitalWrite(6, LOW);
    digitalWrite(4, HIGH);
    digitalWrite(5, LOW);
    analogWrite(rightChannel, n);
    analogWrite(leftChannel, n+4);
}

void backward(unsigned n) {
    digitalWrite(6, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(5, HIGH);
    digitalWrite(4, LOW);
    analogWrite(rightChannel, n);
    analogWrite(leftChannel, n-4);
}

void setup() {
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
}

void loop() {
    backward(100);
}

*/
