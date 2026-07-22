#include <Arduino.h>

#define FAN_PIN 17

void setup() {
    Serial.begin(115200);
    delay(1000);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW); // starting off
    Serial.println("*** FAN TEST ON/OFF ***");
}

void loop() {
    Serial.println("Turning fan ON, get ready!");
    digitalWrite(FAN_PIN, HIGH);
    delay(4000);

    Serial.println("Turning fan OFF :(");
    digitalWrite(FAN_PIN, LOW);
    delay(4000);
}