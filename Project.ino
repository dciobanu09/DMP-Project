#include "Servo.h"
#include <TimerOne.h>

volatile Servo myServo;

volatile int needWater = 0;
// volatile int soilAverage = 0;
volatile int pourSomeWater = 0;
volatile int waterLevel = 0;

#define WATER_LEVEL_SIGNAL_PIN A0
#define WATER_LEVEL_POWER_PIN 7

#define SOIL_HUMIDITY_SIGNAL_PIN A1
#define SOIL_HUMIDITY_POWER_PIN 6

#define SERVO_CONTROL_PIN 9

#define DHT_PIN 5
#define LED_PIN 2
#define INTERRUPT_BUTTON 2

void setup() {
  Serial.begin(9600);
  pinMode(WATER_LEVEL_POWER_PIN, OUTPUT);
  pinMode(SOIL_HUMIDITY_POWER_PIN, OUTPUT);
  pinMode(DHT_PIN, INPUT);
  pinMode(INTERRUPT_BUTTON, INPUT_PULLUP);
  digitalWrite(WATER_LEVEL_POWER_PIN, LOW);
  digitalWrite(SOIL_HUMIDITY_POWER_PIN, LOW);
  digitalWrite(SERVO_CONTROL_PIN, LOW);
  digitalWrite(13, LOW);
  Timer1.initialize(100000000);
  Timer1.attachInterrupt(verify);
}

void loop() {
  digitalWrite(SOIL_HUMIDITY_SIGNAL_PIN, HIGH);
  digitalWrite(WATER_LEVEL_POWER_PIN, HIGH);
  
  if (waterLevel < 500) {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }

  for (int i = 0; i < 5; i++) {
    wait_for_dht11();
    start_signal(5);
    read_dht11(5);
  }

  if (needWater == 1 && waterLevel > 500) {
    delay(100);
    myServo.attach(SERVO_CONTROL_PIN);
    delay(100);
    Serial.println("Dry");
    int currentIndex = 0;
    int soilHumidity = 221;
    for (int i = 0; i < 180 && soilHumidity > 220; i++) {
      currentIndex++;
      soilHumidity = analogRead(SOIL_HUMIDITY_SIGNAL_PIN);
      Serial.println(soilHumidity);
      myServo.write(i);
      delay(15);
    }
    for (int i = currentIndex; i >= 0; i--) {
      myServo.write(i);
      delay(15);
    }
    myServo.detach();
    needWater = 0;
  }

  delay(1000);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_BUTTON), pourWater, RISING);
  for (int i = 0; i < 100; i++) {
    Serial.print("Water level: ");
    Serial.println(analogRead(WATER_LEVEL_SIGNAL_PIN));
    delay(10);
  }
  detachInterrupt(digitalPinToInterrupt(INTERRUPT_BUTTON));

  if (pourSomeWater == 1) {
    myServo.attach(SERVO_CONTROL_PIN);
    for (int i = 0; i < 180; i++) {
      myServo.write(i);
      delay(15);
    }
    for (int i = 179; i >= 0; i--) {
      myServo.write(i);
      delay(15);
    }
    myServo.detach();
    delay(50);
    pourSomeWater = 0;
    delay(50);
  }

  delay(1000);
  Timer1.attachInterrupt(verify);
  delay(10);
  waterLevel = analogRead(WATER_LEVEL_SIGNAL_PIN);
  digitalWrite(WATER_LEVEL_POWER_PIN, LOW);
  digitalWrite(SOIL_HUMIDITY_POWER_PIN, LOW);

  delay(1000);
}

void pourWater() {
  delay(50);
  pourSomeWater = 1;
  delay(50);
}

void verify() {
  Timer1.detachInterrupt();
  int soilAverage = 0;
  for (int i = 0; i < 100; i++) {
    int soilHumidity = analogRead(SOIL_HUMIDITY_SIGNAL_PIN);
    soilAverage += soilHumidity;
    Serial.print("SOIL: ");
    Serial.println(soilHumidity);
  }
  Serial.println("SOILTOTAL: ");
  Serial.println(soilAverage);
  if ((soilAverage / 100) > 220) {
    needWater = 1;
    delay(200);
  }
}

void dec2bin(int n) {
  int c, k;

  for (c = 15; c >= 0; c--) {
    k = n >> c;

    if (k & 1)
      Serial.print("1");
    else
      Serial.print("0");
  }
}

void dec2bin8(int n) {
  int c, k;

  for (c = 7; c >= 0; c--) {
    k = n >> c;

    if (k & 1)
      Serial.print("1");
    else
      Serial.print("0");
  }
}


void wait_for_dht11() {
  delay(2000);
}

void start_signal(uint8_t dht11_pin) {
  pinMode(dht11_pin, OUTPUT);
  digitalWrite(dht11_pin, LOW);
  delay(18);
  digitalWrite(dht11_pin, HIGH);
  pinMode(dht11_pin, INPUT);
  digitalWrite(dht11_pin, HIGH);
}

void read_dht11(uint8_t dht11_pin) {
  uint16_t rawHumidity = 0;
  uint16_t rawTemperature = 0;
  uint8_t checkSum = 0;
  uint16_t data = 0;

  uint8_t humi;
  uint8_t humd;
  uint8_t tempi;
  uint8_t tempd;

  unsigned long startTime;

  for (int8_t i = -3; i < 80; i++) {
    byte live;
    startTime = micros();

    do {
      live = (unsigned long)(micros() - startTime);
      if (live > 90) {
        Serial.println("ERROR_TIMEOUT");
        return;
      }
    } while (digitalRead(dht11_pin) == (i & 1) ? HIGH : LOW);

    if (i >= 0 && (i & 1)) {
      data <<= 1;

      // TON of bit 0 is maximum 30 usecs and of bit 1 is at least 68 usecs.
      if (live > 30) {
        data |= 1;  // we got a one
      }
    }

    switch (i) {
      case 31:
        rawHumidity = data;
        break;
      case 63:
        rawTemperature = data;
      case 79:
        checkSum = data;
        data = 0;
        break;
    }
  }

  Serial.println("Humidity: ");
  dec2bin(rawHumidity);
  Serial.print("\t");
  humi = rawHumidity >> 8;
  dec2bin8(humi);
  Serial.print("\t");
  rawHumidity = rawHumidity << 8;
  humd = rawHumidity >> 8;
  dec2bin8(humd);
  Serial.print("\t");
  Serial.print(humi);
  Serial.print(".");
  Serial.print(humd);
  Serial.print("%");
  Serial.println("");

  Serial.println("Temperature Degree Celsius: ");
  dec2bin(rawTemperature);
  Serial.print("\t");
  tempi = rawTemperature >> 8;
  dec2bin8(tempi);
  Serial.print("\t");
  rawTemperature = rawTemperature << 8;
  tempd = rawTemperature >> 8;
  //tempd = (byte)rawTemperature;
  dec2bin8(tempd);
  Serial.print("\t");
  Serial.print(tempi);
  Serial.print(".");
  Serial.print(tempd);
  Serial.print("C");
  Serial.println("");

  Serial.println("Checksum Byte: ");
  dec2bin8(checkSum);
  Serial.println("");
  dec2bin8(tempi + tempd + humi + humd);
  Serial.println("");
  if ((byte)checkSum == (byte)(tempi + tempd + humi + humd)) {
    Serial.print("CHECKSUM_OK");
  } else {
    Serial.print("CHECKSUM_ERROR");
  }
  Serial.println("");
  Serial.println("");
  Serial.println("");
}