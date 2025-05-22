#include <Wire.h>
#include <Servo.h>

// ===== Parking System =====
Servo myservo;

int IR1 = 7;           // Entry sensor on pin 7
int IR2 = 8;           // Exit sensor on pin 8

int servoPin = 9;      // Servo connected to pin 9
int totalSlots = 2;    // Total parking slots
int Slot = totalSlots;

int flag1 = 0;
int flag2 = 0;

bool isGateOpen = false;

// ===== Irrigation System =====
const int soilSensorPin = A0;
const int pumpPin = 10;
const int dryThreshold = 600;  // Adjust based on soil moisture sensor

unsigned long previousMoistureMillis = 0;
const long moistureInterval = 2000; // Every 2 seconds

void setup() {
  Serial.begin(9600);

  // Sensor setup
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);

  // Servo setup (initially closed)
  myservo.attach(servoPin);
  myservo.write(0);
  isGateOpen = false;

  // Pump setup
  pinMode(pumpPin, OUTPUT);
  digitalWrite(pumpPin, LOW); // Pump off

  Serial.println("🔧 System Initialized (Parking + Irrigation)");
  delay(1000);
}

void loop() {
  // ===== Parking System =====
  if (digitalRead(IR1) == LOW && flag1 == 0) {
    if (Slot > 0) {
      flag1 = 1;
      if (flag2 == 0) {
        myservo.write(90);   // Open gate
        isGateOpen = true;
        Slot--;

        Serial.println("🚗 Car Entering...");
        delay(2000);  // Shortened delay

        myservo.write(0);    // Close gate
        isGateOpen = false;

        flag1 = 0;
        flag2 = 0;
      }
    } else {
      Serial.println("❌ Parking Full");
      delay(1000);  // Shortened delay
    }
  }

  if (digitalRead(IR2) == LOW && flag2 == 0) {
    flag2 = 1;
    if (flag1 == 0) {
      myservo.write(90);   // Open gate
      isGateOpen = true;
      Slot++;

      Serial.println("🚙 Car Exiting...");
      delay(2000);  // Shortened delay

      myservo.write(0);    // Close gate
      isGateOpen = false;

      flag1 = 0;
      flag2 = 0;
    }
  }

  // Display parking status
  int carsInside = totalSlots - Slot;
  Serial.print("Slots Left: ");
  Serial.print(Slot);
  Serial.print(" | Cars Inside: ");
  Serial.print(carsInside);
  Serial.print(" | Gate Open: ");
  Serial.println(isGateOpen ? "YES" : "NO");

  // ===== Irrigation System =====
  unsigned long currentMillis = millis();
  if (currentMillis - previousMoistureMillis >= moistureInterval) {
    previousMoistureMillis = currentMillis;

    int moisture = analogRead(soilSensorPin);
    Serial.print("Soil Moisture: ");
    Serial.println(moisture);

    if (moisture > dryThreshold) {
      digitalWrite(pumpPin, HIGH);
      Serial.println("⚠ Soil is dry - Pump ON");
    } else {
      digitalWrite(pumpPin, LOW);
      Serial.println("✅ Soil is moist - Pump OFF");
    }
  }

  delay(200); // Small delay to reduce CPU load
}
