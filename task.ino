#include <avr/io.h>
#include <avr/interrupt.h>

#define LED1_PIN 13
#define LED2_PIN 12
#define LED3_PIN 11
#define LED4_PIN 10

#define PIR_PIN 2
#define TEMP_SENSOR_PIN A0
#define SOIL_MOISTURE_PIN A1

volatile bool pirTriggered = false;
volatile int temperatureValue = 0;
volatile int soilMoistureValue = 0;

void setup() {
  // Configure LED pins as output
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);

  // Configure PIR and soil moisture pins as input
  pinMode(PIR_PIN, INPUT);
  pinMode(SOIL_MOISTURE_PIN, INPUT);

  // Configure Timer1 for sensor readings and LED control
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1B |= (1 << WGM12); // Set CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set prescaler 1024
  OCR1A = 312; // 100 ms with 1024 prescaler
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 compare interrupt

  // Configure Timer0 for LED2 blinking
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0B |= (1 << CS02) | (1 << CS00); // Set prescaler 1024
  OCR0A = 1990; // 625 ms with 1024 prescaler
  TIMSK0 |= (1 << OCIE0A); // Enable Timer0 compare interrupt

  // Enable Pin Change Interrupt for PIR sensor (PCINT18)
  PCMSK2 |= (1 << PCINT18); // PIR_PIN
  PCICR |= (1 << PCIE2); // Enable Pin Change Interrupt 2

  // Enable global interrupts
  sei(); 
  Serial.begin(9600); // Initialize serial communication
}

void loop() {
  // This function is empty as all tasks are handled by Timer1 and Timer0 interrupts
}

// Timer1 ISR for sensor readings and LED control
ISR(TIMER1_COMPA_vect) {
  // Read temperature sensor value
  int rawValue = analogRead(TEMP_SENSOR_PIN);
  float voltage = rawValue * 5.0 / 1023.0;
  float temperatureC = (voltage - 0.5) * 100.0;
  temperatureValue = (int)temperatureC;

  // Control LED4 based on temperature value
  if (temperatureValue > 30) {
    digitalWrite(LED4_PIN, HIGH); // Turn on LED4 if temperature is above 30°C
  } else {
    digitalWrite(LED4_PIN, LOW); // Turn off LED4 if temperature is below or equal to 30°C
  }

  // Read soil moisture sensor value
  soilMoistureValue = analogRead(SOIL_MOISTURE_PIN);

  // Control LED3 based on soil moisture value
  if (soilMoistureValue > 500) {
    digitalWrite(LED3_PIN, HIGH); // Turn on LED3 if soil moisture is above 500
  } else {
    digitalWrite(LED3_PIN, LOW); // Turn off LED3 if soil moisture is below or equal to 500
  }

  // If motion detected, turn on LED1 for 2 seconds
  if (pirTriggered) {
    digitalWrite(LED1_PIN, HIGH); // Turn on LED1 when motion detected
    delay(2000); // Keep LED1 on for 2 seconds
    digitalWrite(LED1_PIN, LOW); // Turn off LED1
    pirTriggered = false; // Reset the motion detection flag
  }
}

// Timer0 ISR for LED2 blinking
ISR(TIMER0_COMPA_vect) {
  static bool led2State = LOW; // Initialize LED2 state
  digitalWrite(LED2_PIN, led2State); // Set LED2 state
  led2State = !led2State; // Toggle LED2 state
  delay(9000); // Wait for 9 seconds
  Serial.println("TIMER0_COMPA_vect triggered"); // Debug statement
}

// Pin Change Interrupt ISR for PIR sensor
ISR(PCINT2_vect) {
  if (digitalRead(PIR_PIN) == HIGH) {
    pirTriggered = true; // Set motion detection flag when PIR sensor detects motion
    digitalWrite(LED1_PIN, HIGH); // Turn on LED1 when motion detected
  }
}
