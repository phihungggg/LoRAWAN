#include <Arduino.h>
#include <HardwareSerial.h>
#include <DHT.h>

// Function prototypes
void sendATCommand(const char* command);
void resetRa08H();
void receiveCommand(float temperature);
void sendTemperatureHumidityData(float temperature, float humidity);
void sendOtherData(int LED);
void processReceivedData(String data, float temperature);

// Define constants
#define RXD2 16
#define TXD2 17
#define RST_PIN 5   // GPIO pin connected to RST pin of Ra-08H
#define LED_PIN 23  // GPIO pin connected to the LED
#define BUZZER_PIN 18 // GPIO pin connected to the Buzzer

// Global variables
HardwareSerial mySerial(2);  // Use UART2
bool newDataAvailable = false; // Flag to indicate new data received
int counter = 0;               // Counter to track intervals
DHT dht;

void setup() {
  pinMode(RST_PIN, OUTPUT);
  dht.setup(4); // data pin 4
  digitalWrite(RST_PIN, HIGH);  // Ensure RST pin starts in HIGH state
  delay(100);  // Optional delay for stabilization

  // Initialize the built-in serial port for debugging
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for Serial to initialize
  }
  Serial.println("ESP32 started");

  // Initialize the LED pin as an output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Ensure LED starts in the OFF state

  // Initialize the Buzzer pin as an output
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, HIGH);  // Ensure Buzzer starts in the OFF state

  // Initialize UART2 for communication with the Ra-08H module
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("HardwareSerial started on UART2 at 9600 baud rate");

  // Send initial AT commands to configure the Ra-08H module
  sendATCommand("AT+CJOINMODE=1\r\n");
  delay(500);
  sendATCommand("AT+CDEVADDR=260B113D\r\n");
  delay(500);
  sendATCommand("AT+CAPPSKEY=81AC8EC2855479A5C7AD487A3458BF7A\r\n");
  delay(500);
  sendATCommand("AT+CNWKSKEY=9B6F9061A32A336442733D6DF9E6ED04\r\n");
  delay(500);
  sendATCommand("AT+CFREQBANDMASK=0003\r\n");
  delay(500);
  sendATCommand("AT+CULDLMODE?\r\n");
  delay(500);
  sendATCommand("AT+CULDLMODE=2\r\n");
  delay(500);
  sendATCommand("AT+CWORKMODE=2\r\n");
  delay(500);
  sendATCommand("AT+CCLASS=0\r\n");
  delay(500);
  sendATCommand("AT+CBL=?\r\n");
  delay(500);
  sendATCommand("AT+CBL?\r\n");
  delay(500);
  sendATCommand("AT+CSTATUS=?\r\n");
  delay(500);
  sendATCommand("AT+CSTATUS?\r\n");
  delay(500);
  sendATCommand("AT+CJOIN=1,1,8,8\r\n");
  delay(500);
  resetRa08H();
}

void loop() {
  sendATCommand("AT+CRX1DELAY=1\r\n");
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();

  // Check for incoming data from the Ra-08H (UART2)
  receiveCommand(temperature);

  // Increment counter every second
  delay(500);
  counter++;
  // Send data every 10 seconds
  if (counter >= 20) {
    delay(dht.getMinimumSamplingPeriod());
    Serial.print(dht.getStatusString());
    Serial.print("\t");
    Serial.print(humidity, 1);
    Serial.print("\t\t");
    Serial.print(temperature, 1);
    Serial.print("\t\t");
    Serial.println(dht.toFahrenheit(temperature), 1);
    sendTemperatureHumidityData(temperature, humidity);
    // Reset counter
    counter = 0;
  }
  // Print LED status
  if (digitalRead(LED_PIN) == HIGH) {
    sendOtherData(1);
  } else {
    sendOtherData(0);
  }
}

void sendTemperatureHumidityData(float temperature, float humidity) {
  // Convert to integers
  float temp_int = temperature * 100;
  float hum_int = humidity * 100;

  // Build the payload with identifier 0
  char payload[19];
  sprintf(payload, "00%04X%04X", (unsigned int)temp_int, (unsigned int)hum_int); // Convert to hex string

  // Send the payload using AT+DTRX command
  char command[50];
  sprintf(command, "AT+DTRX=0,2,%d,%s\r\n", 9, payload); // Payload length is 9 bytes

  sendATCommand(command);
}

void sendOtherData(int LED) {
  // Example other data
  const char* other_data = ""; // Example data

  // Build the payload with identifier 1
  char payload[5];
  sprintf(payload, "01%02X", LED); // Convert to hex string

  // Send the payload using AT+DTRX command
  char command[50];
  sprintf(command, "AT+DTRX=1,1,%d,%s\r\n", 4, payload); // Payload length is 1 byte

  sendATCommand(command);
}

void resetRa08H() {
  digitalWrite(RST_PIN, LOW);  // Set RST pin LOW to reset Ra-08H
  delay(500);  // Hold LOW for a short period (adjust as needed)
  digitalWrite(RST_PIN, HIGH);   // Set RST pin HIGH to release reset
}

void sendATCommand(const char* command) {
  Serial.print("Sending: ");
  Serial.println(command);
  mySerial.print(command);
  delay(100); // Wait for the command to be sent
}

void receiveCommand(float temperature) {
  // Check for incoming data from UART2 (Ra-08H)
  while (mySerial.available()) {
    String response = mySerial.readStringUntil('\n'); // Read until newline character
    Serial.print("Received: ");
    Serial.println(response); 
    processReceivedData(response, temperature); // Process the received data
  }
}

void processReceivedData(String data, float temperature) {
  // Remove any whitespace from the received data
  data.trim();

  // Check if the received data ends with "0B05"
  if (data.endsWith("0B05")) {
    // Toggle the LED state
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  } else if (data.equals("OK+RECV:02,00,00")) {
    // Do nothing if the data is "OK+RECV:02,00,00"
    return;
  } else {
    // Process other received data if needed
    if (data.startsWith("OK+RECV")) {
      // Extract the last 4 characters from the string
      String hexValue = data.substring(data.length() - 2);

      // Convert the hex value to a decimal integer
      long receivedThreshold = strtol(hexValue.c_str(), NULL, 16);

      // Print the received value
      Serial.print("Received value: ");
      Serial.println(receivedThreshold);

      // Check if the received threshold is less than the current temperature
      if (temperature > receivedThreshold) {
        // Activate the buzzer if the temperature exceeds the received threshold
        digitalWrite(BUZZER_PIN, LOW);
      } else {
        // Deactivate the buzzer otherwise
        digitalWrite(BUZZER_PIN, HIGH);
      }
    }
  }
}
