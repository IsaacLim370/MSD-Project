#define BLYNK_TEMPLATE_ID "TMPL6wqXXT28f"
#define BLYNK_TEMPLATE_NAME "SmartLab Bin"
#define BLYNK_AUTH_TOKEN "HVAmF9VNDbWogNeIfe-fgQDqkaVHY-eE"
#define BLYNK_PRINT Serial

// Load Wi-Fi library
#include <esp_now.h>
#include <ESP_FlexyStepper.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char auth[] = BLYNK_AUTH_TOKEN;

// Replace with your network credentials
const char* ssid = "";
const char* password = "";

BlynkTimer timer;
WidgetLED led1(V0);

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String outputelectronicsState = "off";
String outputmetalState = "off";
String outputbatteryState = "off";

// Pin Definitions
const int trigPin = 5;
const int echoPin = 18;
const int trigPin_2 = 19;
const int echoPin_2 = 21;
const int trigPin_Gate = 12;
const int echoPin_Gate = 14;
#define STEP_PIN 27
#define DIR_PIN 26
#define SERVO_PIN_1 33
#define STEP_PIN_2 2
#define DIR_PIN_2 4

// Define sound speed in cm/uS
#define SOUND_SPEED 0.034

long duration;
long duration_2;
long duration_Gate;
float distanceCm;
float distanceCm_2;
float distanceCm_Gate;

ESP_FlexyStepper stepper;
ESP_FlexyStepper stepper2;
Servo servo1;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 10000;

// Define the initial and target positions
const int initialPosition = 0;
const int targetPosition = -300;

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
// Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin_2, OUTPUT);
  pinMode(echoPin_2, INPUT);
  pinMode(trigPin_Gate, OUTPUT);
  pinMode(echoPin_Gate, INPUT);

  // Stepper Motor Control Pins
  stepper.connectToPins(STEP_PIN, DIR_PIN);
  stepper2.connectToPins(STEP_PIN_2, DIR_PIN_2);
  //Servo Motor attachment
  servo1.attach(SERVO_PIN_1);
  // Initialize servos to a neutral position
  servo1.write(90);  // Stop servo motor (assuming 90 is neutral for continuous rotation servos)

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop(){
    Blynk.run();
    timer.run();
    digitalWrite(trigPin_Gate, LOW);
    delayMicroseconds(2);
    // Sets the trigPin2 on HIGH state for 10 micro seconds
    digitalWrite(trigPin_Gate, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_Gate, LOW);

    // Measure distance for the second sensor
    duration_Gate = pulseIn(echoPin_Gate, HIGH);
    distanceCm_Gate = duration_Gate * SOUND_SPEED / 2;
    //Serial.print("DistanceGate: ");
    //Serial.println(distanceCm_Gate);
    delay(50);
    GateMovement();

   // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure distance for the first sensor
    duration = pulseIn(echoPin, HIGH);
    distanceCm = duration * SOUND_SPEED / 2;
    //Serial.print("DistanceOne: ");
    //Serial.println(distanceCm);

    // Add a delay to avoid interference
    delay(50); // Adjust this delay as needed

    // Clears the trigPin2
    digitalWrite(trigPin_2, LOW);
    delayMicroseconds(2);
    // Sets the trigPin2 on HIGH state for 10 micro seconds
    digitalWrite(trigPin_2, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin_2, LOW);

    // Measure distance for the second sensor
    duration_2 = pulseIn(echoPin_2, HIGH);
    distanceCm_2 = duration_2 * SOUND_SPEED / 2;
    //Serial.print("DistanceTwo: ");
    //Serial.println(distanceCm_2);

    // Add a short delay before the next loop
    delay(100); // Optional: Adjust this delay as needed



  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // Extract the object recognition result from the POST body
            if (header.indexOf("POST") >= 0) {
              if (header.indexOf("electronics") >= 0) {
                Serial.println("Electronics detected!");
                led1.on();
                Blynk.logEvent("hazardous_trash");
                if (distanceCm <= 3 && distanceCm_2 <= 27) {
                    stepper.setTargetPositionToStop(); // Stop the stepper motor
                    Serial.println("Target distances reached. Stopping motor.");
                    ServoMovement();

                    delay(2000);  // Let the servos run for 2 seconds
                    stepper.moveRelativeInSteps(-1); //Move stepper motor backwards
                      if (distanceCm == 15 && distanceCm_2 == 15){
                          stepper.setTargetPositionToStop(); // Stop the stepper motor
                        }
                    } 
              else {
                    stepper.moveRelativeInSteps(1);  // Move stepper motor continuously
                  }
              } 
              else if (header.indexOf("metal") >= 0) {
                Serial.println("Metal detected!");
                if (distanceCm <= 27 && distanceCm_2 <= 3) {
                stepper.setTargetPositionToStop(); // Stop the stepper motor
                Serial.println("Target distances reached. Stopping motor.");
                ServoMovement();

                delay(2000);  // Let the servos run for 2 seconds
                stepper.moveRelativeInSteps(1); //Move stepper motor backwards
                if (distanceCm == 15 && distanceCm_2 == 15){
                stepper.setTargetPositionToStop(); // Stop the stepper motor
                }
                } 
                else {
                stepper.moveRelativeInSteps(-1);  // Move stepper motor continuously
                }
              } 
              else if (header.indexOf("battery") >= 0) {
                Serial.println("Battery detected!");
                led1.on();
                Blynk.logEvent("hazardous_trash");
                if (distanceCm <= 15 && distanceCm_2 <= 15) {
                ServoMovement();
                }
              }
            }
            
             // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for Electronics
            client.println("<p>Electronics - State " + outputelectronicsState + "</p>");
            if (outputelectronicsState == "off") {
              client.println("<p><a href=\"/electronics/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/electronics/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Display current state, and ON/OFF buttons for Metal
            client.println("<p>Metal - State " + outputmetalState + "</p>");
            if (outputmetalState == "off") {
              client.println("<p><a href=\"/metal/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/metal/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            // Display current state, and ON/OFF buttons for Battery
            client.println("<p>Battery - State " + outputbatteryState + "</p>");
            if (outputbatteryState == "off") {
              client.println("<p><a href=\"/battery/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/battery/off\"><button class=\"button button2\">OFF</button></a></p>");
            }

            client.println("</body></html>");
            client.println();  // End the HTTP response
            break;
          } else {  // Clear currentLine when a newline is received
            currentLine = "";
          }
        } else if (c != '\r') {  // If you got anything else but a carriage return character
          currentLine += c;      // Add it to the end of the currentLine
        }
      }
    }
    Serial.println(header);
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}

void ServoMovement()
{
    servo1.write(180);  // Rotate servo 1
    delay(3000);
    servo1.write(90);
    delay(3000);
    servo1.write(0);
    delay(3000);
    servo1.write(90);
    delay(50);
}

void GateMovement()
{
  stepper2.setSpeedInStepsPerSecond(2000);
  stepper2.setAccelerationInStepsPerSecondPerSecond(4000);

  if (distanceCm_Gate <= 10) 
  {
    // If an object is detected within 10 cm, move to the target position (-300 steps)
    if (stepper2.getCurrentPositionInSteps() != targetPosition) {
      stepper2.moveToPositionInSteps(targetPosition);
    }
  } 
  else 
  {
    // If the object is farther than 10 cm, move back to the initial position
    if (stepper2.getCurrentPositionInSteps() != initialPosition) {
      stepper2.moveToPositionInSteps(initialPosition);
    }
  }

  delay(100); // Short delay before the next sensor check
}
