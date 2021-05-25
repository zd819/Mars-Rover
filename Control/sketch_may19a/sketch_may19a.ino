const int relay_pin = 8; // Relay pin

void setup() {
   Serial.begin(115200);
   pinMode(relay_pin,OUTPUT);
}

void loop() {
   // Activate relay
   digitalWrite(relay_pin, HIGH);
   // Wait for 1 second
   delay(1000);
   // Deactivate relay
   digitalWrite(relay_pin, LOW);
   // Wait for 1 second
   delay(1000);
}
