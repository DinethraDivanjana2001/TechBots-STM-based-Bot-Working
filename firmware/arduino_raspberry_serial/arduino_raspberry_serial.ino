void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValueA0 = analogRead(0); // Read the value from analog pin A0
  int sensorValueA1 = analogRead(1); // Read the value from analog pin A1

  // Print the values to the serial monitor
  // Serial.print("A0: ");
  Serial.println(sensorValueA0);
  Serial.print("\t");
  Serial.println(sensorValueA1);

  // delay(100); // Delay for stability

  // Note: You can adjust the delay time for different sampling rates.
}
