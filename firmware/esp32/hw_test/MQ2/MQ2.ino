#define MQ2_PIN 34

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);                 // 0-4095
  analogSetPinAttenuation(MQ2_PIN, ADC_11db);

  Serial.println("MQ-2 Analog Test");
}

void loop() {

  int adc = analogRead(MQ2_PIN);

  float voltage = adc * 3.3 / 4095.0;

  Serial.println("------------------------");
  Serial.print("ADC     : ");
  Serial.println(adc);

  Serial.print("Voltage : ");
  Serial.print(voltage, 3);
  Serial.println(" V");

  delay(1000);
}