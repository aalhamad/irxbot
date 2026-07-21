#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DeviceAddress addr;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting...");

  sensors.begin();

  int count = sensors.getDeviceCount();

  Serial.print("Device Count = ");
  Serial.println(count);

  for (int i = 0; i < count; i++)
  {
    if (sensors.getAddress(addr, i))
    {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" Address: ");

      for (uint8_t j = 0; j < 8; j++)
      {
        if (addr[j] < 16) Serial.print("0");
        Serial.print(addr[j], HEX);
      }

      Serial.println();
    }
    else
    {
      Serial.print("Failed to read address for sensor ");
      Serial.println(i);
    }
  }
}

void loop()
{
  sensors.requestTemperatures();

  int count = sensors.getDeviceCount();

  Serial.println("--------------------");

  for (int i = 0; i < count; i++)
  {
    float temp = sensors.getTempCByIndex(i);

    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(temp);
    Serial.println(" C");
  }

  delay(2000);
}