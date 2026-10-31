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

  sensors.requestTemperatures();

  for (int i = 0; i < count; i++)
  {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print("  addr: ");

    if (sensors.getAddress(addr, i))
    {
      for (uint8_t j = 0; j < 8; j++)
      {
        if (addr[j] < 16) Serial.print("0");
        Serial.print(addr[j], HEX);
        if (j < 7) Serial.print(":");
      }
    }
    else
    {
      Serial.print("UNKNOWN");
    }

    Serial.print("  temp: ");
    Serial.print(sensors.getTempCByIndex(i), 2);
    Serial.println(" C");
  }
  Serial.println("-- warm a sensor to identify which index it is --");
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