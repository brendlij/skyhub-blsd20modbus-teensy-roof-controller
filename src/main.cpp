#include <Arduino.h>
#include <BLSD20Modbus.h>
#include "Config.h"
#include "RoofController.h"
#include "SerialLink.h"

ModbusRTU bus(Serial1);
BLSD20Modbus motor(bus, RoofConfig::MODBUS_SLAVE_ID);
RoofController roof(motor);
SerialLink link(roof, Serial);

void setup()
{
    Serial.begin(115200);
    bus.begin(RoofConfig::MODBUS_BAUD, SERIAL_8E1, Pins::MODBUS_DE);

    roof.begin();
    link.begin();
}

void loop()
{
    roof.update();
    link.update();
}
