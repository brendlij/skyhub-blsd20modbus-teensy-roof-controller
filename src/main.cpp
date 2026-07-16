#include <Arduino.h>
#include <BLSD20Modbus.h>
#include "Config.h"
#include "RoofController.h"
#include "SerialLink.h"
#include "StatusLed.h"

ModbusRTU bus(Serial1);
BLSD20Modbus motor(bus, RoofConfig::MODBUS_SLAVE_ID);
RoofController roof(motor);
StatusLed statusLed;
SerialLink link(roof, statusLed, Serial);

void setup()
{
    Serial.begin(115200);
    bus.begin(RoofConfig::MODBUS_BAUD, SERIAL_8E1, Pins::MODBUS_DE);

    roof.begin();
    link.begin();
    statusLed.begin(Pins::LED_R, Pins::LED_G, Pins::LED_B, Pins::LED_BUTTON, RoofConfig::DEBOUNCE_MS);
}

void loop()
{
    roof.update();
    link.update();
    statusLed.update(roof);
}
