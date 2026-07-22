#include <Arduino.h>
#include <BLSD20Modbus.h>
#include "BLSD20Driver.h"
#include "CommandLayer.h"
#include "Config.h"
#include "MotionController.h"
#include "SerialLink.h"
#include "StateMachine.h"
#include "StatusLed.h"

// Firmware layer wiring (Spec Abschnitt 4):
//   USB / Buttons -> Command Layer -> State Machine -> Motion Controller
//   -> BLSD20 Driver -> BLSD20Modbus -> Motor

ModbusRTU bus(Serial1);
BLSD20Modbus motor(bus, RoofConfig::MODBUS_SLAVE_ID);

BLSD20Driver driver(motor);
MotionController motionController(driver);
StateMachine stateMachine(driver, motionController);
CommandLayer commandLayer(stateMachine);
StatusLed statusLed;
SerialLink serialLink(stateMachine, commandLayer, statusLed, Serial);

void setup()
{
    Serial.begin(115200);
    bus.begin(RoofConfig::MODBUS_BAUD, SERIAL_8E1, Pins::MODBUS_DE);

    // Boot (Spec Abschnitt 12): configure the drive, read sensors, derive
    // Position/Zone, only then start accepting Fahrbefehle.
    driver.begin();
    stateMachine.begin();
    commandLayer.begin();
    serialLink.begin();
    statusLed.begin(Pins::LED_R, Pins::LED_G, Pins::LED_B, Pins::LED_BUTTON, RoofConfig::DEBOUNCE_MS);
}

void loop()
{
    driver.pollTelemetry();
    driver.pollTemperature();
    stateMachine.tick();
    commandLayer.tick();
    serialLink.update();
    statusLed.update(stateMachine);
}
