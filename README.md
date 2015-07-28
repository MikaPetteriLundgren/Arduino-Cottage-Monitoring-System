Arduino-Cottage-Monitoring-System
=================

Arduino based house/cottage monitoring system is a remote companion device for Domoticz system which is located in different
geographical location. The Arduino based house/cottage monitoring system includes following features:

* Monitors Nexa LMST-606 door sensors with 433MHz RF receiver
* Temperature sensor DS18B20
* Ability to turn external wired alarm siren on/off with "switch box"
* Ability to detect power outages with opto-isolator 4N35
* Can be powered with 9V battery if needed (back-up power in case of power outage if no UPS is used/available)
* 2-way communication with Domoticzs over internet using MQTT protocol (ethernet shield is used in the system)

Temperature readings are read from DS18B20 digital temperature sensor via OneWire bus and Dallas Temperature Control Library. 
Door sensor monitoring utilises 433MHz RF receiver connected to a GPIO pin of the Arduino. Siren control requires a switch box
which includes a transistor and input for external power supply. Another external power supply is connected to input of opto-isolator
and the system will detect based on output level of the opto-isolator is mains power present or not (this functionality requires that the
Arduino is powered by UPS or external back-up battery during the power outage).

All the data between Domoticz is transmitted/received via internet using the MQTT protocol.

In practice all the "intelligence" is in Domoticz and therefore for example decision about turning the alarm siren on/off is done by the Domoticz.
The Arduino has a timer which will eventually turn siren off if elapsed.

Arduino-Cottage-Monitoring-System sketch will need following HW and SW libraries to work:

**HW**

* Arduino and ethernet shield
* DS18B20 temperature sensor
* RX433 RF receiver and antenna
* Opto-isolator (for example 4N35)
* Couple of external power supplies and UPS or 9V back-up battery
* NPN-transistor (for example BU406) to switch wired alarm siren on/off

**Libraries**

* Ethernet for ethernet communication
* OneWire for communication with DS18B20 sensor
* SPI for ethernet shield
* DallasTemperature for DS18B20 temperature sensor
* EEPROM for reading/writing to/from EEPROM
* Time and TimeAlarms for alarm functionality
* PubSubClient for MQTT communication

**HW connections**

Temperature readings are read from DS18B20 digital temperature sensor via OneWire bus.
RX433 RF receiver is connected to the Arduino via single GPIO pin.
Input of the opto-isolator 4N35 is connected to Arduino Uno's power jack. Output of the opto-isolator is connected to GPIO pin
of the Arduino. The wired alarm siren is driven by GPIO pin of the Arduino which is connected to base of transistor. External power supply and
the alarm siren are connected to collector of the transistor whereas emitter is connected to GND. So in practice the transistor works as a switch
in this context.

This repository includes also breadboard and schematics pictures about the cottage monitoring system and external switch box which includes above
mentioned transistor in PDF and Fritzing formats.

**Functionality of the sketch**

Already during the setup, MQTT connection is established and callback topic subscribed just in case if there has been a power outage
when Arduino was shutdown last time (UPS or 9V back-up battery ran out of juice). If there was a power outage then current temperature and mains
supply status is updated to the Domoticz.

Purpose of main loop is to be simple in order to spend as much time as possible in NexaReceive function in order to capture all transmissions
from the door sensors. If data has been received from the door sensor, it will be send immediately to the Domoticz. The Nexa door sensors send
several times same message in a row and therefore there is filtering functionality which filters same messages out.

Supply status is checked in the main loop as well. There is a debounce function included which filters short glitches out.

Timings of temperature measurements are taken care by timers. Measured temperature values are sent immediately to the Domoticz.

The alarm siren will be turned on according to the MQTT callback messages from the Domoticz. The alarm siren functionality includes a feature which will
turn siren off after predefined time period has elapsed in order to avoid situation where the alarm siren stays on "forever" if there would be communication
break between the Arduino and the Domoticz.

It's constantly checked in main loop is MQTT connection alive because of MQTT callback function. If the MQTT connection has been disconnected, it will be
established again. If the MQTT connection has been disconnected 10 times altogether for some reason, an ethernet connection will be initialized again.

Events of the door sensors & alarm siren and temperature values are added to the MQTT payload (payload differs depends on a type of a sensor/switch). 
The created MQTT payload is sent to MQTT server via internet.

It's possible to print amount of free RAM memory of Arduino via serial port by uncommenting `#define RAM_DEBUG` line

It's possible to print timing and debug information about Nexa transmitters via serial port by uncommenting `#define NEXA_DEBUG` line

It's possible to print "current" time via serial port by uncommenting `#define DEBUG` line
