/* Arduino based house/cottage monitoring system is a remote companion device for Domoticz system which is located in different
   geographical location. The Arduino based house/cottage monitoring system includes following features:
   -Monitors Nexa LMST-606 door sensors with 433MHz RF receiver
   -Temperature sensor DS18B20
   -Ability to turn external wired alarm siren on/off with "switch box"
   -Ability to detect power outages with opto-isolator 4N35
   -Can be powered with 9V battery if needed (back-up power in case of power outage if no UPS is used/available)
   -2-way communication with Domoticzs over internet using MQTT protocol (ethernet shield is used in the system)

   Temperature readings are read from DS18B20 digital temperature sensor via OneWire bus and Dallas Temperature Control Library. 
   Door sensor monitoring utilises 433MHz RF receiver connected to a GPIO pin of the Arduino. Siren control requires a switch box
   which includes a transistor and input for external power supply. Another external power supply is connected to input of opto-isolator
   and the system will detect based on output level of the opto-isolator is mains power present or not (this functionality requires that the
   Arduino is powered by UPS or external back-up battery during the power outage). The power outage functionality includes a debounce filter in
   order to filter short glitches out.
   
   All the data between Domoticz is transmitted/received via internet using the MQTT protocol.

   In practice all the "intelligence" is in Domoticz and therefore for example decision about turning the alarm siren on/off is done by the Domoticz.
   The Arduino has a timer which will eventually turn siren off if elapsed.
   
   The sketch also includes a DEBUG functions to check the amount of free RAM, print Nexa receiver timing data and relevant time information.
   */

#include <Ethernet.h>
#include <OneWire.h>
#include <SPI.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <PubSubClient.h>

unsigned long startupDelay = 120000; /* Start-up delay in milliseconds which is needed because start-up of 3G modem takes pretty long time.
3G modem has to be connected to internet before Arduino is started-up. Default value is 2min (120000ms)*/

// Temperature measurement variables are initialised
const unsigned int tempMeasInterval = 1800; // Set temperature measurement interval in seconds. Default value 1800s (30min)
float Temperature = 0; // Measured temperature is stored to this variable

// OneWire and Dallas temperature sensors library are initialised
#define ONE_WIRE_BUS 2 // OneWire data wire is plugged into pin 2 on the Arduino. Parasite powering scheme is used.
OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature.

// Network settings
byte mac[] = { 0x90, 0xA5, 0xDA, 0x0D, 0xCC, 0x55 }; // this must be unique
EthernetClient ethClient; // Initialize Arduino Ethernet Client
int mqttConnectionFails = 0; // If MQTT connection is disconnected for some reason, this variable is increment by 1

//MQTT configuration
#define  DEVICE_ID  "Uno"
#define MQTT_SERVER "192.168.1.2" // IP address of MQTT server. CHANGE THIS TO CORRECT ONE!

// MQTT callback function header
void mqttCallback(char* topic, byte* payload, unsigned int length);

//MQTT initialization
PubSubClient mqttClient(MQTT_SERVER, 1883, mqttCallback, ethClient);
char clientID[50];
char topic[50];
char msg[80];
char subscribeTopic[ ] = "/events/domoticz/NAME_OF_TOPIC/#"; // Subscribed topic for the callback function. Arduino will listen this topic for incoming MQTT messages from the Domoticz. CHANGE THIS TO CORRECT ONE!

//MQTT variables
String topicID = "MQTT_TOPIC_ID"; // MQTT topicID which is used to create a MQTT topic used in data transmission to the Domoticz. CHANGE THIS TO CORRECT ONE!
int temperatureSensordtype = 80; // dtype (device type for temperature sensor) is used to help creating MQTT payload
int switchdtype = 32; // dtype (device type for door switches and siren) is used to help creating MQTT payload
const int temperatureSensorIDX = 18; // IDX number of temperature sensor
const int sideDoorSwitchIDX = 15; // IDX number of side door switch
const int frontDoorSwitchIDX = 16; // IDX number of front door switch
const int supplyIDX = 19; // IDX number of virtual electricity on/off switch
const int sirenIDX = 22; // IDX number of virtual electricity on/off switch
int switchStatus = 0; // Status of switch (0 = Closed/Off & 1 = Open/on)

// Variables for supply status function
const int eepromAddress = 5; // EEPROM address to store status of supplyPin
const int supplyPin = 8; // Output of opto-isolator is connected to pin 8
boolean supplyStatus = LOW; // Status of supply voltage (LOW = present and HIGH = not present)
boolean previousSupplyStatus = LOW; // Previous status of supply voltage (LOW = present and HIGH = not present)
const int debounceDelay = 5; // Debounce delay (to wait until input is stable). Debounce time is calculated with following equation: debounceDelay x 100ms

// Variables for Nexa receiver functionality
const int rxPin = 7; // Input pin of 433 MHz receiver
unsigned long prevSender = 0; // Previous Nexa device which sent data is stored to this variable
const int switchInterval = 2000; // Interval in milliseconds which determines how long have to be waited before same data is accepted from the same Nexa device
boolean prevOn = false; // Status of previous Nexa switch which sent data is stored to this variable
unsigned long time = 0; // Used for storing system time

// Variables for external siren functionality
const int sirenPin = 5; // Output pin for external siren control
const int sirenWatchDogTimer = 300; // Max time siren will be on if not turned off manually or by Domoticz. Default value 5min (300s)

//#define DEBUG // Comment this line out if there is no need to print debug information via serial port
//#define RAM_DEBUG // Comment this line out if there is no need to print RAM debug information via serial port
//#define NEXA_DEBUG // Comment this line out if there is no need to print NEXA receiver debug information via serial port

void setup()
{
  Serial.begin(9600); // Start serial port
  //while (!Serial) ; // Needed for Leonardo only

  Serial.print(F("Arduino will wait "));
  Serial.print(startupDelay / 1000);
  Serial.println(F("s before boot continues"));
  delay(startupDelay);

  sensors.begin(); // Start up the library

  // SD Card SPI CS signal configured to output and set to high state. SC Card and WiFi controller shares same SPI bus
  pinMode(4,OUTPUT);
  digitalWrite(4,HIGH);

  pinMode(rxPin, INPUT); // Input of 433 MHz receiver

  //Variable for external siren functionality is initialised
  pinMode(sirenPin, OUTPUT);
  digitalWrite(sirenPin, LOW);

  // Start Ethernet on Arduino
  startEthernet();

  //Create MQTT client String
  String clientIDStr = "Arduino-";
  clientIDStr.concat(DEVICE_ID);
  clientIDStr.toCharArray(clientID, clientIDStr.length()+1);

  //MQTT connection is established and topic subscribed for the callback function
  mqttClient.connect(clientID) ? Serial.println("MQTT client connected") : Serial.println("MQTT client connection failed..."); //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator
  mqttClient.subscribe(subscribeTopic) ? Serial.println("MQTT topic subscribed succesfully") : Serial.println("MQTT topic subscription failed..."); //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator

  // Timers are initialised
  Alarm.timerRepeat(tempMeasInterval, tempFunction);  // Temperature measurement and delivery function is run every 30min

  // Initialise supply status pin and read status from EEPROM
  pinMode(supplyPin, INPUT);
  previousSupplyStatus = EEPROM.read(eepromAddress); // If previousSupplyStatus is HIGH then there has been power outage when Arduino shutdown last time
  Serial.print(F("State of previousSupplyStatus is: "));
  Serial.println(previousSupplyStatus);

  // Check was there a power outage when Arduino was shutdown last time. If true then update temperature and supply status to Domoticz
  supplyStatus = debounce(supplyPin);

  if(supplyStatus == LOW && previousSupplyStatus == HIGH) // LOW = present and HIGH = not present
    {
      Serial.println(F("Arduino was shutdown during power outage"));
      EEPROM.write(eepromAddress, supplyStatus); // Write status of supplyPin to EEPROM
      previousSupplyStatus = supplyStatus;

      tempFunction(); // Measure temperature and send it to Domoticz
      supplyFunction(); // Send status of power supply to Domoticz
    }

  // Variables of Nexa receiver functionality are initialised
  boolean on = false, group = false;

  Serial.println(F("Setup completed succesfully!\n"));
}


void loop()
{
    //If connection to MQTT broker is disconnected. Connect and subscribe again
    if (!mqttClient.connected())
    {
    	(mqttClient.connect(clientID)) ? Serial.println(F("MQTT client connected")) : Serial.println(F("MQTT client connection failed...")); //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator

    	//MQTT topic subscribed for the callback function
    	(mqttClient.subscribe(subscribeTopic)) ? Serial.println(F("MQTT topic subscribed succesfully")) : Serial.println(F("MQTT topic subscription failed...")); //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator

    	mqttConnectionFails +=1; // // If MQTT connection is disconnected for some reason, this variable is increment by 1

    	// Ethernet connection to be disconnected and initialized again if MQTT connection has been disconnected 10 times
    	if (mqttConnectionFails >= 10)
    	{
    		Serial.println(F("Ethernet connection to be initialized again!"));
    		mqttConnectionFails = 0;
    		startEthernet();
    	}
    }

    mqttClient.loop(); //This should be called regularly to allow the MQTT client to process incoming messages and maintain its connection to the server.

	// Nexa receiver code
    unsigned long sender; // Variable Sender is used to store sender of the message
    boolean on, group; // Variable On is used to store status of device and group is a group where device belongs to

    // NexaReceive function returns received message if message has been received. Otherwise 0 is returned.
    if (NexaReceive(sender, on, group) != 0) {

      // Nexa device sends several times same message in a row. Same messages are filtered out with the following if statement
       if(prevSender != sender || prevOn != on || now() > (time + switchInterval))
	{
          printResult(sender, group, on); // Print the received message via Serial Monitor
          time = now();
          switchFunction(sender, on); // Send state of door switch to Domoticz via MQTT protocol
	}
        prevSender = sender; // Sender of the last message is stored to prevSender variable
        prevOn = on;  // Status of the last message is stored to prevSender variable
    }

	// If state of power supply has changed, update temperature and supply status to Domoticz

    if(digitalRead(supplyPin) != previousSupplyStatus) // Check "quickly" if status of power supply has changed
    {
    	supplyStatus = debounce(supplyPin);

    	if(supplyStatus != previousSupplyStatus) // Check if state change is real or just a glitch
    	{
			Serial.print(F("State of power supply has changed. Current state is: "));
			// Switch status: 0 = HIGH (not present) and 1 = LOW (present)
			supplyStatus ? Serial.println(F("Power supply not present")) : Serial.println(F("Power supply present")); //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator

			EEPROM.write(eepromAddress, supplyStatus); // Write status of supplyPin to EEPROM
			previousSupplyStatus = supplyStatus;

			tempFunction(); // Measure temperature and send it to Domoticz because it's possible that temperature change function hasn't run yet
			supplyFunction(); // Send status of power supply to Domoticz
    	}
    }

    Alarm.delay(0); //Timers are only checks and their functions called when you use this delay function. You can pass 0 for minimal delay.


   #if defined RAM_DEBUG
     Serial.print(F("Amount of free RAM memory: "));
     Serial.print(memoryFree()); // Prints the amount of free RAM memory
     Serial.println(F(" / 2048 bytes")); //ATmega328 has 2kB of RAM memory
   #endif
}


float tempReading() // Function tempReading reads temperature from DS18B20 sensor and returns it
{
  sensors.requestTemperatures(); // Send the command to get temperatures
  return (float) (sensors.getTempCByIndex(0)); // Return measured temperature
}

boolean debounce(int pin) // Debounce function returns state of the pin after it has been stable longer than debounceDelay
{
  boolean state = LOW;
  boolean previousState = LOW;

  previousState = digitalRead(pin); // Store state of pin
  for(int counter = 0; counter < debounceDelay; counter++)
  {
    delay(100); // wait for 100 ms
    state = digitalRead(pin); // Read state of pin
    if(state != previousState)
      {
        counter = 0; // Reset the counter if the state changes
        previousState = state; // Save the current state
      }
  }
  return state; // Return state of the pin when it has been stable long enough
}

void startEthernet()
{
  ethClient.stop();

  Serial.println(F("Connecting Arduino to network..."));

  delay(1000);

  // Connect to network and obtain an IP address using DHCP
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println(F("DHCP Failed, reset Arduino to try again\n"));
  }
  else
  {
    Serial.println(F("Arduino connected to network using DHCP"));
    Serial.print(F("IP address: "));
    Serial.println(Ethernet.localIP()); // print your local IP address:
  }
  delay(1000);
}

// NexaReceive function returns received message if message has been received. Otherwise 0 is returned.
unsigned long NexaReceive(unsigned long &sender, boolean &on, boolean &group) {
  int i = 0;
  unsigned long t = 0;

  byte prevBit = 0;
  byte bit = 0;
  unsigned int recipient = 0;
  unsigned long receivedData = 0;
  int rotations = 0;

  #ifdef NEXA_DEBUG
    int t1 = 0; // Latch 1 time only needed for NEXA_DEBUGging purposes
    int t2 = 0; //latch 2 time only needed for NEXA_DEBUGging purposes.
  #endif

  // Latch 1
  // Latch timing has been loosened to accommodate varieties in the Nexa transmitters.
  while(t < 8000 || t > 13000) {
    t = pulseIn(rxPin, LOW, 13500);
    if(rotations++ > 10000)
      return 0;
  }
  
  #ifdef NEXA_DEBUG
    t1 = t; // Save latch timing for NEXA_DEBUGging purposes
  #endif

  rotations = 0;
  // Latch 2
  // Latch timing has been loosened to accommodate varieties in the Nexa transmitters.
  while(t < 2200 || t > 2900) {
    t = pulseIn(rxPin, LOW, 3400);
    if(rotations++ > 10000)
      return 0;
  }
  
  #ifdef NEXA_DEBUG
    t2 = t; // Save latch timing for NEXA_DEBUGging purposes
  #endif

  // data collection from receiver message
  while (i < 64)
  {
    t = pulseIn(rxPin, LOW, 1560);
    if (t > 200 && t < 400)
      bit = 0;
    else if (t > 1100 && t < 1560)
      bit = 1;
    else
     return 0;

    if (i % 2 == 1) {
      if ((prevBit ^ bit) == 0) // must be either 01 or 10, cannot be 00 or 11
        return 0;
      receivedData <<= 1;
      receivedData |= prevBit;
      if (i < 53) { // first 26 data bits (Sender)
        sender <<= 1;
        sender |= prevBit;
      }
      else if (i == 53)
      { // 26th data bit (Group)
        group = prevBit;
      }
      else if (i == 55)
      { // 27th data bit (On/Off)
        on = prevBit;
      }
      else
      { // last 4 data bits
        recipient <<= 1;
        recipient |= prevBit;
      }
    }
    prevBit = bit;
    ++i;
  }

  #ifdef NEXA_DEBUG
    Serial.println(receivedData,BIN);
    Serial.print("0x"); Serial.println(receivedData,HEX);
    Serial.print("Timings: ");
    Serial.print(t1); // Timing for latch 1
    Serial.print(",");
    Serial.println(t2); // Timing for latch 2
  #endif

  sender = receivedData >> 6;
  on = (receivedData >> 4) & 0x01;
  group = (receivedData >> 5) & 0x01;
  return receivedData;
}

// Function printResult prints out received message via Serial monitor
void printResult(unsigned long sender, boolean group, boolean on)
{
  Serial.print("## Sender: ");
  Serial.print(sender);

  group ? Serial.print(" Group Cmd ") : Serial.print(""); //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator
  on ? Serial.println(" On") : Serial.println(" Off"); //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator
}

String createMQTTPayload(int idx, int deviceType) //Create MQTT message payload. Returns created message as a String.
{
	String dataMsg = "{\"idx\":";
	dataMsg.concat(idx);

	switch (deviceType)
	  {
		case 80: // Temperature sensor
			char buffer[10]; // Needed with dtostrf function
	    	dataMsg.concat(F(",\"svalue\":\""));
	    	dataMsg.concat(dtostrf(Temperature, 5, 1, buffer)); //Converts float temperature to String with 1 digit precision
	    	dataMsg.concat("\"}");
	    	break;

		case 32: // Shutter contact. Applied to switches and sirens in this case as well
	    	dataMsg.concat(F(",\"nvalue\":\""));
	    	dataMsg.concat(switchStatus);
	    	dataMsg.concat("\"}");
	    	break;

		default: // If dtype is unknown, then default case to be used. Default case is a temperature sensor.
			Serial.println(F("Unknown dtype received. Default procedure to be done."));
	    	dataMsg.concat(F(",\"svalue\":\""));
	    	dataMsg.concat(Temperature);
	    	dataMsg.concat("\"}");
	    	break;
	  }

	return dataMsg;
}

void sendMQTTPayload(String payload) // Sends MQTT payload to the MQTT server running on a Raspberry Pi.
// MQTT server deliveres data to the Domoticz server running on a same Raspberry Pi
{

  // Create MQTT topic and convert it to char array
	String topicStr = "/actions/domoticz/";
	topicStr.concat(topicID);
	topicStr.toCharArray(topic, topicStr.length()+1);

	// Convert payload to char array
	payload.toCharArray(msg, payload.length()+1);

    //If connection to MQTT broker is disconnected, connect again
    if (!mqttClient.connected())
    {
    	(mqttClient.connect(clientID)) ? Serial.println(F("MQTT client connected")) : Serial.println(F("MQTT client connection failed...")); //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator
    }

	//Publish payload to MQTT broker
	if (mqttClient.publish(topic, msg))
	{
		Serial.print("Following data published to MQTT broker: ");
		Serial.print(topic);
		Serial.print(" ");
		Serial.println(payload);
		Serial.println();
	}
	else
		Serial.println(F("Publishing to MQTT broker failed..."));
}

// mqttCallback function handles message arrived on subscribed MQTT topic(s)
void mqttCallback(char* topic, byte* payload, unsigned int length) {

  Serial.println("MQTT callback received");

  //Create a String from the received payload (char array)
  String callBackPayload = "";
  int i;

  for (i = 0; i < length; i++)
	{
		//Serial.print((char)payload[i]);
		callBackPayload += (char)payload[i];
	}

  // Act according to received payload
  if (callBackPayload == "On") // Siren to be set On
  {
	  Serial.println(F("Siren turned On"));
	  digitalWrite(sirenPin, HIGH);
	  Alarm.timerOnce(sirenWatchDogTimer, sirenOffFunction); // Timer is set which will turn off siren after sirenWatchDogTimer period has elapsed
  }
  else if (callBackPayload == "Off" && digitalRead(sirenPin) == LOW) // No actions needed if siren is already off
  {
	  Serial.println(F("Siren is already off. No actions needed!"));
  }
  else if (callBackPayload == "Off") // Siren to be set Off
    {
  	  sirenOffFunction();
    }
  else // Not valid state. No actions to be taken
  {
	  Serial.print(F("Following not valid data received. No actions to be taken:"));
	  Serial.println(callBackPayload);
  }
}

void tempFunction() // Function tempFunction reads temperature from DS18B20 sensor and sends it to Domoticz via MQTT protocol
{
  Temperature = tempReading(); // Current temperature is measured

	#if defined DEBUG
	  Serial.print(F("Current time is: "));
	  Serial.println(now());
	#endif

  // Read temperature is printed
  Serial.print(F("Temperature: "));
  Serial.print(Temperature);
  Serial.println(F("DegC"));

  //Send data to MQTT broker running on a Raspberry Pi
  sendMQTTPayload(createMQTTPayload(temperatureSensorIDX, temperatureSensordtype));
}

void switchFunction(unsigned long sender, boolean on) //Send state of switch/siren to Domoticz via MQTT protocol
{

  // Switch status: 0 = Closed/Off and 1 = Open/On
  on ? switchStatus = 1 : switchStatus = 0; //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator

  if (sender == 13624338) // 13624338 = Front door switch
  {
	  //Send data to MQTT broker running on a Raspberry Pi
	  sendMQTTPayload(createMQTTPayload(frontDoorSwitchIDX, switchdtype));
  }
  else if (sender == 13625334) // 13625334 = Side door switch
  {
	  sendMQTTPayload(createMQTTPayload(sideDoorSwitchIDX, switchdtype));
  }
  else if (sender == 12345678) // 12345678 = Wired alarm siren
  {
	  sendMQTTPayload(createMQTTPayload(sirenIDX, switchdtype));
  }
  else
  {
	  Serial.println("Unknown switch data received...");
  }
}

void supplyFunction() //Send status of electric supply to Domoticz via MQTT protocol
{
    // Switch status: 0 = HIGH (not present/off) and 1 = LOW (present/on)
    supplyStatus ? switchStatus = 0 : switchStatus = 1; //condition ? valueIfTrue : valueIfFalse - This is a Ternary operator

    //Send data to MQTT broker running on a Raspberry Pi
    sendMQTTPayload(createMQTTPayload(supplyIDX, switchdtype));
}

void sirenOffFunction() //Switches external siren off and updates state of the siren back to Domoticz
{
	digitalWrite(sirenPin, LOW);
	Serial.println(F("Siren turned off"));

	// Switch status: 0 = Closed/Off and 1 = Open/On
	unsigned long sirenSenderID = 12345678; // Variable sirenSenderID is used to store sender of the message
	switchFunction(sirenSenderID, false); // Send off state of siren to Domoticz via MQTT protocol
}

// variables created by the build process when compiling the sketch. Used in memoryFree function
extern int __bss_end;
extern void *__brkval;

int memoryFree() //Function to return the amount of free RAM
{
  int freeValue;
  if((int)__brkval == 0)
  {
    freeValue = ((int)&freeValue) - ((int)&__bss_end);
  }
  else
  {
    freeValue = ((int)&freeValue) - ((int)__brkval);
  }
  return freeValue;
}

