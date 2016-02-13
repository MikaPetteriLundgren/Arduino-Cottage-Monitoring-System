/*
  Arduino-Cottage-Monitoring-System.h header file includes settings for the Arduino-Cottage-Monitoring-System sketch.
  Header file needs to be stored within a same folder with the sketch.
*/
#ifndef Arduino-Cottage-Monitoring-System_h
	#define Arduino-Cottage-Monitoring-System_h

	#define MQTT_SERVER "192.168.1.93" // IP address of the MQTT server
	#define DEVICE_ID "Uno"
	#define MQTT_TOPIC "domoticz/in" // Default incoming topic in Domoticz is domoticz/in
	#define MQTT_SUBSCRIBE_TOPIC "domoticz/out" // Default outgoing topic in Domoticz is domoticz/out

#endif
