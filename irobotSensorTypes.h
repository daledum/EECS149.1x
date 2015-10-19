/** \file irobotSensorTypes.h
 *
 * Sensor packet IDs and structures for the iRobot Create Open Interface
 *
 * \author Jeff C. Jensen
 * \date 2013-12-09
 * \copyright Copyright (C) 2013, Jeff C. Jensen, Edward A. Lee, and Sanjit A. Seshia.
 * 			  This software accompanies An Introductory Lab in Embedded and Cyber-Physical Systems
 * 			  and is licensed under a Creative Commons Attribution-NonCommercial-NoDerivs 3.0
 * 			  Unported License. See http://leeseshia.org/lab.
 */

#ifndef _IROBOT_SENSORTYPES_H
#define _IROBOT_SENSORTYPES_H

#include "irobotTypes.h"

/// iRobot sensor packet IDs
typedef enum{
	SENSOR_GROUP0							= 0,	///< Sensor Group 0
	SENSOR_GROUP1							= 1,	///< Sensor Group 1
	SENSOR_GROUP2							= 2,	///< Sensor Group 2
	SENSOR_GROUP3							= 3,	///< Sensor Group 3
	SENSOR_GROUP4							= 4,	///< Sensor Group 4
	SENSOR_GROUP5							= 5,	///< Sensor Group 5
	SENSOR_GROUP6							= 6,	///< Sensor Group 6
	SENSOR_BUMPS_WHEELDROPS					= 7,	///< Bumps & wheel drops
	SENSOR_WALL								= 8,	///< Wall
	SENSOR_CLIFF_LEFT						= 9,	///< Left cliff
	SENSOR_CLIFF_FRONT_LEFT					= 10,	///< Front left cliff
	SENSOR_CLIFF_FRONT_RIGHT				= 11,	///< Front right cliff
	SENSOR_CLIFF_RIGHT						= 12,	///< Right cliff
	SENSOR_VIRTUAL_WALL						= 13,	///< Virtual wall
	SENSOR_LOW_SIDE_DRIVER_WHEEL_OVERDRIVE	= 14,	///< Low-side driver and wheel overdrive
	SENSOR_UNUSED0							= 15,	///< Unused
	SENSOR_UNUSED1							= 16,	///< Unused
	SENSOR_INFRARED							= 17,	///< Infrared receiver
	SENSOR_BUTTONS							= 18,	///< Buttons
	SENSOR_DISTANCE							= 19,	///< Distance travelled
	SENSOR_ANGLE							= 20,	///< Angle turned
	SENSOR_CHARGING_STATE					= 21,	///< Charging state
	SENSOR_VOLTAGE							= 22,	///< Battery voltage
	SENSOR_CURRENT							= 23,	///< Battery current
	SENSOR_BATTERY_TEMPERATURE				= 24,	///< Battery temperature
	SENSOR_BATTERY_CHARGE					= 25,	///< Battery charge
	SENSOR_BATTERY_CAPACITY					= 26,	///< Battery capacity
	SENSOR_WALL_SIGNAL						= 27,	///< Wall analog signal
	SENSOR_CLIFF_LEFT_SIGNAL				= 28,	///< Left cliff analog signal
	SENSOR_CLIFF_FRONT_LEFT_SIGNAL			= 29,	///< Front left cliff analog signal
	SENSOR_CLIFF_FRONT_RIGHT_SIGNAL			= 30,	///< Front right cliff analog signal
	SENSOR_CLIFF_RIGHT_SIGNAL				= 31,	///< Right cliff analog signal
	SENSOR_CARGO_BAY_DIGITAL_INPUTS			= 32,	///< Cargo bay digital inputs
	SENSOR_CARGO_BAY_ANALOG_SIGNAL			= 33,	///< Cargo bay analog input
	SENSOR_CHARGING_SOURCES_AVAILABLE		= 34,	///< Charging sources available
	SENSOR_OI_MODE							= 35,	///< Open Interface (OI) mode
	SENSOR_SONG_NUMBER						= 36,	///< Song number
	SENSOR_SONG_PLAYING						= 37,	///< Song playing?
	SENSOR_NUMBER_OF_STREAM_PACKETS			= 38,	///< Number of packets in sensor stream
	SENSOR_REQUESTED_VELOCITY				= 39,	///< Requested drive velocity
	SENSOR_REQUESTED_RADIUS					= 40,	///< Requested drive radius
	SENSOR_REQUESTED_RIGHT_VELOCITY			= 41,	///< Requested right wheel velocity
	SENSOR_REQUESTED_LEFT_VELOCITY			= 42	///< Requested left wheel velocity
} irobotSensorCode;

// iRobot sensor packet sizes
#define SENSOR_GROUP0_SIZE							26	///< Size of the SENSOR_GROUP0 packet
#define SENSOR_GROUP1_SIZE							10	///< Size of the SENSOR_GROUP1 packet
#define SENSOR_GROUP2_SIZE							6	///< Size of the SENSOR_GROUP2 packet
#define SENSOR_GROUP3_SIZE							10	///< Size of the SENSOR_GROUP3 packet
#define SENSOR_GROUP4_SIZE							14	///< Size of the SENSOR_GROUP4 packet
#define SENSOR_GROUP5_SIZE							12	///< Size of the SENSOR_GROUP5 packet
#define SENSOR_GROUP6_SIZE							52	///< Size of the SENSOR_GROUP6 packet
#define SENSOR_BUMPS_WHEELDROPS_SIZE				1	///< Size of the BUMPS_WHEELDROPS packet
#define SENSOR_WALL_SIZE							1	///< Size of the WALL_SIZE packet
#define SENSOR_CLIFF_LEFT_SIZE						1	///< Size of the CLIFF_LEFT packet
#define SENSOR_CLIFF_FRONT_LEFT_SIZE				1	///< Size of the CLIFF_FRONT_LEFT packet
#define SENSOR_CLIFF_FRONT_RIGHT_SIZE				1 	///< Size of the CLIFF_FRONT_RIGHT packet
#define SENSOR_CLIFF_RIGHT_SIZE						1 	///< Size of the CLIFF_RIGHT packet
#define SENSOR_VIRTUAL_WALL_SIZE					1 	///< Size of the VIRUAL_WALL packet
#define SENSOR_LOW_SIDE_DRIVER_WHEEL_OVERDRIVE_SIZE	1 	///< Size of the LOW_SIDE_DRIVER_WHEEL_OVERDRIVE packet
#define SENSOR_UNUSED0_SIZE							1 	///< Size of the UNUSED0 packet
#define SENSOR_UNUSED1_SIZE							1 	///< Size of the UNUSED1 packet
#define SENSOR_INFRARED_SIZE						1 	///< Size of the INFRARED packet
#define SENSOR_BUTTONS_SIZE							1 	///< Size of the BUTTONS packet
#define SENSOR_DISTANCE_SIZE						2 	///< Size of the DISTANCE packet
#define SENSOR_ANGLE_SIZE							2 	///< Size of the ANGLE packet
#define SENSOR_CHARGING_STATE_SIZE					1 	///< Size of the CHARGING_STATE packet
#define SENSOR_VOLTAGE_SIZE							2 	///< Size of the VOLTAGE packet
#define SENSOR_CURRENT_SIZE							2 	///< Size of the CURRENT packet
#define SENSOR_BATTERY_TEMPERATURE_SIZE				1 	///< Size of the BATTERY_TEMPERATURE packet
#define SENSOR_BATTERY_CHARGE_SIZE					2 	///< Size of the BATTERY_CHARGE packet
#define SENSOR_BATTERY_CAPACITY_SIZE				2 	///< Size of the BATTERY_CAPACITY packet
#define SENSOR_WALL_SIGNAL_SIZE						2 	///< Size of the WALL_SIGNAL packet
#define SENSOR_CLIFF_LEFT_SIGNAL_SIZE				2 	///< Size of the CLIFF_LEFT_SIGNAL packet
#define SENSOR_CLIFF_FRONT_LEFT_SIGNAL_SIZE			2 	///< Size of the CLIFF_FRONT_LEFT_SIGNAL packet
#define SENSOR_CLIFF_FRONT_RIGHT_SIGNAL_SIZE		2 	///< Size of the CLIFF_FRONT_RIGHT_SIGNAL packet
#define SENSOR_CLIFF_RIGHT_SIGNAL_SIZE				2 	///< Size of the CLIFF_RIGHT_SIGNAL packet
#define SENSOR_CARGO_BAY_DIGITAL_INPUTS_SIZE		1 	///< Size of the CARGO_BAY_DIGITAL_INPUTS packet
#define SENSOR_CARGO_BAY_ANALOG_SIGNAL_SIZE			2 	///< Size of the CARGO_BAY_ANALOG_SIGNAL packet
#define SENSOR_CHARGING_SOURCES_AVAILABLE_SIZE		1 	///< Size of the CHARGING_SOURCES_AVAILABLE packet
#define SENSOR_OI_MODE_SIZE							1 	///< Size of the OI_MODE packet
#define SENSOR_SONG_NUMBER_SIZE						1 	///< Size of the SONG_NUMBER packet
#define SENSOR_SONG_PLAYING_SIZE					1 	///< Size of the SONG_PLAYING packet
#define SENSOR_NUMBER_OF_STREAM_PACKETS_SIZE		1 	///< Size of the NUMBER_OF_STREAM_PACKETS packet
#define SENSOR_REQUESTED_VELOCITY_SIZE				2 	///< Size of the REQUESTED_VELOCITY packet
#define SENSOR_REQUESTED_RADIUS_SIZE				2 	///< Size of the REQUESTED_RADIUS packet
#define SENSOR_REQUESTED_RIGHT_VELOCITY_SIZE		2 	///< Size of the REQUESTED_RIGHT_VELOCITY packet
#define SENSOR_REQUESTED_LEFT_VELOCITY_SIZE			2 	///< Size of the REQUESTED_LEFT_VELOCITY packet

/// Minimum sensor update period, in ms
#define SENSOR_MIN_UPDATE_PERIOD	15

/// Next highest power of 2 that is greater than any iRobot packet size;
///		used to create stream buffers large enough to hold any packet.
#define SENSOR_SIZE_UPPER_BOUND		64

/// iRobot bump and wheel-drop sensors.
typedef struct{
	bool	wheeldropCaster;	///< State of the caster wheeldrop sensor; when true, indicates the wheel is not in contact with the ground.
	bool	wheeldropLeft;		///< State of the left wheeldrop sensor; when true, indicates the wheel is not in contact with the ground.
	bool	wheeldropRight;		///< State of the right wheeldrop sensor; when true, indicates the wheel is not in contact with the ground.
	bool	bumpLeft;			///< State of the left bump sensor; if true, the left side of the robot is in contact with an object.
	bool	bumpRight;			///< State of the right bump sensor; if true, the right side of the robot is in contact with an object.
} irobotBumps_WheelDrops_t;

/// iRobot button sensors.
typedef struct{
	bool	advance;			///< advance button
	bool	play;				///< play button
} irobotButtons_t;

/// iRobot cargo bay digital inputs.
typedef struct{
	bool	digitalInput0;		///< digital input pin 0
	bool	digitalInput1;		///< digital input pin 1
	bool	digitalInput2;		///< digital input pin 2
	bool	digitalInput3;		///< digital input pin 3
	bool	deviceDetect;		///< device detect pin
} irobotCargoBayDigitalInputs_t;

/// iRobot charging source sensors.
typedef struct{
	bool	internalCharger;	///< internal charger
	bool	homeBase;			///< home docking station
} irobotChargingSourcesAvailable_t;

/// iRobot charging state sensor.
typedef enum{
	CHARGING_STATE_NOT_CHARGING				= 0,	///< Robot is not charging
	CHARGING_STATE_RECONDITION_CHARGING		= 1,	///< Robot is in reconditioning charge mode
	CHARGING_STATE_FULL_CHARGING			= 2,	///< Robot has full charge and is charging
	CHARGING_STATE_TRICKLE_CHARGING			= 3,	///< Robot is trickle charging
	CHARGING_STATE_WAITING					= 4,	///< Robot is waiting to charge
	CHARGING_STATE_CHARGING_FAULT_CONDITION	= 5		///< Robot has a charge fault
} irobotChargingState_t;

/// iRobot low-side driver and wheel overcurrent sensors.
typedef struct{
	bool	lowSideDriver0;		///< Overcurrent sensor for low-side driver 0; if true, the device has drawn too much current and has been shut down.
	bool	lowSideDriver1;		///< Overcurrent sensor for low-side driver 1; if true, the device has drawn too much current and has been shut down.
	bool	lowSideDriver2;		///< Overcurrent sensor for low-side driver 2; if true, the device has drawn too much current and has been shut down.
	bool	leftWheel;			///< Overcurrent sensor for the left wheel; if true, the device has drawn too much current and has been shut down.
	bool	rightWheel;			///< Overcurrent sensor for the right wheel; if true, the device has drawn too much current and has been shut down.
} irobotLowSideDriver_WheelOvercurrent_t;

/// Mode (full, safe, passive) of the iRobot.
typedef enum{
	OISTATE_OFF		= 0,	///< Off mode
	OISTATE_PASSIVE	= 1,	///< Passive mode
	OISTATE_SAFE	= 2,	///< Safe mode
	OISTATE_FULL	= 3		///< Full mode
} irobotOIState_t;

/// Sensor Group 0
typedef struct{
	/// iRobot bump and wheel-drop sensors.
	irobotBumps_WheelDrops_t bumps_wheelDrops;

	///Binary interpretation of the infrared wall sensor; if true, the sensor has read above a predefined threshold and indicates a wall (or object) is in	front of the robot.
	bool wall;

	/// Binary interpretation of the left cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffLeft;
	/// Binary interpretation of the front left cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffFrontLeft;
	/// Binary interpretation of the front right cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffFrontRight;
	/// Binary interpretation of the right cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffRight;

	/// Virtual Wall sensor is a particular infrared packet omitted by a virtual wall device, and received by the IR on top of the robot. If the signal is received, the value is true.
	bool virtualWall;

	/// iRobot low-side driver and wheel overcurrent sensors.
	irobotLowSideDriver_WheelOvercurrent_t lowSideDriver_WheelOvercurrent;

	/// Most recently received byte from the IR receiver. A value of 0xFF (255) indicates no byte is being received.
	uint8_t infrared;

	/// iRobot button sensors.
	irobotButtons_t buttons;

	/// Distance (in mm) the robot has traveled since the last sensor packet.
	int16_t distance;

	/// Angle (in degrees) through which the robot has rotated since the last sensor packet.
	int16_t angle;

	/// iRobot charging state sensor.
	irobotChargingState_t chargingState;
	
	/// Voltage of the iRobot battery, in mV.
	uint16_t voltage;

	/// Current (mA) flowing into or out of the battery. Negative currents indicate current is leaving the battery; positive currents (present in charging) indicate current is entering the battery.
	int16_t current;
	
	/// Temperature of the battery, in degrees Celcius.
	int8_t batteryTemperature;

	/// Current charge of the create battery, in mAh.
	uint16_t batteryCharge;
	
	/// Estimated charging capacity of the battery, in mAh.
	uint16_t batteryCapacity;
} irobotSensorGroup0_t;

/// Sensor Group 1
typedef struct{
	/// iRobot bump and wheel-drop sensors.
	irobotBumps_WheelDrops_t bumps_wheelDrops;

	/// Binary interpretation of the infrared wall sensor; if true, the sensor has read above a predefined threshold and indicates a wall (or object) is in	front of the robot.
	bool wall;

	/// Binary interpretation of the left cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffLeft;
	/// Binary interpretation of the front left cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffFrontLeft;
	/// Binary interpretation of the front right cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffFrontRight;
	/// Binary interpretation of the right cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffRight;

	/// Virtual Wall sensor is a particular infrared packet omitted by a virtual wall device, and received by the IR on top of the robot. If the signal is received, the value is true.
	bool virtualWall;

	/// iRobot low-side driver and wheel overcurrent sensors.
	irobotLowSideDriver_WheelOvercurrent_t lowSideDriver_wheelovercurrent;
} irobotSensorGroup1_t;

/// Sensor Group 2
typedef struct{
	/// Most recently received byte from the IR receiver. A value of 0xFF (255) indicates no byte is being received.
	uint8_t infrared;

	/// iRobot button sensors.
	irobotButtons_t buttons;

	/// Distance (in mm) the robot has traveled since the last sensor packet.
	int16_t distance;

	/// Angle (in degrees) through which the robot has rotated since the last sensor packet.
	int16_t angle;
} irobotSensorGroup2_t;

/// Sensor Group 3
typedef struct{
	/// iRobot charging state sensor.
	irobotChargingState_t chargingState;
	
	/// Voltage of the iRobot battery, in mV.
	uint16_t voltage;

	/// Current (mA) flowing into or out of the battery. Negative currents indicate current is leaving the battery; positive currents (present in charging) indicate current is entering the battery.
	int16_t current;
	
	/// Temperature of the battery, in degrees Celcius.
	int8_t batteryTemperature;

	/// Current charge of the create battery, in mAh.
	uint16_t batteryCharge;
	
	/// Estimated charging capacity of the battery, in mAh.
	uint16_t batteryCapacity;
} irobotSensorGroup3_t;

/// Sensor Group 4
typedef struct{
	uint16_t	wallSignal;				///< Sampled value of the analog wall sensor, range 0-4095.
	uint16_t	cliffLeftSignal;		///< Sampled value of the analog left cliff sensor, range 0-4095.
	uint16_t	cliffFrontLeftSignal;	///< Sampled value of the analog front left cliff sensor, range 0-4095.
	uint16_t	cliffFrontRightSignal;	///< Sampled value of the analog front right cliff sensor, range 0-4095.
	uint16_t	cliffRightSignal;		///< Sampled value of the analog right cliff sensor, range 0-4095.

	/// digital inputs
	irobotCargoBayDigitalInputs_t	cargoBayDigitalInputs;

	/// sampled cargo bay analog line; range 0-1023
	uint16_t	cargoBayAnalogSignal;

	/// charging sources available
	irobotChargingSourcesAvailable_t	chargingSourcesAvailable;
} irobotSensorGroup4_t;

/// Sensor Group 5
typedef struct{
	/// irobot OI mode
	irobotOIState_t	oiState;

	uint8_t			songNumber;		///< index of the song that is loaded or is playing
	bool			songPlaying;	///< song is playing?

	uint8_t			nStreamPackets;	///< number of packets in each stream broadcast

	int16_t			requestedVelocity;	///< requested velocity, in mm/s for DRIVE mode
	int16_t			requestedRadius;	///< requested radius, in mm for DRIVE mode

	int16_t			requestedRightWheelVelocity;	///< requested right wheel velocity in mm/s for DRIVE_DIRECT mode
	int16_t			requestedLeftWheelVelocity;		///< requested left wheel velocity in mm/s for DRIVE_DIRECT mode
} irobotSensorGroup5_t;

/// Sensor Group 6
typedef struct{
	/// iRobot bump and wheel-drop sensors.
	irobotBumps_WheelDrops_t bumps_wheelDrops;

	/// Binary interpretation of the infrared wall sensor; if true, the sensor has read above a predefined threshold and indicates a wall (or object) is in	front of the robot.
	bool wall;

	/// Binary interpretation of the left cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffLeft;
	/// Binary interpretation of the front left cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffFrontLeft;
	/// Binary interpretation of the front right cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffFrontRight;
	/// Binary interpretation of the right cliff sensor; if true, the sensor has read below a predefined threshold and indicates the sensor is not immediately above ground.
	bool cliffRight;

	/// Virtual Wall sensor is a particular infrared packet omitted by a virtual wall device, and received by the IR on top of the robot. If the signal is received, the value is true.
	bool virtualWall;

	/// iRobot low-side driver and wheel overcurrent sensors.
	irobotLowSideDriver_WheelOvercurrent_t lowSideDriver_WheelOvercurrent;

	/// Most recently received byte from the IR receiver. A value of 0xFF (255) indicates no byte is being received.
	uint8_t infrared;

	/// iRobot button sensors.
	irobotButtons_t buttons;

	/// Distance (in mm) the robot has traveled since the last sensor packet.
	int16_t distance;

	/// Angle (in degrees) through which the robot has rotated since the last sensor packet.
	int16_t angle;

	/// iRobot charging state sensor.
	irobotChargingState_t chargingState;
	
	/// Voltage of the iRobot battery, in mV.
	uint16_t voltage;

	/// Current (mA) flowing into or out of the battery. Negative currents indicate current is leaving the battery; positive currents (present in charging) indicate current is entering the battery.
	int16_t current;
	
	/// Temperature of the battery, in degrees Celcius.
	int8_t batteryTemperature;

	/// Current charge of the create battery, in mAh.
	uint16_t batteryCharge;
	
	///  Estimated charging capacity of the battery, in mAh.
	uint16_t batteryCapacity;

	uint16_t	wallSignal;				///< Sampled value of the analog wall sensor, range 0-4095.
	uint16_t	cliffLeftSignal;		///< Sampled value of the analog left cliff sensor, range 0-4095.
	uint16_t	cliffFrontLeftSignal;	///< Sampled value of the analog front left cliff sensor, range 0-4095.
	uint16_t	cliffFrontRightSignal;	///< Sampled value of the analog front right cliff sensor, range 0-4095.
	uint16_t	cliffRightSignal;		///< Sampled value of the analog right cliff sensor, range 0-4095.

	/// digital inputs
	irobotCargoBayDigitalInputs_t	cargoBayDigitalInputs;

	/// sampled cargo bay analog line; range 0-1023
	uint16_t	cargoBayAnalogSignal;

	/// charging sources available
	irobotChargingSourcesAvailable_t	chargingSourcesAvailable;

	/// irobot OI mode
	irobotOIState_t	oiState;

	uint8_t			songNumber;		///< index of the song that is loaded or is playing
	bool			songPlaying;	///< song is playing?

	uint8_t			nStreamPackets;	///< number of packets in each stream broadcast

	int16_t			requestedVelocity;	///< requested velocity, in mm/s for DRIVE mode
	int16_t			requestedRadius;	///< requested radius, in mm for DRIVE mode

	int16_t			requestedRightWheelVelocity;	///< requested right wheel velocity in mm/s for DRIVE_DIRECT mode
	int16_t			requestedLeftWheelVelocity;		///< requested left wheel velocity in mm/s for DRIVE_DIRECT mode
} irobotSensorGroup6_t;


#endif // _IROBOT_SENSORTYPES_H
