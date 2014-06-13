#include "AxisDrive.h"

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatcoe.h>
#include <ethercatdc.h>
#include <ethercatprint.h>

#include <rtt/RTT.hpp>

//
#include <stdio.h>
//

const uint16 StatusWord = 0x6041;
const uint16 ControlWord = 0x6040;

const uint16 SW_ReadyToSwitchOn_Mask = 0x0001;
const uint16 SW_SwitchedOn_Mask = 0x0002;
const uint16 SW_OperationEnabled_Mask = 0x0004;
const uint16 SW_Fault_Mask = 0x0008;
const uint16 SW_VoltageEnabled_Mask = 0x0010;
const uint16 SW_QuickStop_Mask = 0x0020;
const uint16 SW_SwitchOnDisabled_Mask = 0x0040;
const uint16 SW_Warning_Mask = 0x0080;
const uint16 SW_ManufactureSpecific_Mask = 0x0100;
const uint16 SW_Remote_Mask = 0x0200;
const uint16 SW_TargetReached_Mask = 0x0400;
const uint16 SW_InternalLimitActive_Mask = 0x0800;
const uint16 SW_HomingComplete_Mask = 0x1000;

const uint16 NotReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_SwitchOnDisabled_Mask;
const uint16 NotReadyToSwitchOn_Pattern = 0x0000;

const uint16 SwitchOnDisabled_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_SwitchOnDisabled_Mask;
const uint16 SwitchOnDisabled_Pattern = SW_SwitchOnDisabled_Mask;

const uint16 ReadyToSwitchOn_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16 ReadyToSwitchOn_Pattern = SW_ReadyToSwitchOn_Mask | SW_QuickStop_Mask;

const uint16 SwitchedOn_Mask = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_Fault_Mask | SW_QuickStop_Mask | SW_SwitchOnDisabled_Mask;
const uint16 SwitchedOn_Pattern = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_QuickStop_Mask;

const uint16 OperationEnabled_Mask = SW_ReadyToSwitchOn_Mask	|SW_SwitchedOn_Mask|SW_OperationEnabled_Mask|SW_Fault_Mask|SW_QuickStop_Mask|SW_SwitchOnDisabled_Mask;
const uint16 OperationEnabled_Pattern = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask | SW_QuickStop_Mask;

const uint16 Fault_Mask = SW_ReadyToSwitchOn_Mask|SW_SwitchedOn_Mask|SW_OperationEnabled_Mask|SW_Fault_Mask|SW_SwitchOnDisabled_Mask;
const uint16 Fault_Pattern = SW_Fault_Mask;

const uint16 QuickStopActive_Mask = SW_ReadyToSwitchOn_Mask|SW_SwitchedOn_Mask|SW_OperationEnabled_Mask|SW_Fault_Mask|SW_QuickStop_Mask|SW_SwitchOnDisabled_Mask;
const uint16 QuickStopActive_Pattern = SW_ReadyToSwitchOn_Mask | SW_SwitchedOn_Mask | SW_OperationEnabled_Mask;

const char *driveBridgeStatusHistoryStrings[16] = {
"Bridge Enabled",
"Dynamic Brake Enabled",
"Stop Enabled",
"Positive Stop Enabled",
"Negative Stop Enabled",
"Positive Torque Inhibit Active",
"Negative Torque Inhibit Active",
"External Brake Active",
"Reserved1",
"Reserved2",
"Reserved3",
"Reserved4",
"Reserved5",
"Reserved6",
"Reserved7",
"Reserved8"
};

const char *driveProtectionStatusHistoryStrings[16] = {
"Drive Reset",
"Drive Internal Error",
"Short Circuit",
"Current Overshot",
"Under Voltage",
"Over Voltage",
"Drive Over Temperature",
"Reserved9",
"Reserved10",
"Reserved11",
"Reserved12",
"Reserved13",
"Reserved14",
"Reserved15",
"Reserved16",
"Reserved17"
};

const char *systemProtectionStatusHistoryStrings[16] = {
"Parameter Restore Error",
"Parameter Store Error",
"Invalid Hall State",
"Phase Sync. Error",
"Motor Over Temperature",
"Phase Detection Fault",
"Feedback Sensor Error",
"Motor Over Speed",
"Max Measured Position",
"Min Measured Position",
"Comm. Error (Node Guarding)",
"PWM Input Broken Wire",
"Reserved18",
"Reserved19",
"Reserved20",
"Reserved21"
};

const char *driveSystemStatus1HistoryStrings[16] = {
"Log Entry Missed",
"Software Disable",
"User Disable",
"User Positive Inhibit",
"User Negative Inhibit",
"Current Limiting",
"Continous Current",
"Current Loop Saturated",
"User Under Voltage",
"User Over Voltage",
"Non-sinusoidal Commutation",
"Phase Detection",
"Reserved22",
"User Auxiliary Disable",
"Shunt Regulator",
"Phase Detection Complete"
};

const char *driveSystemStatus2HistoryStrings[16] = {
"Zero Velocity",
"At Command",
"Velocity Following Error",
"Positive Target Velocity Limit",
"Negative Target Velocity Limit",
"Command Limitter Active",
"In Home Position",
"Position Following Error",
"Max Target Position Limit",
"Min Target Position Limit",
"Set Position",
"Reserved23",
"Homing Active",
"Reserved24",
"Homing Complete",
"Zero Position Error"
};

const char *driveSystemStatus3HistoryStrings[16] = {
"Reserved25",
"Reserved26",
"Reserved27",
"Reserved28",
"Reserved29",
"Reserved30",
"Commanded Stop",
"User Stop",
"Capture 1 Triggered",
"Capture 2 Triggered",
"Capture 3 Triggered",
"Commanded Positive Limit",
"Commanded Negative Limit",
"Reserved31",
"Reserved32",
"Reserved33"
};

const char *eventActionStrings[49] = {
"Event Action: Parameter Restore Error",
"Event Action: Parameter Store Error",
"Event Action: Invalid Hall State",
"Event Action: Phase Synch Error",
"Event Action: Motor Over Temperature",
"Event Action: Feedback Sensor Error",
"Event Action: Log Entry Missed",
"Event Action: Current Limiting",
"Event Action: Continuous Current",
"Event Action: Current Loop Saturated",
"Event Action: User Under Voltage",
"Event Action: User Over Voltage",
"Event Action: Shunt Regulator",
"Event Action: Command Limiter Active",
"Event Action: Motor Over Speed",
"Event Action: At Command",
"Event Action: Zero Velocity",
"Event Action: Velocity Following Error",
"Event Action: Positive Velocity Limit",
"Event Action: Negative Velocity Limit",
"Event Action: Max Measured Position Limit",
"Event Action: Min Measured Position Limit",
"Event Action: At Home Position",
"Event Action: Position Following Error",
"Event Action: Max Target Position Limit",
"Event Action: Min Target Position Limit",
"Reserved",
"Reserved",
"Reserved",
"Reserved",
"Reserved",
"Reserved",
"Event Action: Communication Error",
"Event Action: User Positive Limit",
"Event Action: User Negative Limit",
"Event Action: Drive Reset",
"Event Action: Drive Internal Error",
"Event Action: Short Circuit",
"Event Action: Current Overshoot",
"Event Action: Hardware Under Voltage",
"Event Action: Hardware Over Voltage",
"Event Action: Drive Over Temperature",
"Event Action: Software Disable",
"Event Action: User Disable",
"Event Action: User Auxiliary Disable",
"Event Action: Phase Detection Fault",
"Event Action: Commanded Positive Limit",
"Event Action: Commanded Negative Limit",
"Event Action: PWM and Direction Broken Wire"
};

const char *actionsStrings[12] = {
"No Action",
"Disable Power Bridge",
"Disable Positive Direction",
"Disable Negative Direction",
"Dynamic Brake",
"Positive Stop",
"Negative Stop",
"Stop",
"Apply Brake then Disable Bridge",
"Apply Brake then Dynamic Brake",
"Apply Brake and Disable Bridge",
"Apply Brake and Dynamic Brake"
};

void AxisDrive::readEvents()
{
	RTT::log(RTT::Info) << "readEvents: ";
	uint16 ev = 0;
	int size = sizeof(ev);
	for (int i=0; i<48; ++i)
	{
		ecx_SDOread(contextt_ptr_, slave_id_, 0x2065, i+1, FALSE, &size, &ev, EC_TIMEOUTRXM);
		RTT::log(RTT::Info) << eventActionStrings[i] << "(" << actionsStrings[ev] << "), ";
	}
	RTT::log(RTT::Info) << RTT::endlog();
}

void AxisDrive::readPositionLimits()
{
	printf("readPositionLimits():\n");
	int32 pos = 0;
	int size = sizeof(pos);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2039, 0x03, FALSE, &size, &pos, EC_TIMEOUTRXM);
	printf(" Max Measured Position Limit: %d\n", pos);
	size = sizeof(pos);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2039, 0x04, FALSE, &size, &pos, EC_TIMEOUTRXM);
	printf(" Min Measured Position Limit: %d\n", pos);
	size = sizeof(pos);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2039, 0x08, FALSE, &size, &pos, EC_TIMEOUTRXM);
	printf(" Max Target Position Limit:   %d\n", pos);
	size = sizeof(pos);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2039, 0x09, FALSE, &size, &pos, EC_TIMEOUTRXM);
	printf(" Min Target Position Limit:   %d\n", pos);
	size = sizeof(pos);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2037, 0x01, FALSE, &size, &pos, EC_TIMEOUTRXM);
	printf(" Motor Over Speed Limit:      %d\n", pos);
	size = sizeof(pos);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2037, 0x02, FALSE, &size, &pos, EC_TIMEOUTRXM);
	printf(" Zero Speed Limit:      %d\n", pos);
	size = sizeof(pos);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2037, 0x05, FALSE, &size, &pos, EC_TIMEOUTRXM);
	printf(" Positive Velocity Limit:      %d\n", pos);
	size = sizeof(pos);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2037, 0x06, FALSE, &size, &pos, EC_TIMEOUTRXM);
	printf(" Negative Velocity Limit:      %d\n", pos);
}

void AxisDrive::printDriveStatus()
{
	// 2002h: Drive Status
	// 01h: Drive Bridge Status History
	// 02h: Drive Protection Status History
	// 03h: System Protection Status History
	// 04h: Drive/System Status 1 History
	// 05h: Drive/System Status 2 History
	// 06h: Drive/System Status 3 History
	printf("driveStatus: ");
	int size = 2;
	uint16 word = 0x0000;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2002, 0x01, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveBridgeStatusHistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2002, 0x02, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveProtectionStatusHistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2002, 0x03, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", systemProtectionStatusHistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2002, 0x04, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveSystemStatus1HistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2002, 0x05, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveSystemStatus2HistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2002, 0x06, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveSystemStatus3HistoryStrings[i]);
	printf("\n");
}

void AxisDrive::readHistory()
{
	// 2003h: Drive Status History
	// 01h: Drive Bridge Status History
	// 02h: Drive Protection Status History
	// 03h: System Protection Status History
	// 04h: Drive/System Status 1 History
	// 05h: Drive/System Status 2 History
	// 06h: Drive/System Status 3 History
	printf("readHistory(): ");
	int size = 2;
	uint16 word = 0x0000;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2003, 0x01, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveBridgeStatusHistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2003, 0x02, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveProtectionStatusHistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2003, 0x03, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", systemProtectionStatusHistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2003, 0x04, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveSystemStatus1HistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2003, 0x05, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveSystemStatus2HistoryStrings[i]);
	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2003, 0x06, FALSE, &size, &word, EC_TIMEOUTRXM);
	for (int i=0; i<16;++i)
		if ( (word&(1<<i)) != 0 )
			printf("%s, ", driveSystemStatus3HistoryStrings[i]);
	printf("\n");

	size = 2;
	ecx_SDOread(contextt_ptr_, slave_id_, 0x2028, 0x11, FALSE, &size, &word, EC_TIMEOUTRXM);
	printf(" Log Counter: Software Disable: %d\n", (int)word);

	resetHistory();
}

void AxisDrive::resetHistory()
{
	// 2003h: Drive Status History
	// 01h: Drive Bridge Status History
	// 02h: Drive Protection Status History
	// 03h: System Protection Status History
	// 04h: Drive/System Status 1 History
	// 05h: Drive/System Status 2 History
	// 06h: Drive/System Status 3 History

	printf("resetHistory()\n");
	uint16 word = 0xFFFF;
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x2003, 0x01, FALSE, sizeof(word), &word, EC_TIMEOUTRXM);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x2003, 0x02, FALSE, sizeof(word), &word, EC_TIMEOUTRXM);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x2003, 0x03, FALSE, sizeof(word), &word, EC_TIMEOUTRXM);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x2003, 0x04, FALSE, sizeof(word), &word, EC_TIMEOUTRXM);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x2003, 0x05, FALSE, sizeof(word), &word, EC_TIMEOUTRXM);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x2003, 0x06, FALSE, sizeof(word), &word, EC_TIMEOUTRXM);
}

void AxisDrive::printState(AxisDrive::DriveState state)
{
	switch (state)
	{
	case Unknown:
		printf("Unknown\n");
		break;
	case NotReadyToSwitchOn:
		printf("NotReadyToSwitchOn\n");
		break;
	case SwitchOnDisabled:
		printf("SwitchOnDisabled\n");
		break;
	case ReadyToSwitchOn:
		printf("ReadyToSwitchOn\n");
		break;
	case SwitchedOn:
		printf("SwitchedOn\n");
		break;
	case OperationEnabled:
		printf("OperationEnabled\n");
		break;
	case Fault:
		printf("Fault\n");
		break;
	case QuickStopActive:
		printf("QuickStopActive\n");
		break;
	default:
		printf("WRONG\n");
	}
}

void AxisDrive::printMode(AxisDrive::Mode mode)
{
	switch (mode)
	{
	case ProfilePosition:
		printf("ProfilePosition");
		break;
	case ProfileVelocity:
		printf("ProfileVelocity");
		break;
	case ProfileTorque:
		printf("ProfileTorque");
		break;
	case Homing:
		printf("Homing");
		break;
	case CyclicSynchronousPosition:
		printf("CyclicSynchronousPosition");
		break;
	case CyclicSynchronousVelocity:
		printf("CyclicSynchronousVelocity");
		break;
	case CyclicSynchronousTorque:
		printf("CyclicSynchronousTorque");
		break;
	case Config0:
		printf("Config0");
		break;
	case Config1:
		printf("Config1");
		break;
	case None:
		printf("None");
		break;
	default:
		printf("wrong mode");
		break;
	}
}

