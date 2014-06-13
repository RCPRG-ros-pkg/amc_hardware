#if !defined(AXIS_DRIVE_H)
#define AXIS_DRIVE_H

#include "ethercat_driver.h"

#include <rtt/Port.hpp>

#include <ethercattype.h>

class AxisDrive : public ECDriver
{
public:
	enum DriveState {Unknown, NotReadyToSwitchOn, SwitchOnDisabled, ReadyToSwitchOn, SwitchedOn, OperationEnabled, Fault, QuickStopActive};
	enum StateCommand {ResetFault, DisableVoltage, ShutDown, SwitchOn, EnableOperation, QuickStop, BeginHoming};
	enum Mode {ProfilePosition=0x01, ProfileVelocity=0x03, ProfileTorque=0x04, Homing=0x06, CyclicSynchronousPosition=0x08, CyclicSynchronousVelocity=0x09, CyclicSynchronousTorque=0x0A, Config0=0x9E, Config1=0xDE, None=0xFF};

	AxisDrive(const std::string &neme, ecx_contextt* contextt_ptr, uint16_t slave_id);
	~AxisDrive();
	bool configure();
	bool start();
	void update();
	void stop();
	
private:
	
	DriveState getState();

	bool enterState(StateCommand cmd);

	void configureHoming();
	bool beginHoming();
	void waitForHomingComplete();
	bool isHomingCompleted();

	void enterMode(Mode mode);
	void printMode(Mode mode);
	Mode getMode();
	void setCurrent(double current, bool offset);
	void setVelocity(double velocity, bool offset);
	void setPosition(double position);
	double getActualCurrent();
	double getActualVelocity();
	double getActualPosition();

	void readPositionLimits();
	void readHistory();
	void resetHistory();
	void printState(DriveState state);
	void readEvents();
	void printDriveStatus();
  bool enableOperation();
  void processStatus();
protected:
	DriveState translateState(uint16 statusWord);
	bool enterStateNoCheck(StateCommand cmd);
	bool enterStateProfile(StateCommand cmd);
	bool enterStateCyclicSynchronous(StateCommand cmd);

	bool readConversionConstants();

#pragma pack(1)
	typedef struct
	{
		uint16 controlWord;
		int32 targetPosition;
		int32 targetVelocity;
		int16 targetCurrent;
		int16 currentOffset;
		int32 velocityOffset;
		uint16 userBits;
	} RPDO;

	typedef struct
	{
		uint16 statusWord;
		int32 actualPosition;
		int32 actualVelocity;
		int16 actualCurrent;
		uint16 digitalInputs;
	} TPDO;
#pragma pack(0)

	RPDO *rpdo_;
	TPDO *tpdo_;

	// unit conversion routines
	int32 DA1toDrive(double da1);
	double DrivetoDA1(int32 drive);
	int32 DS4toDrive(double ds4);
	double DrivetoDS4(int32 drive);
	int16 DC2toDrive(double dc2);
	double DrivetoDC2(int16 drive);

	Mode actualMode_;
	Mode mode_;
	uint16 conv_KB;
	uint16 conv_KDS0;
	uint16 conv_KDS1;
	uint16 conv_KDS2;
	uint16 conv_KDS3;
	uint16 conv_KOV_PBV;
	uint16 conv_KP_PBC;
	uint32 conv_KS_PBF;
	double conv_KOV_V;
	double conv_KP_A;
	double conv_KS_HZ;

	// values used for unit conversion
	double conv_DA1;
	double conv_DS4;
	double conv_DC2;

	double encTicks;
	
	bool homing_compleat_;
	int servo_state_;
	
	bool enable_;
	bool homing_;
	
	std::string mode_param_;
	
	StateCommand target_state_;
	
	RTT::OutputPort<double> port_motor_position_;
	RTT::OutputPort<double> port_motor_velocity_;
	RTT::OutputPort<double> port_motor_current_;
	
	RTT::InputPort<double> port_motor_position_command_;
	RTT::InputPort<double> port_motor_velocity_command_;
	RTT::InputPort<double> port_motor_current_command_;
};

#endif	// AXIS_DRIVE_H


