#include "AxisDrive.h"

#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatcoe.h>
#include <ethercatdc.h>
#include <ethercatprint.h>

#include <native/task.h>
#include <rtt/RTT.hpp>

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

AxisDrive::AxisDrive(const std::string &name, ecx_contextt* contextt_ptr, uint16_t slave_id) : ECDriver(name, contextt_ptr, slave_id),
	conv_DA1(1.0),
	conv_DS4(1.0),
	conv_DC2(1.0),
	encTicks(2000),
	rpdo_(NULL),
	tpdo_(NULL),
	homing_compleat_(false),
	servo_state_(0),
	mode_(CyclicSynchronousPosition),
	enable_(false),
	homing_(false)
{
	rpdo_=(RPDO*)contextt_ptr->slavelist[slave_id].outputs;
	tpdo_=(TPDO*)contextt_ptr->slavelist[slave_id].inputs;
	memset(rpdo_, 0, sizeof(RPDO));
	memset(tpdo_, 0, sizeof(TPDO));
	
	std::cout << "outputs : " << (long)contextt_ptr->slavelist[slave_id].outputs << std::endl;
	std::cout << "inputs : " << (long)contextt_ptr->slavelist[slave_id].inputs << std::endl;
	
	this->provides()->addConstant("homingCompleat", homing_compleat_);
	this->provides()->addConstant("servo_state", servo_state_);
		
  this->provides()->addProperty("mode", mode_param_);
		
	this->provides()->addPort("MotorPosition", port_motor_position_);
	this->provides()->addPort("MotorVelocity", port_motor_velocity_);
	this->provides()->addPort("MotorCurrent", port_motor_current_);
	
	this->provides()->addPort("MotorPositionCommand", port_motor_position_command_);
	this->provides()->addPort("MotorVelocityCommand", port_motor_velocity_command_);
	this->provides()->addPort("MotorCurrentCommand", port_motor_current_command_);
	
	this->provides()->addPort("DigitalOutput1", port_do1_command_);
	this->provides()->addPort("DigitalOutput2", port_do2_command_);
	this->provides()->addPort("DigitalOutput3", port_do3_command_);
	this->provides()->addPort("DigitalOutput4", port_do4_command_);
	
	this->provides()->addOperation("beginHoming",	&AxisDrive::beginHoming, this, RTT::OwnThread);
	//this->provides()->addOperation("isHomingComplete",	&AxisDrive::isHomingComplete, this, RTT::OwnThread);
	this->provides()->addOperation("enableOperation",	&AxisDrive::enableOperation, this, RTT::OwnThread);
}

AxisDrive::~AxisDrive() {
}

bool AxisDrive::enableOperation() {
  if (isHomingCompleted()) {
    enterMode(mode_);
    enable_ = true;
    return true;
  } else {
    return false;
  }
}

void AxisDrive::processStatus() {
  homing_compleat_ = isHomingCompleted();
  servo_state_ = getState();
}

bool AxisDrive::configure() {
	enterMode(ProfileTorque);

	if (getState() == Fault) {
		RTT::log(RTT::Warning) << "AxisDrive::init: fault detected" << RTT::endlog();
		if (!enterStateNoCheck(ResetFault)) {
			RTT::log(RTT::Error) << "AxisDrive::init: could not recover from fault" << RTT::endlog();
			return false;
		}
	}
	
	if (!enterStateNoCheck(DisableVoltage)) {
		RTT::log(RTT::Error) << "AxisDrive::init: could not change state to VoltageDisabled" << RTT::endlog();
		return false;
	}

	if (!readConversionConstants()) {
		RTT::log(RTT::Error) << "AxisDrive::init: could not read conversion constants" << RTT::endlog();
		return false;
	}

	//readPositionLimits();

	if (!enterStateNoCheck(ShutDown)) {
		RTT::log(RTT::Error) << "AxisDrive::init: could not change state to ReadyToSwitchOn" << RTT::endlog();
		return false;
	}
  
	uint8 mantissa;
	int8 exponent;
	int size;
	size = sizeof(mantissa);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x60C2, 0x01, FALSE, &size, &mantissa, EC_TIMEOUTRXM);

	size = sizeof(exponent);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x60C2, 0x02, FALSE, &size, &exponent, EC_TIMEOUTRXM);
	RTT::log(RTT::Warning) << "AxisDrive::init: Interpolation Time Period: " << (int)mantissa << "*10^" << (int)exponent << RTT::endlog();
	mantissa = 1;
	exponent = -3;
	size = sizeof(exponent);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x60C2, 0x02, FALSE, size, &exponent, EC_TIMEOUTRXM);
	size = sizeof(mantissa);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x60C2, 0x01, FALSE, size, &mantissa, EC_TIMEOUTRXM);

	return true;
}

bool AxisDrive::start() {
  if (mode_param_ == "position") {
    mode_ = CyclicSynchronousPosition;
  } else if (mode_param_ == "velocity") {
    mode_ = CyclicSynchronousVelocity;
  } else if (mode_param_ == "current") {
    mode_ = CyclicSynchronousTorque;
  } else {
    return false;
  }
  return true;
}

void AxisDrive::update() {
  double current, velocity, position;
  
  processStatus();
  
  if (getState() == AxisDrive::SwitchOnDisabled) {
		// go to ReadyToSwitchOn state
		target_state_ = ShutDown;
	} else if (getState() == AxisDrive::ReadyToSwitchOn) {
		// go to OperationDisabled state
		target_state_ = SwitchOn;
	} else if (getState() == AxisDrive::SwitchedOn) {
	  //std::cout << "homing : " << target_state_  << std::endl;
	  enterMode(Homing);
	  target_state_ = BeginHoming;
	  homing_ = true;
	}
	
	if (isHomingCompleted()) {
	  homing_ = false;
	  enterMode(mode_);
	  target_state_ = EnableOperation;
	}
  
  enterStateNoCheck(target_state_);
  
  if (port_motor_position_command_.read(position) == RTT::NewData) {
    setPosition(position);
  }

  if (port_motor_velocity_command_.read(velocity) == RTT::NewData) {
    setVelocity(velocity, (actualMode_ == CyclicSynchronousPosition));
  }
  
  if (port_motor_current_command_.read(current) == RTT::NewData) {
    bool offset = (actualMode_ == CyclicSynchronousPosition) || (actualMode_ == CyclicSynchronousVelocity);
    setCurrent(current, offset);
  }
  
  bool dio;
  
  if (port_do1_command_.read(dio) == RTT::NewData) {
    if (dio) {
      rpdo_->userBits |= 1<<0;
    } else {
      rpdo_->userBits &= ~(1<<0);
    }
  }
  
  if (port_do2_command_.read(dio) == RTT::NewData) {
    if (dio) {
      rpdo_->userBits |= 1<<1;
    } else {
      rpdo_->userBits &= ~(1<<1);
    }
  }
  
  if (port_do3_command_.read(dio) == RTT::NewData) {
    if (dio) {
      rpdo_->userBits |= 1<<2;
    } else {
      rpdo_->userBits &= ~(1<<2);
    }
  }
  
  if (port_do4_command_.read(dio) == RTT::NewData) {
    if (dio) {
      rpdo_->userBits |= 1<<3;
    } else {
      rpdo_->userBits &= ~(1<<3);
    }
  }
  
  port_motor_position_.write(getActualPosition());
  port_motor_velocity_.write(getActualVelocity());
  port_motor_current_.write(getActualCurrent());
}

void AxisDrive::stop() {
  enterState(AxisDrive::ShutDown);
}

AxisDrive::DriveState AxisDrive::getState()
{
	uint16 status = 0;
	if (contextt_ptr_->slavelist[slave_id_].state == EC_STATE_OPERATIONAL) {
    status = tpdo_->statusWord;
	} else {
		int size = sizeof(status);
		int ret;
		ret = ecx_SDOread(contextt_ptr_, slave_id_, StatusWord, 0, FALSE, &size, &status, EC_TIMEOUTRXM);
		if (ret<1) {
			return Unknown;
	  }
	}
	return translateState(status);
}

AxisDrive::DriveState AxisDrive::translateState(uint16 statusWord)
{
	DriveState state = Unknown;

	if ((statusWord&NotReadyToSwitchOn_Mask) == NotReadyToSwitchOn_Pattern)
		state = NotReadyToSwitchOn;
	else if ((statusWord&SwitchOnDisabled_Mask) == SwitchOnDisabled_Pattern)
		state = SwitchOnDisabled;
	else if ((statusWord&ReadyToSwitchOn_Mask) == ReadyToSwitchOn_Pattern)
		state = ReadyToSwitchOn;
	else if ((statusWord&SwitchedOn_Mask) == SwitchedOn_Pattern)
		state = SwitchedOn;
	else if ((statusWord&OperationEnabled_Mask) == OperationEnabled_Pattern)
		state = OperationEnabled;
	else if ((statusWord&Fault_Mask) == Fault_Pattern)
		state = Fault;
	else if ((statusWord&QuickStopActive_Mask) == QuickStopActive_Pattern)
		state = QuickStopActive;

	return state;
}

bool AxisDrive::enterState(AxisDrive::StateCommand cmd)
{
	DriveState old = getState();

	if (old == SwitchOnDisabled && cmd == ShutDown)
	{
		return enterStateNoCheck(cmd);
	}
	else if (old == ReadyToSwitchOn && (cmd == DisableVoltage || cmd == SwitchOn))
	{
		return enterStateNoCheck(cmd);
	}
	else if (old == SwitchedOn && (cmd == DisableVoltage || cmd == ShutDown || cmd == EnableOperation))
	{
		return enterStateNoCheck(cmd);
	}
	else if (old == OperationEnabled && (cmd == DisableVoltage || cmd == SwitchOn || cmd == QuickStop))
	{
		return enterStateNoCheck(cmd);
	}
	else if (old == Fault && cmd == ResetFault)
	{
		return enterStateNoCheck(cmd);
	}
	else if (old == QuickStopActive && (cmd == DisableVoltage || cmd == EnableOperation))
	{
		return enterStateNoCheck(cmd);
	}

	return false;
}

bool AxisDrive::enterStateNoCheck(AxisDrive::StateCommand cmd)
{
// 6040h: ControlWord (uint16)
/*
	80 Reset Fault On any transition to "1" of bit 7 causes a Reset Fault
	04 Disable Voltage Drive in "Switch On Disabled" state
	06 Shutdown Drive in "Ready to Switch On" state
	07 Switch On Drive in "Switched On" state
	0F Enable Operation Drive in "Operation Enabled" state
	02 Stop Drive in "Stop Active" state
	1F Start Homing Starts Homing (when in homing mode)
	0F End Homing Ends Homing
*/	
	int size = 2;
	uint16 controlWord = 0;
	switch (cmd)
	{
	case DisableVoltage:
		controlWord = 0x04;
		break;
	case ShutDown:
		controlWord = 0x06;
		break;
	case SwitchOn:
		controlWord = 0x07;
		break;
	case EnableOperation:
		controlWord = 0x0F;
		break;
	case ResetFault:
		controlWord = 0x00;
		if (ecx_SDOwrite(contextt_ptr_, slave_id_, ControlWord, 0x00, FALSE, sizeof(controlWord), &controlWord, EC_TIMEOUTRXM) < 1)
			return false;
		rt_task_sleep(5*1000000);
		controlWord = 0x80;
		if (ecx_SDOwrite(contextt_ptr_, slave_id_, ControlWord, 0x00, FALSE, sizeof(controlWord), &controlWord, EC_TIMEOUTRXM) < 1)
			return false;
		rt_task_sleep(5*1000000);
		return true;
		break;
	case QuickStop:
		controlWord = 0x02;
		break;
	case BeginHoming:
		controlWord = 0x1F;
		break;
	default:
		return false;
		break;
	}


  //std::cout << "controlWord : " << controlWord << std::endl;
  rpdo_->controlWord = controlWord;
  //std::cout << "controlWord : " << controlWord << std::endl;
	if (contextt_ptr_->slavelist[slave_id_].state != EC_STATE_OPERATIONAL) {

		if (ecx_SDOwrite(contextt_ptr_, slave_id_, ControlWord, 0x00, FALSE, size, &controlWord, EC_TIMEOUTRXM) < 1) {
			return false;
		}

    //std::cout << "dupa" << std::endl;

		rt_task_sleep(1*1000000);
		return true;
	}
	
	return true;
}

void AxisDrive::configureHoming()
{
	// 6098h: Homing Method (int8, method)
	uint8 homingMethod = 4;
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x6098, 0x00, FALSE, sizeof(homingMethod), &homingMethod, EC_TIMEOUTRXM);

/*
	if (homingMethod == 35)
	{
		int32 measuredPosition = 0;
		ecx_SDOwrite(slave_id_, 0x2039, 0x01, FALSE, sizeof(measuredPosition), &measuredPosition, EC_TIMEOUTRXM);

		int32 homePosition = 0;
		ecx_SDOwrite(slave_id_, 0x2039, 0x02, FALSE, sizeof(homePosition), &homePosition, EC_TIMEOUTRXM);
	}
*/
	// 6099.01h: Speed During Search For Switch (uint32, DS4)
	uint32 speedSwitch = DS4toDrive(4000);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x6099, 0x01, FALSE, sizeof(speedSwitch), &speedSwitch, EC_TIMEOUTRXM);
	//printf("speedSwitch: %d\n", speedSwitch);

	// 6099.02h: Speed During Search For Zero (uint32, DS4)
	uint32 speedZero = DS4toDrive(2000);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x6099, 0x02, FALSE, sizeof(speedZero), &speedZero, EC_TIMEOUTRXM);
	//printf("speedZero: %d\n", speedZero);

	// 609Ah: Homing Acceleration (uint32, DA1)
	uint32 accel = DA1toDrive(10000);
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x609A, 0x00, FALSE, sizeof(accel), &accel, EC_TIMEOUTRXM);
	//printf("accel: %d\n", accel);

	// 607Ch: Home Offset (int32, counts)
	int32 offset = 0;
	ecx_SDOwrite(contextt_ptr_, slave_id_, 0x607C, 0x00, FALSE, sizeof(offset), &offset, EC_TIMEOUTRXM);
}

bool AxisDrive::beginHoming()
{
	//DriveState old = getState();
	//if (old == ReadyToSwitchOn)
	//{
  //enterMode(Homing);
	//return enterStateNoCheck(BeginHoming);
	//}
	//RTT::log(RTT::Error) << "AxisDrive::beginHoming: wrong state" << old << RTT::endlog();
	//return false;
	
	homing_ = true;
  return true;
}

void AxisDrive::waitForHomingComplete()
{
	while (1)
	{
		rt_task_sleep(5*1000000);
		if (isHomingCompleted())
			break;
	}
	enterStateNoCheck(ShutDown);
}

bool AxisDrive::isHomingCompleted()
{
	return (tpdo_->statusWord & SW_HomingComplete_Mask)!=0;
}

void AxisDrive::enterMode(Mode mode)
{
	int8 modeOfOperation = (int8)mode;
	if (ecx_SDOwrite(contextt_ptr_, slave_id_, 0x6060, 0x00, FALSE, sizeof(modeOfOperation), &modeOfOperation, EC_TIMEOUTRXM)>0)
	{
		actualMode_ = mode;
	}
}

AxisDrive::Mode AxisDrive::getMode()
{
	int8 modeOfOperation = 0;
	int size = sizeof(modeOfOperation);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x6061, 0x00, FALSE, &size, &modeOfOperation, EC_TIMEOUTRXM);
	return (Mode)modeOfOperation;
}

void AxisDrive::setCurrent(double current, bool offset)
{
	if (offset)
	{
		rpdo_->currentOffset = DC2toDrive(current);
	}
	else
	{
		rpdo_->targetCurrent = DC2toDrive(current);
	}
}

void AxisDrive::setVelocity(double velocity, bool offset)
{
	if (offset)
	{
		rpdo_->velocityOffset = DS4toDrive(velocity);
	}
	else
	{
		rpdo_->targetVelocity = DS4toDrive(velocity);
	}
}

void AxisDrive::setPosition(double position)
{
	rpdo_->targetPosition = (int32)(position);
}

double AxisDrive::getActualCurrent()
{
	return DrivetoDC2(tpdo_->actualCurrent);
}

double AxisDrive::getActualVelocity()
{
	return DrivetoDS4(tpdo_->actualVelocity);
}

double AxisDrive::getActualPosition()
{
	return (double)tpdo_->actualPosition;
}

bool AxisDrive::readConversionConstants()
{
/*
KB	DC Bus Voltage in volts. This value can be read from 200F.01h.
KDS	Maximum dynamic index speed (in counts/s). This value can be read from 20CA.07h, 20CA.08h, 20CA.09h, and 20CA.0Ah.
KI	Feedback interpolation value. Only applies to drives that support 1 Vpp Sin/Cos feedback. For all other drives, KI = 1. When applicable, this value can be read from 2032.08h.
KMS	Maximum profiler speed (in counts/s) for an Accel/Decel command profile. This value can be read from 203C.09h for Configuration 0 and 203C.0Ch for Configuration 1.
KOV	The hardware defined, DC bus, over-voltage limit of the drive in volts. This value can be read from 20D8.09h.
KP	The maximum rated peak current of the drive in amps. For example, 20 for the DPRALTE-020B080. This value can be read from 20D8.0Ch.
KS	Switching frequency of the drive in Hz. Most drives have a switching frequency of 20 kHz (KS = 20,000), however, drives that operate beyond 400 V usually have a switching frequency of 10 kHz (KS = 10,000). This value can be read from 20D8.24h (in kHz).
*/
	int size = sizeof(conv_KB);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x200F, 0x01, FALSE, &size, &conv_KB, EC_TIMEOUTRXM);

	size = sizeof(conv_KDS0);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x20CA, 0x07, FALSE, &size, &conv_KDS0, EC_TIMEOUTRXM);
	size = sizeof(conv_KDS1);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x20CA, 0x08, FALSE, &size, &conv_KDS1, EC_TIMEOUTRXM);
	size = sizeof(conv_KDS2);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x20CA, 0x09, FALSE, &size, &conv_KDS2, EC_TIMEOUTRXM);
	size = sizeof(conv_KDS3);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x20CA, 0x0A, FALSE, &size, &conv_KDS3, EC_TIMEOUTRXM);

	// DC Bus Over Voltage in V in PBV units (scaling factor 10)
	size = sizeof(conv_KOV_PBV);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x20D8, 0x09, FALSE, &size, &conv_KOV_PBV, EC_TIMEOUTRXM);
	conv_KOV_V = (double)conv_KOV_PBV/10.0;

	// Maximum Peak Current in A in PBC units (scaling factor 10)
	size = sizeof(conv_KP_PBC);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x20D8, 0x0C, FALSE, &size, &conv_KP_PBC, EC_TIMEOUTRXM);
	conv_KP_A = (double)conv_KP_PBC/10.0;

	// read switching frequency in kHz in PBF units (scaling factor 2^16)
	size = sizeof(conv_KS_PBF);
	ecx_SDOread(contextt_ptr_, slave_id_, 0x20D8, 0x24, FALSE, &size, &conv_KS_PBF, EC_TIMEOUTRXM);
	conv_KS_HZ = 1000.0*(double)conv_KS_PBF/(double)0x10000;

	bool result = true;

	if (0 == conv_KS_HZ)
	{
		RTT::log(RTT::Error) << "AxisDrive::readConversionConstants: conv_KS_HZ == 0" << RTT::endlog();
		conv_DA1 = 1.0;
		conv_DS4 = 1.0;
		result = false;
	}
	else
	{
		// DA1: 2^34/(Ks^2)
		conv_DA1 = ((double)(1<<30))/(conv_KS_HZ*conv_KS_HZ/16.0);

		// DS4: 2^17/Ks
		conv_DS4 = ((double)(1<<17))/conv_KS_HZ;
	}

	if (0 == conv_KP_A)
	{
		RTT::log(RTT::Error) << "AxisDrive::readConversionConstants: conv_KP_A == 0" << RTT::endlog();
		conv_DC2 = 1.0;
		result = false;
	}
	else
	{
		// DC2: 2^15/Kp
		conv_DC2 = ((double)(1<<15))/conv_KP_A;
	}
	return result;
}

int32 AxisDrive::DA1toDrive(double da1)
{
	return (int32)(da1*conv_DA1);
}

double AxisDrive::DrivetoDA1(int32 drive)
{
	return (double)drive/conv_DA1;
}

int32 AxisDrive::DS4toDrive(double ds4)
{
	return (int32)(ds4*conv_DS4);
}

double AxisDrive::DrivetoDS4(int32 drive)
{
	return (double)drive/conv_DS4;
}

int16 AxisDrive::DC2toDrive(double dc2)
{
	return (int32)(dc2*conv_DC2);
}

double AxisDrive::DrivetoDC2(int16 drive)
{
	return (double)drive/conv_DC2;
}


