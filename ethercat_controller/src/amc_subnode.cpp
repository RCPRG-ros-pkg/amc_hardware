


#include <ethercattype.h>
#include <nicdrv.h>
#include <ethercatbase.h>
#include <ethercatmain.h>
#include <ethercatconfig.h>
#include <ethercatcoe.h>
#include <ethercatdc.h>
#include <ethercatprint.h>

#include "amc_subnode.h"

#include <native/task.h>
#include <rtt/RTT.hpp>

#define SUBNODE_ADDR(addr) (addr + subnode_id_*0x0800)

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

AMCSubNode::AMCSubNode(const std::string &name, ecx_contextt* contextt_ptr, uint16_t slave_id, int subnode_id) :
  service_(new RTT::Service(name)),
  subnode_id_(subnode_id),
  slave_id_(slave_id),
  contextt_ptr_(contextt_ptr),
	conv_DA1_(1.0),
	conv_DS4_(1.0),
	conv_DC2_(1.0),
	rpdo_ptr_(NULL),
	tpdo_ptr_(NULL),
	homing_compleat_(false),
	servo_state_(0),
	mode_(CyclicSynchronousPosition),
	enable_(false),
	homing_(false),
	position_valid_(false),
	name_(name)
{
	this->provides()->addConstant("position_valid", position_valid_);
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
	
	this->provides()->addOperation("beginHoming",	&AMCSubNode::beginHoming, this, RTT::OwnThread);
	this->provides()->addOperation("enableOperation",	&AMCSubNode::enableOperation, this, RTT::OwnThread);
}

AMCSubNode::~AMCSubNode() {
}

void AMCSubNode::setRPDOPtr(uint8_t * ptr) {
  rpdo_ptr_ = (RPDO_t *)ptr;
  memset(rpdo_ptr_, 0, getRPDOSize());
}


void AMCSubNode::setTPDOPtr(uint8_t * ptr) {
  tpdo_ptr_ = (TPDO_t *)ptr;
  memset(tpdo_ptr_, 0, getTPDOSize());
}

size_t AMCSubNode::getRPDOSize() {
  if (mode_ == CyclicSynchronousPosition) {
		return sizeof(RPDO_pos);
	} else if (mode_ == CyclicSynchronousVelocity) {
		return sizeof(RPDO_vel);
	} else if (mode_ == CyclicSynchronousTorque) {
		return sizeof(RPDO_cur);
	}
}

size_t AMCSubNode::getTPDOSize() {
  return sizeof(TPDO_t);
}

bool AMCSubNode::isHomingCompleted() {
  if (actualMode_ == Homing) {
	  return (tpdo_ptr_->status_word & SW_HomingComplete_Mask)!=0;
	} else {
	  return false;
	}
}

void AMCSubNode::setControlMode(Mode mode) {
	if (mode_ == CyclicSynchronousPosition) {
		rpdo_ptr_->position.mode_of_operation = (uint16_t)mode;
	} else if (mode_ == CyclicSynchronousVelocity) {
		rpdo_ptr_->velocity.mode_of_operation = (uint16_t)mode;
	} else if (mode_ == CyclicSynchronousTorque) {
		rpdo_ptr_->current.mode_of_operation = (uint16_t)mode;
	}
  actualMode_ = mode;
}

bool AMCSubNode::enableOperation() {
  if (position_valid_) {
    enable_ = true;
    return true;
  } else {
    return false;
  }
}

void AMCSubNode::processStatus() {
  homing_compleat_ = isHomingCompleted();
  servo_state_ = getState();
}

AMCSubNode::Mode AMCSubNode::getInitMode() {
  uint16_t mode;
  int size = sizeof(mode);
  ecx_SDOread(contextt_ptr_, slave_id_, SUBNODE_ADDR(0x20E6), 0x01, FALSE, &size, &mode, EC_TIMEOUTRXM);
  return (Mode)mode;
} 

bool AMCSubNode::configure() {
  mode_ = getInitMode();

  if (mode_ == CyclicSynchronousPosition) {
    mode_param_ = "position";
  } else if (mode_ == CyclicSynchronousVelocity) {
    mode_param_ = "velocity";
  } else if (mode_ == CyclicSynchronousTorque) {
    mode_param_ = "current";
  } else {
    return false;
  }

	if (!readConversionConstants()) {
		RTT::log(RTT::Error) << "AMCSubNode::init: could not read conversion constants" << RTT::endlog();
		return false;
	}

	uint8 mantissa;
	int8 exponent;
	int size;
	size = sizeof(mantissa);
	ecx_SDOread(contextt_ptr_, slave_id_, SUBNODE_ADDR(0x60C2), 0x01, FALSE, &size, &mantissa, EC_TIMEOUTRXM);

	size = sizeof(exponent);
	ecx_SDOread(contextt_ptr_, slave_id_, SUBNODE_ADDR(0x60C2), 0x02, FALSE, &size, &exponent, EC_TIMEOUTRXM);
	RTT::log(RTT::Info) << "AMCSubNode::init: Interpolation Time Period: " << (int)mantissa << "*10^" << (int)exponent << RTT::endlog();
	mantissa = 1;
	exponent = -3;
	size = sizeof(exponent);
	ecx_SDOwrite(contextt_ptr_, slave_id_, SUBNODE_ADDR(0x60C2), 0x02, FALSE, size, &exponent, EC_TIMEOUTRXM);
	size = sizeof(mantissa);
	ecx_SDOwrite(contextt_ptr_, slave_id_, SUBNODE_ADDR(0x60C2), 0x01, FALSE, size, &mantissa, EC_TIMEOUTRXM);

  uint16_t stat;
  size = sizeof(stat);
  
  ecx_SDOread(contextt_ptr_, slave_id_, SUBNODE_ADDR(0x2002), 0x07, FALSE, &size, &stat, EC_TIMEOUTRXM);

  position_valid_ = (stat & 1) == 1;
	return true;
}

bool AMCSubNode::start() {
  setControlMode(mode_);
  enterStateNoCheck(ShutDown);
  if (position_valid_ && (mode_ == CyclicSynchronousPosition)) {
    int size;
    int32_t pos;
    size = sizeof(pos);
    ecx_SDOread(contextt_ptr_, slave_id_, SUBNODE_ADDR(0x6064), 0x00, FALSE, &size, &pos, EC_TIMEOUTRXM);
    setPosition(pos);
  }
  return true;
}

void AMCSubNode::update() {
  double current, velocity, position;
  
  processStatus();
  
  if (getState() != AMCSubNode::OperationEnabled) {
    setPosition(tpdo_ptr_->actual_position);
  }
  
  if (getState() == AMCSubNode::Fault) {
		// go to ReadyToSwitchOn state
		RTT::log(RTT::Warning) << "Fault" << RTT::endlog();
		target_state_ = ResetFault;
	} else if (getState() == AMCSubNode::QuickStopActive) {
		// go to ReadyToSwitchOn state
		RTT::log(RTT::Warning) << "QuickStop" << RTT::endlog();
		target_state_ = DisableVoltage;
	} else if (getState() == AMCSubNode::NotReadyToSwitchOn) {
		// go to ReadyToSwitchOn state
		//RTT::log(RTT::Warning) << "NotReadyToSwitchOn" << RTT::endlog();
		target_state_ = ShutDown;
	} else if (getState() == AMCSubNode::SwitchOnDisabled) {
		// go to ReadyToSwitchOn state
		//RTT::log(RTT::Warning) << "ReadyToSwitchOn" << RTT::endlog();
		target_state_ = ShutDown;
	} else if (getState() == AMCSubNode::ReadyToSwitchOn) {
		// go to OperationDisabled state
		//RTT::log(RTT::Warning) << "OperationDisabled" << RTT::endlog();
		setPosition(tpdo_ptr_->actual_position);
		target_state_ = SwitchOn;
	} else if (getState() == AMCSubNode::SwitchedOn) {
	  //RTT::log(RTT::Warning) << "SwitchedOn" << RTT::endlog();
	  
	  setPosition(tpdo_ptr_->actual_position);
	  
	  if (position_valid_) {
	    RTT::log(RTT::Warning) << "enable" << RTT::endlog();
	    setControlMode(mode_);
	    target_state_ = EnableOperation;
	  } else {
	    RTT::log(RTT::Warning) << "homing" << RTT::endlog();
	    setControlMode(Homing);
	    target_state_ = BeginHoming;
	    homing_ = true;
	  }
	} else if (getState() == AMCSubNode::OperationEnabled) {
	  //RTT::log(RTT::Warning) << "OperationEnabled" << RTT::endlog();
	  if (position_valid_) {
	    if (port_motor_position_command_.read(position) == RTT::NewData) {
        setPosition(position);
      }

      if (port_motor_velocity_command_.read(velocity) == RTT::NewData) {
        setVelocity(velocity);
      }
    
      if (port_motor_current_command_.read(current) == RTT::NewData) {
        setCurrent(current);
      }
      
      // dio
      
      bool dio;
      
      if (port_do1_command_.read(dio) == RTT::NewData) {
        if (dio) {
          setDO(0);
        } else {
          resetDO(0);
        }
      }
      
      if (port_do2_command_.read(dio) == RTT::NewData) {
        if (dio) {
          setDO(1);
        } else {
          resetDO(1);
        }
      }
      
      if (port_do3_command_.read(dio) == RTT::NewData) {
        if (dio) {
          setDO(2);
        } else {
          resetDO(2);
        }
      }
      
      if (port_do4_command_.read(dio) == RTT::NewData) {
        if (dio) {
          setDO(3);
        } else {
          resetDO(3);
        }
      }
      
	  } else {
	    if ((actualMode_ == Homing)) {
	      if (isHomingCompleted()) {
	        position_valid_ = true;
	        setPosition(tpdo_ptr_->actual_position);
	        setControlMode(mode_);
	        target_state_ = EnableOperation;
	      } else {
	        target_state_ = BeginHoming;
        }
	    } else {
	      setControlMode(Homing);
	      target_state_ = BeginHoming;
      }
	  }
  }
  
  enterStateNoCheck(target_state_);
  
  if (position_valid_) {
    port_motor_position_.write(getActualPosition());
    port_motor_velocity_.write(getActualVelocity());
    port_motor_current_.write(getActualCurrent());
  }
}

void AMCSubNode::stop() {
  enterState(AMCSubNode::ShutDown);
}

AMCSubNode::DriveState AMCSubNode::getState() {
	uint16 status = 0;
  status = tpdo_ptr_->status_word;

	return translateState(status);
}

AMCSubNode::DriveState AMCSubNode::translateState(uint16 statusWord) {
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

bool AMCSubNode::enterState(AMCSubNode::StateCommand cmd) {
	DriveState old = getState();

	if (old == SwitchOnDisabled && cmd == ShutDown) {
		return enterStateNoCheck(cmd);
	}
	else if (old == ReadyToSwitchOn && (cmd == DisableVoltage || cmd == SwitchOn)) {
		return enterStateNoCheck(cmd);
	}
	else if (old == SwitchedOn && (cmd == DisableVoltage || cmd == ShutDown || cmd == EnableOperation)) {
		return enterStateNoCheck(cmd);
	}
	else if (old == OperationEnabled && (cmd == DisableVoltage || cmd == SwitchOn || cmd == QuickStop)) {
		return enterStateNoCheck(cmd);
	}
	else if (old == Fault && cmd == ResetFault) {
		return enterStateNoCheck(cmd);
	}
	else if (old == QuickStopActive && (cmd == DisableVoltage || cmd == EnableOperation)) {
		return enterStateNoCheck(cmd);
	}

	return false;
}

bool AMCSubNode::enterStateNoCheck(AMCSubNode::StateCommand cmd) {
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
	
	if (mode_ == CyclicSynchronousPosition) {
		controlWord = rpdo_ptr_->position.control_word;
	}
	else if (mode_ == CyclicSynchronousVelocity) {
		controlWord = rpdo_ptr_->velocity.control_word;
	}
	else if (mode_ == CyclicSynchronousTorque) {
		controlWord = rpdo_ptr_->current.control_word;
	} else {
	  return false;
	}
	
	switch (cmd)
	{
	case DisableVoltage:
	  //RTT::log(RTT::Warning) << "DisableVoltage" << RTT::endlog();
		controlWord = 0x00;
		break;
	case ShutDown:
	  //RTT::log(RTT::Warning) << "ShutDown" << RTT::endlog();
		controlWord = 0x06;
		break;
	case SwitchOn:
	  //RTT::log(RTT::Warning) << "SwitchOn" << RTT::endlog();
		controlWord = 0x07;
		break;
	case EnableOperation:
	  //RTT::log(RTT::Warning) << "EnableOperation" << RTT::endlog();
		controlWord = 0x0F;
		break;
	case ResetFault:
	  //RTT::log(RTT::Warning) << "ResetFault" << RTT::endlog();
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
	  //RTT::log(RTT::Warning) << "QuickStop" << RTT::endlog();
		controlWord = 0x02;
		break;
	case BeginHoming:
	  //RTT::log(RTT::Warning) << "BeginHoming" << RTT::endlog();
		controlWord = 0x1F;
		break;
	default:
		return false;
		break;
	}
  
  if (mode_ == CyclicSynchronousPosition) {
		rpdo_ptr_->position.control_word = controlWord;
	}
	else if (mode_ == CyclicSynchronousVelocity) {
		rpdo_ptr_->velocity.control_word = controlWord;
	}
	else if (mode_ == CyclicSynchronousTorque) {
		rpdo_ptr_->current.control_word = controlWord;
	} else {
	  return false;
	}
  
	return true;
}

bool AMCSubNode::beginHoming() {
	homing_ = true;
  return true;
}

AMCSubNode::Mode AMCSubNode::getMode()
{
	if (mode_ == CyclicSynchronousPosition) {
		return (Mode)rpdo_ptr_->position.mode_of_operation;
	} else if (mode_ == CyclicSynchronousVelocity) {
		return (Mode)rpdo_ptr_->velocity.mode_of_operation;
	} else if (mode_ == CyclicSynchronousTorque) {
		return (Mode)rpdo_ptr_->current.mode_of_operation;
	}
}

void AMCSubNode::setDO(unsigned int n) {
  uint16_t dio;
  if (mode_ == CyclicSynchronousPosition) {
		dio = rpdo_ptr_->position.user_bits;
	} else if (mode_ == CyclicSynchronousVelocity) {
		dio = rpdo_ptr_->velocity.user_bits;
	} else if (mode_ == CyclicSynchronousTorque) {
		dio = rpdo_ptr_->current.user_bits;
	}
  
  dio |= (1<<n);
  
  if (mode_ == CyclicSynchronousPosition) {
		rpdo_ptr_->position.user_bits = dio;
	} else if (mode_ == CyclicSynchronousVelocity) {
		rpdo_ptr_->velocity.user_bits = dio;
	} else if (mode_ == CyclicSynchronousTorque) {
		rpdo_ptr_->current.user_bits = dio;
	}
  
}

void AMCSubNode::resetDO(unsigned int n) {
  uint16_t dio;
  if (mode_ == CyclicSynchronousPosition) {
		dio = rpdo_ptr_->position.user_bits;
	} else if (mode_ == CyclicSynchronousVelocity) {
		dio = rpdo_ptr_->velocity.user_bits;
	} else if (mode_ == CyclicSynchronousTorque) {
		dio = rpdo_ptr_->current.user_bits;
	}
  
  dio &= ~(1<<n);
  
  if (mode_ == CyclicSynchronousPosition) {
		rpdo_ptr_->position.user_bits = dio;
	} else if (mode_ == CyclicSynchronousVelocity) {
		rpdo_ptr_->velocity.user_bits = dio;
	} else if (mode_ == CyclicSynchronousTorque) {
		rpdo_ptr_->current.user_bits = dio;
	}
  
}

void AMCSubNode::setCurrent(double current) {
	if (mode_ == CyclicSynchronousPosition) {
		rpdo_ptr_->position.current_offset = DC2toDrive(current);
	}
	else if (mode_ == CyclicSynchronousVelocity) {
		rpdo_ptr_->velocity.current_offset = DC2toDrive(current);
	}
	else if (mode_ == CyclicSynchronousTorque) {
		rpdo_ptr_->current.target_current = DC2toDrive(current);
	}
}

void AMCSubNode::setVelocity(double velocity) {
	if (mode_ == CyclicSynchronousPosition) {
		rpdo_ptr_->position.velocity_offset = DS4toDrive(velocity);
	}
	else if (mode_ == CyclicSynchronousVelocity) {
		rpdo_ptr_->velocity.target_velocity = DS4toDrive(velocity);
	}
}

void AMCSubNode::setPosition(double position) {
  if (mode_ == CyclicSynchronousPosition) {
	  rpdo_ptr_->position.target_position = (int32)(position);
	}
}

double AMCSubNode::getActualCurrent() {
	return DrivetoDC2(tpdo_ptr_->actual_current);
}

double AMCSubNode::getActualVelocity() {
	return DrivetoDS4(tpdo_ptr_->actual_velocity);
}

double AMCSubNode::getActualPosition() {
	return (double)tpdo_ptr_->actual_position;
}

bool AMCSubNode::readConversionConstants() {
/*
KB	DC Bus Voltage in volts. This value can be read from 200F.01h.
KDS	Maximum dynamic index speed (in counts/s). This value can be read from 20CA.07h, 20CA.08h, 20CA.09h, and 20CA.0Ah.
KI	Feedback interpolation value. Only applies to drives that support 1 Vpp Sin/Cos feedback. For all other drives, KI = 1. When applicable, this value can be read from 2032.08h.
KMS	Maximum profiler speed (in counts/s) for an Accel/Decel command profile. This value can be read from 203C.09h for Configuration 0 and 203C.0Ch for Configuration 1.
KOV	The hardware defined, DC bus, over-voltage limit of the drive in volts. This value can be read from 20D8.09h.
KP	The maximum rated peak current of the drive in amps. For example, 20 for the DPRALTE-020B080. This value can be read from 20D8.0Ch.
KS	Switching frequency of the drive in Hz. Most drives have a switching frequency of 20 kHz (KS = 20,000), however, drives that operate beyond 400 V usually have a switching frequency of 10 kHz (KS = 10,000). This value can be read from 20D8.24h (in kHz).
*/
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

	if (0 == conv_KS_HZ) {
		RTT::log(RTT::Error) << "AMCSubNode::readConversionConstants: conv_KS_HZ == 0" << RTT::endlog();
		conv_DA1_ = 1.0;
		conv_DS4_ = 1.0;
		result = false;
	} else {
		// DA1: 2^34/(Ks^2)
		conv_DA1_ = ((double)(1<<30))/(conv_KS_HZ*conv_KS_HZ/16.0);

		// DS4: 2^17/Ks
		conv_DS4_ = ((double)(1<<17))/conv_KS_HZ;
	}

	if (0 == conv_KP_A) {
		RTT::log(RTT::Error) << "AMCSubNode::readConversionConstants: conv_KP_A == 0" << RTT::endlog();
		conv_DC2_ = 1.0;
		result = false;
	} else {
		// DC2: 2^15/Kp
		conv_DC2_ = ((double)(1<<15))/conv_KP_A;
	}
	return result;
}

int32_t AMCSubNode::DA1toDrive(double da1)
{
	return (int32)(da1*conv_DA1_);
}

double AMCSubNode::DrivetoDA1(int32 drive)
{
	return (double)drive/conv_DA1_;
}

int32_t AMCSubNode::DS4toDrive(double ds4)
{
	return (int32)(ds4*conv_DS4_);
}

double AMCSubNode::DrivetoDS4(int32 drive)
{
	return (double)drive/conv_DS4_;
}

int16 AMCSubNode::DC2toDrive(double dc2)
{
	return (int32)(dc2*conv_DC2_);
}

double AMCSubNode::DrivetoDC2(int16 drive)
{
	return (double)drive/conv_DC2_;
}


