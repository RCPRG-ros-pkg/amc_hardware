#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <rtt/RTT.hpp>

#include <rtt/Component.hpp>
#include <ros/ros.h>

#include "EthercatUtils.h"
#include "AxisDrive.h"

using namespace std;

class AmcDriver : public RTT::TaskContext{
private:

	std::string modeStr_;
	std::string ethStr_;
	AxisDrive::Mode mode_;

	RTT::InputPort<std::vector<double> > port_cmd_current_;
	RTT::OutputPort<std::vector<double> > port_current_;
	RTT::InputPort<std::vector<double> > port_cmd_velocity_;
	RTT::OutputPort<std::vector<double> > port_velocity_;
	RTT::InputPort<std::vector<double> > port_cmd_position_;
	RTT::OutputPort<std::vector<double> > port_position_;
	std::vector<double> cmd_current_;
	std::vector<double> current_;
	std::vector<double> cmd_velocity_;
	std::vector<double> velocity_;
	std::vector<double> cmd_position_;
	std::vector<double> position_;

	int current_watchdog_;
	int velocity_watchdog_;
	const int WATCHDOG_VALUE;

	bool initialised_;
	int slave_count_;
	AxisDrive *drives_;

	std::vector<bool> homing_;
	enum State { NoOp, Operation };
	State state_;

public:

	bool startHoming(int slaveNum)
	{
		if (!initialised_)
		{
			RTT::log(RTT::Error) << "The driver is not Configured." << RTT::endlog();
			return false;
		}

		if (1>slaveNum || slave_count_<slaveNum)
		{
			RTT::log(RTT::Error) << "startHoming: wrong slaveNum: " << slaveNum << RTT::endlog();
			return false;
		}

		if (NoOp != state_)
		{
			RTT::log(RTT::Error) << "startHoming: wrong state." << RTT::endlog();
			return false;
		}

		if (homing_[slaveNum-1])
		{
			RTT::log(RTT::Error) << "startHoming: already running for slave " << slaveNum << RTT::endlog();
			return false;
		}

		if (!drives_[slaveNum-1].beginHoming())
		{
			RTT::log(RTT::Error) << "startHoming: could not start for slave " << slaveNum << RTT::endlog();
			return false;
		}

		homing_[slaveNum-1] = true;
		return true;
	}

	bool isHomingComplete(int slaveNum)
	{
		if (!initialised_)
		{
			RTT::log(RTT::Error) << "The driver is not Configured." << RTT::endlog();
			return false;
		}

		if (1>slaveNum || slave_count_<slaveNum)
		{
			RTT::log(RTT::Error) << "isHomingComplete: wrong slaveNum: " << slaveNum << RTT::endlog();
			return false;
		}
		bool completed = drives_[slaveNum-1].isHomingCompleted();
		if (completed)
			homing_[slaveNum-1] = false;
		return completed;
	}

	bool beginOperation()
	{
		if (NoOp != state_)
		{
			RTT::log(RTT::Error) << "beginOperation: wrong state." << RTT::endlog();
			return false;
		}

		if (!initialised_)
		{
			RTT::log(RTT::Error) << "The driver is not Configured." << RTT::endlog();
			return false;
		}

		for (int i=0; i<slave_count_; ++i)
		{
			if (homing_[i])
			{
				RTT::log(RTT::Error) << "beginOperation: slave " << (i+1) << " is in homing state." << RTT::endlog();
				return false;
			}
		}

		for (int i=0; i<slave_count_; ++i)
		{
			drives_[i].enterMode(mode_);
		}

		state_ = Operation;
		ethercatEnterOperationalStateNoBlock();

		return true;
	}

	AmcDriver(const std::string& name):
		TaskContext(name),
		initialised_(false),
		drives_(NULL),
		state_(NoOp),
		mode_(AxisDrive::CyclicSynchronousTorque),
		current_watchdog_(0),
		velocity_watchdog_(0),
		WATCHDOG_VALUE(200)
	{
		this->addProperty("mode", modeStr_);
		this->addProperty("device_name", ethStr_);

		this->addPort("joint_current_command",	port_cmd_current_).doc("Sets desired current or feedforward current.");
		this->addPort("joint_current",		port_current_).doc("Sends out measured current.");
		this->addPort("joint_velocity_command",	port_cmd_velocity_).doc("Sets desired velocity or feedforward velocity.");
		this->addPort("joint_velocity",		port_velocity_).doc("Sends out measured velocity.");
		this->addPort("joint_position_command",	port_cmd_position_).doc("Sets desired position.");
		this->addPort("joint_position",		port_position_).doc("Sends out measured position.");

		this->addOperation("startHoming",	&AmcDriver::startHoming, this, RTT::OwnThread);
		this->addOperation("isHomingComplete",	&AmcDriver::isHomingComplete, this, RTT::OwnThread);
		this->addOperation("beginOperation",	&AmcDriver::beginOperation, this, RTT::OwnThread);
	}

	~AmcDriver()
	{
	}

	// RTT configure hook
	bool configureHook()
	{
		if (0 == modeStr_.compare("current"))
		{
			mode_ = AxisDrive::CyclicSynchronousTorque;
		}
		else if (0 == modeStr_.compare("velocity"))
		{
			mode_ = AxisDrive::CyclicSynchronousVelocity;
		}
		else if (0 == modeStr_.compare("position"))
		{
			mode_ = AxisDrive::CyclicSynchronousPosition;
		}
		else
		{
			RTT::log(RTT::Error) << "Operation mode parameter is not set or has a wrong value: " << modeStr_ << RTT::endlog();
			return false;
		}

		if (ethercatInit(ethStr_.c_str()))
		{
			slave_count_ = getSlaveCount();
			if (slave_count_>0)
			{
				RTT::log(RTT::Info) << "Found "<<slave_count_<<" slaves on ethercat bus." << RTT::endlog();
				current_.resize(slave_count_);
				velocity_.resize(slave_count_);
				position_.resize(slave_count_);
				homing_.resize(slave_count_);
				drives_ = new AxisDrive[slave_count_];
				for (int i=0; i<slave_count_; ++i)
				{
					if (!drives_[i].init(i+1))
					{
						RTT::log(RTT::Error) << "Could not initialise slave " << (i+1) << RTT::endlog();
						return false;
					}
					homing_[i] = false;
				}
				initialised_ = true;
			}
			else
			{
				RTT::log(RTT::Error) << "No slaves found on ethercat bus." << RTT::endlog();
				return false;
			}
		}
		else
		{
			RTT::log(RTT::Error) << "Could not initialise ethercat bus." << RTT::endlog();
			return false;
		}

		if (!ethercatEnterSafeOpState())
		{
			RTT::log(RTT::Error) << "Could not change ethercat state to SafeOp." << RTT::endlog();
		}

		for (int i=0; i<slave_count_; ++i)
		{
			drives_[i].configureHoming();
		}

		return true;
	}

	void cleanupHook()
	{
		for (int i=0; i<slave_count_; ++i)
		{
			drives_[i].enterState(AxisDrive::DisableVoltage);
		}

		ethercatSend();
		ethercatReceive();

		if (!ethercatEnterSafeOpState())
		{
			RTT::log(RTT::Error) << "Could not change ethercat state to SafeOp." << RTT::endlog();
		}

		if (NULL != drives_)
		{
			delete[] drives_;
		}

		if (initialised_)
		{
			ethercatClose();
			initialised_ = false;
		}
	}

	// RTT start hook
	bool startHook()
	{
	}

	void stopHook()
	{
	}

	void switchAllSlavesToReadyToSwitchOn()
	{
		for (int i=0; i<slave_count_; ++i)
		{
			if (drives_[i].getState() == AxisDrive::SwitchOnDisabled)
			{
				// go to ReadyToSwitchOn state
				drives_[i].enterState(AxisDrive::ShutDown);
			}
		}
	}

	void switchAllSlavesToEnableOperation()
	{
		for (int i=0; i<slave_count_; ++i)
		{
			if (drives_[i].getState() == AxisDrive::SwitchOnDisabled)
			{
				// go to ReadyToSwitchOn state
				drives_[i].enterState(AxisDrive::ShutDown);
			}
			else if (drives_[i].getState() == AxisDrive::ReadyToSwitchOn)
			{
				// go to OperationDisabled state
				drives_[i].enterState(AxisDrive::SwitchOn);
			}
			else if (drives_[i].getState() == AxisDrive::SwitchedOn)
			{
				// go to OperationEnabled state
				drives_[i].enterState(AxisDrive::EnableOperation);
			}
		}
	}

	void processCurrentCommand(bool offset)
	{
		if (RTT::NoData != port_cmd_current_.read(cmd_current_))
		{
			if( cmd_current_.size() == slave_count_)
			{
				current_watchdog_ = 0;
				for (unsigned int i = 0; i < 1; i++)
				{
					drives_[i].setCurrent(cmd_current_[i], offset);
				}
			}
			else
			{
				RTT::log(RTT::Error) << "No valid port values: vector size " << cmd_current_.size() << ", should be " << slave_count_ << RTT::endlog();
			}
		}
		else
		{
			if (current_watchdog_ < WATCHDOG_VALUE)
			{
				++current_watchdog_;
			}
			else
			{
				for (unsigned int i = 0; i < 1; i++)
				{
					drives_[i].setCurrent(0, offset);
				}
				RTT::log(RTT::Warning) << "No current data." << RTT::endlog();
			}
		}
	}

	void processVelocityCommand(bool offset)
	{
		if (RTT::NoData != port_cmd_velocity_.read(cmd_velocity_))
		{
			if( cmd_velocity_.size() == slave_count_)
			{
				velocity_watchdog_ = 0;
				for (unsigned int i = 0; i < 1; i++)
				{
					drives_[i].setVelocity(cmd_velocity_[i], offset);
				}
			}
			else
			{
				RTT::log(RTT::Error) << "No valid port values: vector size " << cmd_velocity_.size() << ", should be " << slave_count_ << RTT::endlog();
			}
		}
		else
		{
			if (velocity_watchdog_ < WATCHDOG_VALUE)
			{
				++velocity_watchdog_;
			}
			else
			{
				for (unsigned int i = 0; i < 1; i++)
				{
					drives_[i].setVelocity(0, offset);
				}
				RTT::log(RTT::Warning) << "No velocity data." << RTT::endlog();
			}
		}
	}

	void processPositionCommand()
	{
		if (RTT::NoData != port_cmd_position_.read(cmd_position_))
		{
			if( cmd_position_.size() == slave_count_)
			{
				for (unsigned int i = 0; i < 1; i++)
				{
					drives_[i].setPosition(cmd_position_[i]);
				}
			}
			else
			{
				RTT::log(RTT::Error) << "No valid port values: vector size " << cmd_position_.size() << ", should be " << slave_count_ << RTT::endlog();
			}
		}
	}

	// RTT update hook
	// This function runs every 1 ms (1000 Hz).
	void updateHook()
	{
		if (	AxisDrive::CyclicSynchronousTorque != mode_ &&
			AxisDrive::CyclicSynchronousVelocity != mode_ &&
			AxisDrive::CyclicSynchronousPosition != mode_)
		{
			return;
		}

		ethercatReceive();

		if (NoOp == state_)
		{
			switchAllSlavesToReadyToSwitchOn();
		}
		else
		{
			switchAllSlavesToEnableOperation();
			if (AxisDrive::CyclicSynchronousTorque == mode_)
			{
				processCurrentCommand(false);
			}
			else if (AxisDrive::CyclicSynchronousVelocity == mode_)
			{
				processCurrentCommand(true);
				processVelocityCommand(false);
			}
			else if (AxisDrive::CyclicSynchronousPosition == mode_)
			{
				processCurrentCommand(true);
				processVelocityCommand(true);
				processPositionCommand();
			}
		}
		for (int i=0; i<slave_count_; ++i)
		{
			current_[i] = drives_[i].getActualCurrent();
			velocity_[i] = drives_[i].getActualVelocity();
			position_[i] = drives_[i].getActualPosition();

		}
		port_current_.write(current_);
		port_velocity_.write(velocity_);
		port_position_.write(position_);
		ethercatSync();
		ethercatSend();
	}

};
ORO_CREATE_COMPONENT(AmcDriver)


